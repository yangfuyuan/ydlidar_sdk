/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     ydlidar_driver.cpp                                              *
*  @brief    lidar driver                                                    *
*  Details.                                                                  *
*                                                                            *
*  @author   Tony.Yang                                                       *
*  @email    chushuifurong618@eaibot.com                                     *
*  @version  1.3.7(版本号)                                                    *
*  @date     chushuifurong618@eaibot.com                                     *
*                                                                            *
*                                                                            *
*----------------------------------------------------------------------------*
*  Remark         : Description                                              *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2018/08/09 | 1.3.7     | Tony.Yang      | Create file                     *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/

#if defined(_WIN32)
#pragma warning (disable:4996);
#endif
#include "common.h"
#include "ydlidar_driver.h"
#include "timer.h"
#include <math.h>
using namespace impl;



namespace ydlidar{

   std::map<std::string, std::string> YDlidarDriver::lidar_map;


	YDlidarDriver::YDlidarDriver():
    _serial(nullptr), m_callback(nullptr) {
		isConnected = false;
		isScanning = false;
        //serial parameters
		m_intensities = false;
		isHeartbeat = false;
        isAutoReconnect = true;
        isAutoconnting = false;

        m_baudrate = 115200;
		isSupportMotorCtrl=true;
        m_sampling_rate=-1;
		model = -1;
        firmware_version = 0;
        node_counts = 720;
        scan_node_count = 0;
        m_last_ns       = 0;
        m_ns            = 0;

        //parse parameters
        PackageSampleBytes = 2;
        package_Sample_Index = 0;
        IntervalSampleAngle = 0.0;
        IntervalSampleAngle_LastPackage = 0.0;
        FirstSampleAngle = 0;
        LastSampleAngle = 0;
        CheckSun = 0;
        CheckSunCal = 0;
        SampleNumlAndCTCal = 0;
        LastSampleAngleCal = 0;
        CheckSunResult = true;
        Valu8Tou16 = 0;

        cfg_.autoReconnect = true;
        cfg_.exposure = false;
        cfg_.fixedResolution = false;
        cfg_.heartBeat = false;
        cfg_.ignoreArray.clear();
        cfg_.intensity = false;
        cfg_.maxAngle = 180;
        cfg_.minAngle = -180;
        cfg_.minRange = 0.1;
        cfg_.maxRange = 16.0;
        cfg_.reversion = false;
        cfg_.sampleRate = 9;
        cfg_.scanFrequency = 7;
        cfg_.serialBaudrate = 115200;
        cfg_.serialPort = "";

	}

	YDlidarDriver::~YDlidarDriver(){	

        isScanning = false;
        isAutoReconnect = false; 
        _thread.join();

        ScopedLocker lck(_serial_lock);
		if(_serial){
			if(_serial->isOpen()){
				_serial->close();
			}
		}
		if(_serial){
			delete _serial;
			_serial = NULL;
		}
	}

	result_t YDlidarDriver::connect(const char * port_path, uint32_t baudrate) {
        m_baudrate = baudrate;
        serial_port = string(port_path);
        ScopedLocker lck(_serial_lock);
		if(!_serial){
            _serial = new serial::Serial(port_path, m_baudrate, serial::Timeout::simpleTimeout(DEFAULT_TIMEOUT));
		}

		{
			if(!_serial->open()){
				return RESULT_FAIL;
			}
		}

		isConnected = true;
        {
            ScopedLocker l(_lock);
            sendCommand(LIDAR_CMD_FORCE_STOP);
            sendCommand(LIDAR_CMD_STOP);
        }
		clearDTR();

		return RESULT_OK;
	}

    std::vector<std::string> YDlidarDriver::getLidarList() {
        return lidarPortList();
    }

    std::vector<std::string> YDlidarDriver::lidarPortList() {
        std::vector<PortInfo> lst = list_ports();
        lidar_map.clear();
        std::vector<string> ports;
        for(std::vector<PortInfo>::iterator it = lst.begin(); it != lst.end(); it++) {
            std::string port = "ydlidar" + (*it).device_id;
            ports.push_back(port);
            lidar_map[port] = (*it).port;
        }
        return ports;
    }

    void YDlidarDriver::UpdateLidarParamCfg(const LaserParamCfg &config_msg) {
        ScopedLocker  lock(_cfg_lock);
        bool restart = false;

        if( config_msg.maxAngle < config_msg.minAngle) {
            throw DeviceException("The maximum Angle is greater than the minimum angel, please check the angle range settings.");
        }

        if(config_msg.serialPort.empty()) {
            throw DeviceException("The serial port is empty. please check the serial port settigns. ");
        }

        getLidarList();
        std::map<std::string,std::string>::iterator it;
        it = lidar_map.find(config_msg.serialPort);
        bool found = false;
        if(it != lidar_map.end()) {
            if(cfg_.serialPort != it->second) {
                restart = true;
                cfg_.serialPort = it->second;
            }
        }else {
            for(it = lidar_map.begin(); it != lidar_map.end(); it++) {
                if(config_msg.serialPort == it->second) {
                    if(cfg_.serialPort != it->second) {
                        restart = true;
                        cfg_.serialPort = it->second;
                        found = true;
                        break;
                    }
                }
            }

            if(!found) {

#if defined(_WIN32)
                cfg_.serialPort = config_msg.serialPort;
                restart = true;
#else
                if(config_msg.serialPort != "/dev/ydlidar") {
                    size_t pos = config_msg.serialPort.find("/dev/");
                    if(pos != std::string::npos) {
                        cfg_.serialPort = config_msg.serialPort;
                        restart = true;
                    }else {
                        throw DeviceException("The serial port is error. please check the serial port settigns. ");
                    }
                } else {
					cfg_.serialPort = config_msg.serialPort;
                    restart = true;
                }
#endif
            }
        }


        cfg_.autoReconnect = config_msg.autoReconnect;
        cfg_.fixedResolution = config_msg.fixedResolution;
        cfg_.ignoreArray = config_msg.ignoreArray;
        cfg_.intensity = config_msg.intensity;
        cfg_.maxAngle = config_msg.maxAngle;
        cfg_.minAngle = config_msg.minAngle;
        cfg_.minRange = config_msg.minRange;
        cfg_.maxRange = config_msg.maxRange;
        cfg_.reversion = config_msg.reversion;
        if(cfg_.exposure != config_msg.exposure) {
            cfg_.exposure = config_msg.exposure;
            restart = true;
        }
        if(cfg_.heartBeat != config_msg.heartBeat) {
            cfg_.heartBeat = config_msg.heartBeat;
            restart = true;
        }

        if( cfg_.sampleRate != config_msg.sampleRate) {
            cfg_.sampleRate = config_msg.sampleRate;
            restart = true;
        }
        if(cfg_.scanFrequency != config_msg.scanFrequency) {
            cfg_.scanFrequency = config_msg.scanFrequency;
            restart = true;
        }
        if( cfg_.serialBaudrate != config_msg.serialBaudrate) {
            cfg_.serialBaudrate = config_msg.serialBaudrate;
            restart = true;
        }

        if(restart) {
            if(!checkComms()) {
                throw DeviceException("Serial port is connection failed.");
            }
            bool ret = checkDeviceHealth();
            if(!ret) {
                delay(1000);
            }
            if(!checkDeviceInfo()&&!ret) {
                delay(1000);
                if(!checkDeviceInfo()){
                    throw DeviceInformationException("check lidar device information and health error.");
                }
            }


        }else {
            if(!isconnected()) {
                if(!checkComms()) {
                    throw DeviceException("Serial port is connection failed.");
                }
            }


        }

        setIntensities(cfg_.intensity);
        if(!isscanning()) {
            result_t s_result= startScan();
            if (s_result != RESULT_OK) {
                s_result= startScan();
                if(s_result != RESULT_OK) {
                    isScanning = false;
                    throw DeviceException("Starting scanning failed.");
                }

            }
            delay(50);
        }
        printf("start scanning.....\n");
        setAutoReconnect(cfg_.autoReconnect);

    }

    void YDlidarDriver::RegisterLIDARDataCallback(LIDARDriverDataCallback callback) {
        m_callback = callback;
    }

    void YDlidarDriver::spinOnce() {

        node_info nodes[2048];
        size_t   count = _countof(nodes);

        result_t ans = grabScanData(nodes, count);
        if(ans == RESULT_OK) {
            ans = ascendScanData(nodes, count);
            if(ans == RESULT_OK) {
                ScopedLocker lock(_cfg_lock);
                size_t all_nodes_counts;
                if(!cfg_.fixedResolution||!node_counts){
                    all_nodes_counts = count;
                } else {
                    all_nodes_counts = node_counts;
                }
                float each_angle = 360.0/all_nodes_counts;

                uint64_t max_time =nodes[0].stamp ;
                uint64_t min_time = nodes[0].stamp;
                float    lidar_frequency = nodes[0].scan_frequence /10.f;

                node_info *angle_compensate_nodes = new node_info[all_nodes_counts];
                memset(angle_compensate_nodes, 0, all_nodes_counts*sizeof(node_info));
                for(unsigned int i = 0 ; i < count; i++) {
                    if (nodes[i].distance_q != 0) {
                        float angle = (float)((nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                        if(cfg_.reversion){
                           angle=angle+180;
                           if(angle>=360){ angle=angle-360;}
                           nodes[i].angle_q6_checkbit = ((uint16_t)(angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT;
                        }
                        size_t inter =(int)( angle / each_angle );
                        float angle_pre = angle - inter * each_angle;
                        float angle_next = (inter+1) * each_angle - angle;
                        if (angle_pre < angle_next) {
                            if(inter < all_nodes_counts) {
                                angle_compensate_nodes[inter]=nodes[i];
                            }
                        } else {
                            if (inter < all_nodes_counts -1) {
                                angle_compensate_nodes[inter+1]=nodes[i];
                            }
                        }
                    }
                    if(nodes[i].scan_frequence != 0) {
                        lidar_frequency = nodes[i].scan_frequence/10.f;
                    }

                    if(nodes[i].stamp > max_time) {
                        max_time = nodes[i].stamp;
                    }
                    if(nodes[i].stamp < min_time) {
                        min_time = nodes[i].stamp;
                    }
                 }


                const double scan_time = max_time - min_time;
                LaserScan scan_msg;
                int counts = all_nodes_counts*((cfg_.maxAngle-cfg_.minAngle)/360.0f);
                int angle_start = 180+cfg_.minAngle;
                int node_start = all_nodes_counts*(angle_start/360.0f);
                scan_msg.ranges.resize(counts);
                scan_msg.intensities.resize(counts);
                float range = 0.0;
                float intensity = 0.0;
                int index = 0;

                scan_msg.system_time_stamp =  getTime();
                scan_msg.self_time_stamp = min_time;
                scan_msg.config.min_angle = DEG2RAD(cfg_.minAngle);
                scan_msg.config.max_angle = DEG2RAD(cfg_.maxAngle);
                scan_msg.config.ang_increment = (scan_msg.config.max_angle - scan_msg.config.min_angle) / (double)counts;
                scan_msg.config.time_increment = scan_time / (double)counts;
                scan_msg.config.scan_time = scan_time;
                scan_msg.config.min_range = cfg_.minRange;
                scan_msg.config.max_range = cfg_.maxRange;
                scan_msg.scan_frequency   = lidar_frequency;


                for (size_t i = 0; i < all_nodes_counts; i++) {
                    range = (float)(angle_compensate_nodes[i].distance_q)/1000.f;
                    intensity = (float)(angle_compensate_nodes[i].sync_quality >> LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
                    if (i<all_nodes_counts/2) {
                        index = all_nodes_counts/2-1-i;
                    } else {
                        index =all_nodes_counts-1-(i-all_nodes_counts/2);
                    }

                    if (cfg_.ignoreArray.size() != 0) {
                        float angle = (float)((angle_compensate_nodes[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                        if (angle>180) {
                            angle=360-angle;
                        } else {
                            angle=-angle;
                        }

                        for (uint16_t j = 0; j < cfg_.ignoreArray.size();j = j+2) {
                            if ((cfg_.ignoreArray[j] < angle) && (angle <= cfg_.ignoreArray[j+1])) {
                               range = 0.0;
                               break;
                            }
                        }
                    }

                    if (range > cfg_.maxRange|| range < cfg_.minRange) {
                        range = 0.0;
                    }

                    int pos = index - node_start ;
                    if (0 <= pos && pos < counts) {
                        scan_msg.ranges[pos] =  range;
                        scan_msg.intensities[pos] = intensity;
                    }
                }



                if(m_callback) {
                    m_callback(scan_msg);
                }
                delete[] angle_compensate_nodes;
            }
        }else {
            if( ans == RESULT_TIMEOUT) {
                throw TimeoutException("Get Lidar data timeout");
            }else {
                throw CorruptedDataException("Get Lidar data failed");
            }
        }

    }


	void YDlidarDriver::setDTR() {
		if (!isConnected){
			return ;
		}

		if(_serial){
			_serial->setDTR(1);
		}

	}

	void YDlidarDriver::clearDTR() {
		if (!isConnected){
			return ;
		}

		if(_serial){
			_serial->setDTR(0);
		}
	}

	result_t YDlidarDriver::startMotor() {
        ScopedLocker l(_lock);
		if(isSupportMotorCtrl){
			setDTR();
			delay(500);
		}else{
			clearDTR();
			delay(500);
		}
		return RESULT_OK;
	}

	result_t YDlidarDriver::stopMotor() {
        ScopedLocker l(_lock);
		if(isSupportMotorCtrl){
			clearDTR();
			delay(500);
		}else{
			setDTR();
			delay(500);
		}
		return RESULT_OK;
	}

	void YDlidarDriver::disconnect() {
        isAutoReconnect = false;
		if (!isConnected){
			return ;
		}
		stop();

        ScopedLocker lck(_serial_lock);
		if(_serial){
			if(_serial->isOpen()){
				_serial->close();
			}
		}
		isConnected = false;
	}


	void YDlidarDriver::disableDataGrabbing() {
		{

            if(isScanning) {
                isScanning = false;
                _cond.set();
            }
		}

        _thread.join();
	}

    bool YDlidarDriver::isscanning() const
	{
		return isScanning;
	}
    bool YDlidarDriver::isconnected() const
    {
        return isConnected;
    }

	result_t YDlidarDriver::sendCommand(uint8_t cmd, const void * payload, size_t payloadsize) {
		uint8_t pkt_header[10];
		cmd_packet * header = reinterpret_cast<cmd_packet * >(pkt_header);
		uint8_t checksum = 0;

		if (!isConnected) {
			return RESULT_FAIL;
		}
		if (payloadsize && payload) {
			cmd |= LIDAR_CMDFLAG_HAS_PAYLOAD;
		}

		header->syncByte = LIDAR_CMD_SYNC_BYTE;
		header->cmd_flag = cmd;
		sendData(pkt_header, 2) ;

		if ((cmd & LIDAR_CMDFLAG_HAS_PAYLOAD)&&payloadsize && payload) {
			checksum ^= LIDAR_CMD_SYNC_BYTE;
			checksum ^= cmd;
			checksum ^= (payloadsize & 0xFF);

			for (size_t pos = 0; pos < payloadsize; ++pos) {
				checksum ^= ((uint8_t *)payload)[pos];
			}

			uint8_t sizebyte = (uint8_t)(payloadsize);
			sendData(&sizebyte, 1);

			sendData((const uint8_t *)payload, sizebyte);

			sendData(&checksum, 1);
		}
		return RESULT_OK;
	}

	result_t YDlidarDriver::sendData(const uint8_t * data, size_t size) {
		if (!isConnected) {
			return RESULT_FAIL;
		}

		if (data == NULL || size ==0) {
			return RESULT_FAIL;
		}
		size_t r;
        while (size) {
            r = _serial->write(data, size);
            if( r < 1) {
                return RESULT_FAIL;
            }
            size -= r;
            data += r;
        }
        return RESULT_OK;

	}

	result_t YDlidarDriver::getData(uint8_t * data, size_t size) {
		if (!isConnected) {
			return RESULT_FAIL;
		}
		size_t r;
        while (size) {
            r = _serial->read(data, size);
            if (r < 1) {
                return RESULT_FAIL;
            }
            size -= r;
            data += r;

        }
        return RESULT_OK;

	}

	result_t YDlidarDriver::waitResponseHeader(lidar_ans_header * header, uint32_t timeout) {
		int  recvPos = 0;
		uint32_t startTs = getms();
		uint8_t  recvBuffer[sizeof(lidar_ans_header)];
		uint8_t  *headerBuffer = reinterpret_cast<uint8_t *>(header);
		uint32_t waitTime;

		while ((waitTime=getms() - startTs) <= timeout) {
			size_t remainSize = sizeof(lidar_ans_header) - recvPos;
			size_t recvSize;

			result_t ans = waitForData(remainSize, timeout - waitTime, &recvSize);
			if (ans != RESULT_OK){
				return ans;
			}

			if(recvSize > remainSize) recvSize = remainSize;

			ans = getData(recvBuffer, recvSize);
			if (ans == RESULT_FAIL){
				return RESULT_FAIL;
			}

			for (size_t pos = 0; pos < recvSize; ++pos) {
				uint8_t currentByte = recvBuffer[pos];
				switch (recvPos) {
				case 0:
					if (currentByte != LIDAR_ANS_SYNC_BYTE1) {
						continue;
					}
					break;
				case 1:
					if (currentByte != LIDAR_ANS_SYNC_BYTE2) {
						recvPos = 0;
						continue;
					}
					break;
				}
				headerBuffer[recvPos++] = currentByte;

				if (recvPos == sizeof(lidar_ans_header)) {
					return RESULT_OK;
				}
			}
		}
		return RESULT_FAIL;
	}

	result_t YDlidarDriver::waitForData(size_t data_count, uint32_t timeout, size_t * returned_size) {
		size_t length = 0;
		if (returned_size==NULL) {
			returned_size=(size_t *)&length;
		}
		return (result_t)_serial->waitfordata(data_count, timeout, returned_size);
	}

	int YDlidarDriver::cacheScanData() {
		node_info      local_buf[128];
		size_t         count = 128;
		node_info      local_scan[MAX_SCAN_NODES];
		size_t         scan_count = 0;
		result_t            ans;
		memset(local_scan, 0, sizeof(local_scan));
		waitScanData(local_buf, count);

		uint32_t start_ts = getms();
        uint32_t end_ts = start_ts;
        int timeout_count = 0;

		while(isScanning) {

			if ((ans=waitScanData(local_buf, count)) != RESULT_OK) {
                if (ans != RESULT_TIMEOUT || timeout_count>5) {
                    if(!isAutoReconnect) {//
                        fprintf(stderr, "exit scanning thread!!\n");
                        {

                            ScopedLocker lock(_lock);
                            isScanning = false;
                            _cond.set();
                        }
                        return RESULT_FAIL;
                        //throw DeviceException("wait scan data error for serial exception. exit scanning thread");
                    } else {//
                        isAutoconnting = true;
                        while (isAutoReconnect&&isAutoconnting) {
                            {
                                ScopedLocker lck(_serial_lock);
                                if(_serial){
                                    if(_serial->isOpen()){
                                        _serial->close();

                                    }
                                    delete _serial;
                                    _serial = NULL;
                                    isConnected = false;
                                }
                            }

                            while(isAutoReconnect&&connect(serial_port.c_str(), m_baudrate) != RESULT_OK){
                                delay(1000);
                            }
                            if(!isAutoReconnect) {
                                isScanning = false;
                                return RESULT_FAIL;
                            }
                            if(isconnected()) {
                                {
                                    ScopedLocker lck(_serial_lock);
                                    ans = startAutoScan();
                                }
                                if(ans == RESULT_OK){
                                    timeout_count =0;
                                    isAutoconnting = false;
                                    continue;
                                }

                            }
                        }


                    }

                } else {
                     timeout_count++;
                }
            }else {
                timeout_count = 0;
            }
			for (size_t pos = 0; pos < count; ++pos) {
                if (local_buf[pos].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT) {
                    if ((local_scan[0].sync_flag & LIDAR_RESP_MEASUREMENT_SYNCBIT)) {  
						_lock.lock();
                        memcpy(scan_node_buf, local_scan, scan_count*sizeof(node_info));
                        scan_node_count = scan_count;
                        _lock.unlock();
                        _cond.set();

					}
					scan_count = 0;
				}
				local_scan[scan_count++] = local_buf[pos];
				if (scan_count == _countof(local_scan)){
					scan_count-=1;
				}
			}

			//heartbeat function
            if (isHeartbeat) {
                end_ts = getms();
                if (end_ts - start_ts > DEFAULT_HEART_BEAT) {
                    sendHeartBeat();
                    start_ts = end_ts;
                }
            }
		}

		{
            ScopedLocker lock(_lock);
			isScanning = false;
            _cond.set();
		}
		return RESULT_OK;
	}

	result_t YDlidarDriver::waitPackage(node_info * node, uint32_t timeout) {
		int recvPos = 0;
		uint32_t startTs = getms();
		uint32_t size = (m_intensities)?sizeof(node_package):sizeof(node_packages);
		uint8_t* recvBuffer = new uint8_t[size];

		uint32_t waitTime = 0;
		uint8_t *packageBuffer = (m_intensities)?(uint8_t*)&package.package_Head:(uint8_t*)&packages.package_Head;
		uint8_t  package_Sample_Num = 0;
		int32_t AngleCorrectForDistance = 0;
		int  package_recvPos = 0;
        uint8_t package_type = 0;
        uint8_t scan_frequence = 0;

		(*node).scan_frequence = 0;
		if(package_Sample_Index == 0) {
			recvPos = 0;
			while ((waitTime=getms() - startTs) <= timeout) {
				size_t remainSize = PackagePaidBytes - recvPos;
				size_t recvSize;
				result_t ans = waitForData(remainSize, timeout-waitTime, &recvSize);
				if (ans != RESULT_OK){
					delete[] recvBuffer;
					return ans;
				}

				if (recvSize > remainSize){
					recvSize = remainSize;
				}

				getData(recvBuffer, recvSize);

				for (size_t pos = 0; pos < recvSize; ++pos) {
					uint8_t currentByte = recvBuffer[pos];
					switch (recvPos) {
					case 0:
						if(currentByte==(PH&0xFF)){

						}else{
							continue;
						}
						break;
					case 1:
						CheckSunCal = PH;
						if(currentByte==(PH>>8)){

                        } else {
							recvPos = 0;
							continue;
						}
						break;
					case 2:
						SampleNumlAndCTCal = currentByte;
                        package_type = currentByte&0x01;
                        if ((package_type == CT_Normal) || (package_type == CT_RingStart)){
                            if(package_type == CT_RingStart){
                                scan_frequence = (currentByte&0xFE)>>1;
                                (*node).scan_frequence = scan_frequence;
                            }
						} else {
							recvPos = 0;
							continue;
						}
						break;
					case 3:
						SampleNumlAndCTCal += (currentByte*0x100);
						package_Sample_Num = currentByte;
						break;
					case 4:
						if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
							FirstSampleAngle = currentByte;
						} else {
							recvPos = 0;
							continue;
						}
						break;
					case 5:
						FirstSampleAngle += currentByte*0x100;
						CheckSunCal ^= FirstSampleAngle;
						FirstSampleAngle = FirstSampleAngle>>1;
						break;
					case 6:
						if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
							LastSampleAngle = currentByte;
						} else {
							recvPos = 0;
							continue;
						}
						break;
					case 7:
						LastSampleAngle = currentByte*0x100 + LastSampleAngle;
						LastSampleAngleCal = LastSampleAngle;
						LastSampleAngle = LastSampleAngle>>1;
						if(package_Sample_Num == 1){
							IntervalSampleAngle = 0;
						}else{
							if(LastSampleAngle < FirstSampleAngle){
                                if((FirstSampleAngle >= 180*64) && (LastSampleAngle <= 180*64)){//实际雷达跨度不超过60度
									IntervalSampleAngle = (float)((360*64 + LastSampleAngle - FirstSampleAngle)/((package_Sample_Num-1)*1.0));
									IntervalSampleAngle_LastPackage = IntervalSampleAngle;
                                } else{//这里不应该发生
                                    if( FirstSampleAngle > 360) {///< 负数
                                        IntervalSampleAngle = ((float)(LastSampleAngle - ((int16_t)FirstSampleAngle)))/(package_Sample_Num-1);
                                    } else {//起始角大于结束角
                                        uint16_t temp = FirstSampleAngle;
                                        FirstSampleAngle = LastSampleAngle;
                                        LastSampleAngle = temp;
                                        IntervalSampleAngle = (float)((LastSampleAngle -FirstSampleAngle)/((package_Sample_Num-1)*1.0));
                                    }
                                     IntervalSampleAngle_LastPackage = IntervalSampleAngle;
                                    //IntervalSampleAngle = IntervalSampleAngle_LastPackage;
								}
							} else{
								IntervalSampleAngle = (float)((LastSampleAngle -FirstSampleAngle)/((package_Sample_Num-1)*1.0));
								IntervalSampleAngle_LastPackage = IntervalSampleAngle;
							}
						}
						break;
					case 8:
						CheckSun = currentByte;	
						break;
					case 9:
						CheckSun += (currentByte*0x100);
						break;
					}
					packageBuffer[recvPos++] = currentByte;
				}

				if (recvPos  == PackagePaidBytes ){
					package_recvPos = recvPos;
					break;
				}
			}

			if(PackagePaidBytes == recvPos){
				startTs = getms();
				recvPos = 0;
				while ((waitTime=getms() - startTs) <= timeout) {
					size_t remainSize = package_Sample_Num*PackageSampleBytes - recvPos;
					size_t recvSize;
					result_t ans =waitForData(remainSize, timeout-waitTime, &recvSize);
					if (ans != RESULT_OK){
						delete[] recvBuffer;
						return ans;
					}

					if (recvSize > remainSize){
						recvSize = remainSize;
					}

					getData(recvBuffer, recvSize);

					for (size_t pos = 0; pos < recvSize; ++pos) {
						if(m_intensities){
							if(recvPos%3 == 2){
								Valu8Tou16 += recvBuffer[pos]*0x100;
								CheckSunCal ^= Valu8Tou16;
							}else if(recvPos%3 == 1){
								Valu8Tou16 = recvBuffer[pos];
							}else{
								CheckSunCal ^= recvBuffer[pos]; 
							}
						}else{
							if(recvPos%2 == 1){
								Valu8Tou16 += recvBuffer[pos]*0x100;
								CheckSunCal ^= Valu8Tou16;
							}else{
								Valu8Tou16 = recvBuffer[pos];	
							}
						}				

						packageBuffer[package_recvPos+recvPos] = recvBuffer[pos];
						recvPos++;
					}



					if(package_Sample_Num*PackageSampleBytes == recvPos){
						package_recvPos += recvPos;
						break;
					}
				}
				if(package_Sample_Num*PackageSampleBytes != recvPos){
					delete[] recvBuffer;
					return RESULT_FAIL;
				}
			} else {
				delete[] recvBuffer;
				return RESULT_FAIL;
			}
			CheckSunCal ^= SampleNumlAndCTCal;
			CheckSunCal ^= LastSampleAngleCal;

			if(CheckSunCal != CheckSun){	
				CheckSunResult = false;
			}else{
				CheckSunResult = true;
			}

		}
		uint8_t package_CT;
		if(m_intensities){
			package_CT = package.package_CT;
		}else{
			package_CT = packages.package_CT;    
		}

		if(package_CT == CT_Normal){
            (*node).sync_flag =  Node_NotSync;
		} else{
            (*node).sync_flag =  Node_Sync;
		}
        (*node).sync_quality = Node_Default_Quality;


        if(CheckSunResult){
			if(m_intensities){
				(*node).sync_quality = (((package.packageSample[package_Sample_Index].PakageSampleDistance&0x03)<<LIDAR_RESP_MEASUREMENT_SYNC_QUALITY_SHIFT)| (package.packageSample[package_Sample_Index].PakageSampleQuality));
                (*node).distance_q = package.packageSample[package_Sample_Index].PakageSampleDistance >> LIDAR_RESP_MEASUREMENT_DISTANCE_SHIFT;
			}else{
                (*node).distance_q = packages.packageSampleDistance[package_Sample_Index] >> LIDAR_RESP_MEASUREMENT_DISTANCE_SHIFT;
			}	  
            if(((*node).distance_q) != 0){
                AngleCorrectForDistance = (int32_t)(((atan(((21.8*(155.3 - ((*node).distance_q)) )/155.3)/((*node).distance_q)))*180.0/3.1415) * 64.0);
			}else{
				AngleCorrectForDistance = 0;		
			}
			if((FirstSampleAngle + IntervalSampleAngle*package_Sample_Index + AngleCorrectForDistance) < 0){
				(*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + IntervalSampleAngle*package_Sample_Index + AngleCorrectForDistance + 360*64))<<1) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
			}else{
				if((FirstSampleAngle + IntervalSampleAngle*package_Sample_Index + AngleCorrectForDistance) > 360*64){
					(*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + IntervalSampleAngle*package_Sample_Index + AngleCorrectForDistance - 360*64))<<1) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
				}else{
					(*node).angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + IntervalSampleAngle*package_Sample_Index + AngleCorrectForDistance))<<1) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
				} 
			}
		}else{
            (*node).sync_flag = Node_NotSync;
            (*node).sync_quality = Node_Default_Quality;
			(*node).angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
			(*node).distance_q = 0;
		}


		uint8_t nowPackageNum;
		if(m_intensities){
			nowPackageNum = package.nowPackageNum;
		}else{
			nowPackageNum = packages.nowPackageNum;
		}

        if((*node).sync_flag&LIDAR_RESP_MEASUREMENT_SYNCBIT){
            m_last_ns = m_ns;
            m_ns = getTime() - (nowPackageNum*3 +10)*trans_delay - (nowPackageNum -1)*m_pointTime;
            if( (m_ns - m_last_ns )< 0) {
                m_ns = m_last_ns;
            }
		}
        (*node).stamp = m_ns  + package_Sample_Index*m_pointTime;
		package_Sample_Index++;

		if(package_Sample_Index >= nowPackageNum){
			package_Sample_Index = 0;
            m_ns= (*node).stamp + m_pointTime;
		}
		delete[] recvBuffer;
		return RESULT_OK;
	}

	result_t YDlidarDriver::waitScanData(node_info * nodebuffer, size_t & count, uint32_t timeout) {
		if (!isConnected) {
			count = 0;
			return RESULT_FAIL;
		}

		size_t     recvNodeCount =  0;
		uint32_t   startTs = getms();
        uint32_t   waitTime = 0;
		result_t ans;

		while ((waitTime = getms() - startTs) <= timeout && recvNodeCount < count) {
			node_info node;
			if ((ans = this->waitPackage(&node, timeout - waitTime)) != RESULT_OK) {
				return ans;
			}
			nodebuffer[recvNodeCount++] = node;

			if (recvNodeCount == count) {
				return RESULT_OK;
			}
		}
		count = recvNodeCount;
		return RESULT_FAIL;
	}

    result_t YDlidarDriver::grabScanData(node_info *nodebuffer, size_t &count, uint32_t timeout) {
        switch (_cond.wait(timeout)) {
            case Event::EVENT_TIMEOUT:
                count = 0;
                return RESULT_TIMEOUT;
            case Event::EVENT_OK:
            {
                if(scan_node_count == 0) {
                    return RESULT_FAIL;
                }
            	ScopedLocker lock(_lock);
                size_t size_to_copy = min(count, scan_node_count);
                memcpy(nodebuffer, scan_node_buf, size_to_copy*sizeof(node_info));
                count = size_to_copy;
                scan_node_count = 0;
            }
                return RESULT_OK;
            default:
                count = 0;
                return RESULT_FAIL;


        }
        return RESULT_FAIL;
    }


    static bool angleLessThan(const node_info& a, const node_info& b) {
        return a.angle_q6_checkbit < b.angle_q6_checkbit;
    }

	result_t YDlidarDriver::ascendScanData(node_info * nodebuffer, size_t count) {
		float inc_origin_angle = (float)360.0/count;
		int i = 0;

		for (i = 0; i < (int)count; i++) {
            if((nodebuffer[i].distance_q) == 0) {
				continue;
			} else {
				while(i != 0) {
					i--;
					float expect_angle = (nodebuffer[i+1].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f - inc_origin_angle;
					if (expect_angle < 0.0f) expect_angle = 0.0f;
					uint16_t checkbit = nodebuffer[i].angle_q6_checkbit & LIDAR_RESP_MEASUREMENT_CHECKBIT;
					nodebuffer[i].angle_q6_checkbit = (((uint16_t)(expect_angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
				}
				break;
			}
		}

		if (i == (int)count){
			return RESULT_FAIL;
		}

		for (i = (int)count - 1; i >= 0; i--) {
            if((nodebuffer[i].distance_q) == 0) {
				continue;
			} else {
				while(i != ((int)count - 1)) {
					i++;
					float expect_angle = (nodebuffer[i-1].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f + inc_origin_angle;
					if (expect_angle > 360.0f) expect_angle -= 360.0f;
					uint16_t checkbit = nodebuffer[i].angle_q6_checkbit & LIDAR_RESP_MEASUREMENT_CHECKBIT;
					nodebuffer[i].angle_q6_checkbit = (((uint16_t)(expect_angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
				}
				break;
			}
		}

		float frontAngle = (nodebuffer[0].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
		for (i = 1; i < (int)count; i++) {
            if((nodebuffer[i].distance_q) == 0) {
				float expect_angle =  frontAngle + i * inc_origin_angle;
				if (expect_angle > 360.0f) expect_angle -= 360.0f;
				uint16_t checkbit = nodebuffer[i].angle_q6_checkbit & LIDAR_RESP_MEASUREMENT_CHECKBIT;
				nodebuffer[i].angle_q6_checkbit = (((uint16_t)(expect_angle * 64.0f)) << LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + checkbit;
			}
		}

        size_t zero_pos = 0;
        float pre_degree = (nodebuffer[0].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;

        for (i = 1; i < (int)count ; ++i) {
            float degree = (nodebuffer[i].angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
            if (zero_pos == 0 && (pre_degree - degree > 180)) {
                zero_pos = i;
                break;
            }
            pre_degree = degree;
        }


        node_info *tmpbuffer = new node_info[count];
        for (i = (int)zero_pos; i < (int)count; i++) {
            tmpbuffer[i-zero_pos] = nodebuffer[i];
        }
        for (i = 0; i < (int)zero_pos; i++) {
            tmpbuffer[i+(int)count-zero_pos] = nodebuffer[i];
        }

        memcpy(nodebuffer, tmpbuffer, count*sizeof(node_info));
        delete[] tmpbuffer;
		return RESULT_OK;
	}

	/************************************************************************/
	/* get health state of lidar                                            */
	/************************************************************************/
	result_t YDlidarDriver::getHealth(device_health & health, uint32_t timeout) {
		result_t ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_HEALTH)) != RESULT_OK) {
				return ans;
			}
			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVHEALTH) {
				return RESULT_FAIL;
			}

			if (response_header.size < sizeof(device_health)) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}

			getData(reinterpret_cast<uint8_t *>(&health), sizeof(health));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* get device info of lidar                                             */
	/************************************************************************/
	result_t YDlidarDriver::getDeviceInfo(device_info & info, uint32_t timeout) {
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
            if ((ans = sendCommand(LIDAR_CMD_GET_DEVICE_INFO)) != RESULT_OK) {
				return ans;
			}
			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size < sizeof(lidar_ans_header)) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&info), sizeof(info));
			model = info.model;
            firmware_version = info.firmware_version;
		}

		return RESULT_OK;
	}

	/************************************************************************/
	/* the set to signal quality                                            */
	/************************************************************************/
    void YDlidarDriver::setIntensities(const bool& isintensities){
		m_intensities = isintensities;
		if(m_intensities){
			PackageSampleBytes = 3;
		}else{
			PackageSampleBytes = 2;
		}
	}

	/************************************************************************/	
	/* Get heartbeat function status                                        */
	/************************************************************************/
    bool YDlidarDriver::getHeartBeat() const
	{
		return isHeartbeat;

	}

	/************************************************************************/
	/* set heartbeat function status                                        */
	/************************************************************************/
    void YDlidarDriver::setHeartBeat(const bool& enable)
	{
		isHeartbeat = enable;

	}
        
	/************************************************************************/
	/* send heartbeat function package                                      */
	/************************************************************************/
	result_t YDlidarDriver::sendHeartBeat(){
		if (!isConnected) {
            return RESULT_FAIL;
        }

        result_t ans = sendCommand(LIDAR_CMD_SCAN);
		return ans;
	}

    /**
        * @brief 设置雷达异常自动重新连接 \n
        * @param[in] enable    是否开启自动重连:
        *     true	开启
        *	  false 关闭
        */
    void YDlidarDriver::setAutoReconnect(const bool& enable) {
            isAutoReconnect = enable;
    }

    void YDlidarDriver::checkTimer() {
        //calc stamp
        m_pointTime = 1e9/4000;
        trans_delay = 0;

        switch(model){
            case YDLIDAR_F4://f4
            trans_delay = _serial->getByteTime();
            break;
            case YDLIDAR_G4://g4
            {
                if(m_sampling_rate == -1){
                    sampling_rate _rate;
                    getSamplingRate(_rate);
                    m_sampling_rate = _rate.rate;
                }
                switch(m_sampling_rate){
                    case 1:
                    m_pointTime = 1e9/8000;
                    break;
                    case 2:
                    m_pointTime = 1e9/9000;
                    break;
                }
                if(firmware_version < 521&& firmware_version != 0){
                    setHeartBeat(false);
                }

            }
            trans_delay = _serial->getByteTime();
            break;
            case YDLIDAR_X4://x4
            m_pointTime = 1e9/5000;
            break;
            case YDLIDAR_F4PRO://f4pro
            {
                if(m_sampling_rate == -1){
                    sampling_rate _rate;
                    getSamplingRate(_rate);
                    m_sampling_rate = _rate.rate;
                }
                if(m_sampling_rate ==1){
                    m_pointTime = 1e9/6000;
                }
                if(firmware_version < 521&& firmware_version != 0){
                    setHeartBeat(false);
                }

            }
            trans_delay = _serial->getByteTime();
            break;
            case YDLIDAR_G4C://g4c
            trans_delay = _serial->getByteTime();
            if(firmware_version < 521&& firmware_version != 0){
                setHeartBeat(false);
            }
            break;
        }
    }

	/************************************************************************/
	/*  start to scan                                                       */
	/************************************************************************/
	result_t YDlidarDriver::startScan(bool force, uint32_t timeout ) {
		result_t ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		if (isScanning) {
			return RESULT_OK;
		}

		stop();   
		startMotor();
        checkTimer();

		{
            ScopedLocker l(_lock);
            if ((ans = sendCommand(force?LIDAR_CMD_FORCE_SCAN:LIDAR_CMD_SCAN)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
				return RESULT_FAIL;
			}

            if (response_header.size < 5 ) {
				return RESULT_FAIL;
			}

			ans = this->createThread();
			return ans;
		}
		return RESULT_OK;
	}

	result_t YDlidarDriver::createThread() {
        _thread = CLASS_THREAD(YDlidarDriver, cacheScanData);
        if (_thread.getHandle() == 0) {
            isScanning = false;
            return RESULT_FAIL;
        }
		isScanning = true;
		return RESULT_OK;
	}

    /************************************************************************/
    /*   startAutoScan                                                      */
    /************************************************************************/
    result_t YDlidarDriver::startAutoScan(bool force, uint32_t timeout) {
        result_t ans;
        if (!isConnected) {
            return RESULT_FAIL;
        }
        {
            ScopedLocker l(_lock);
            if ((ans = sendCommand(force?LIDAR_CMD_FORCE_SCAN:LIDAR_CMD_SCAN)) != RESULT_OK) {
                return ans;
            }

            lidar_ans_header response_header;
            if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
                return ans;
            }

            if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
                return RESULT_FAIL;
            }

            if (response_header.size < 5) {
                return RESULT_FAIL;
            }

        }
        startMotor();
        return RESULT_OK;
    }


	/************************************************************************/
	/*   stop scan                                                   */
	/************************************************************************/
	result_t YDlidarDriver::stop() {
        if(isAutoconnting) {
            isAutoReconnect = false;
            isScanning = false;
            disableDataGrabbing();
            return RESULT_OK;
        }
		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
            sendCommand(LIDAR_CMD_FORCE_STOP);
			sendCommand(LIDAR_CMD_STOP);
		}

		stopMotor();

		return RESULT_OK;
	}

	/************************************************************************/
	/*  reset device                                                        */
	/************************************************************************/
	result_t YDlidarDriver::reset(uint32_t timeout) {
        UNUSED(timeout);
		result_t ans;
		if (!isConnected){
			return RESULT_FAIL;
		}
        ScopedLocker l(_lock);
		if ((ans = sendCommand(LIDAR_CMD_RESET))!= RESULT_OK) {
			return ans;
		}
		return RESULT_OK;
	}


    /** Returns true if the device is in good health, If it's not*/
    bool YDlidarDriver::checkDeviceHealth() {
        result_t op_result;
        device_health healthinfo;
        bool ret = false;

        op_result = getHealth(healthinfo);
        if (op_result == RESULT_OK) {
            if (healthinfo.status != 2) {
                ret = true;
            }

        }
        return ret;
    }

    /** Returns true if the device information is correct, If it's not*/
    bool YDlidarDriver::checkDeviceInfo() {
        device_info devinfo;

        if (getDeviceInfo(devinfo) != RESULT_OK ) {
            return false;
        }
        std::string model;
        sampling_rate _rate;
        int _samp_rate=4;
        result_t ans;
        int bad = 0;

        switch (devinfo.model) {
            case YDLIDAR_F4:
                model="F4";
                break;
            case YDLIDAR_T1:
                model="T1";
                break;
            case YDLIDAR_F2:
                model="F2";
                break;
            case YDLIDAR_S4:
                model="S4";
                break;
            case YDLIDAR_G4:
            {
                model="G4";
                ans = getSamplingRate(_rate);
                if (ans == RESULT_OK) {
                    switch (cfg_.sampleRate) {
                    case 4:
                        _samp_rate=0;
                        break;
                    case 8:
                        _samp_rate=1;
                        break;
                    case 9:
                        _samp_rate=2;
                        break;
                    default:
                        _samp_rate = _rate.rate;
                        break;
                    }

                    while (_samp_rate != _rate.rate) {
                        ans = setSamplingRate(_rate);
                        if (ans != RESULT_OK) {
                            bad++;
                            if(bad>5){
                                break;
                            }
                        }
                    }

                    switch (_rate.rate) {
                        case 0:
                            _samp_rate = 4;
                            break;
                        case 1:
                            node_counts = 1440;
                            _samp_rate=8;
                            break;
                        case 2:
                            node_counts = 1440;
                            _samp_rate=9;
                            break;
                    }


                }
                //cfg_.reversion = true;

            }
                break;
            case YDLIDAR_X4:
                model= "X4";
            break;
            case YDLIDAR_F4PRO:
            {
                model="F4Pro";
                ans = getSamplingRate(_rate);
                if (ans == RESULT_OK) {
                    switch (cfg_.sampleRate) {
                    case 4:
                        _samp_rate=0;
                        break;
                    case 6:
                        _samp_rate=1;
                        break;
                    default:
                        _samp_rate = _rate.rate;
                        break;
                    }
                    while (_samp_rate != _rate.rate) {
                        ans = setSamplingRate(_rate);
                        if (ans != RESULT_OK) {
                            bad++;
                            if(bad>5){
                                break;
                            }
                        }
                    }

                    switch (_rate.rate) {
                        case 0:
                            _samp_rate = 4;
                            break;
                        case 1:
                            node_counts = 1440;
                            _samp_rate=6;
                            break;
                    }

                }

            }
                break;
            case YDLIDAR_G4C:
                model = "G4C";
                //cfg_.reversion = true;
                break;
            default:
                model = "Unknown";
                break;
        }

        cfg_.sampleRate = _samp_rate;



        unsigned int maxv = (unsigned int)(devinfo.firmware_version>>8);
        unsigned int midv = (unsigned int)(devinfo.firmware_version&0xff)/10;
        unsigned int minv = (unsigned int)(devinfo.firmware_version&0xff)%10;
        fprintf(stderr, "firmware: %i\n", devinfo.firmware_version);

        printf("[YDLIDAR] Connection established in [%s]:\n"
                   "Firmware version: %u.%u.%u\n"
                   "Hardware version: %u\n"
                   "Model: %s\n"
                   "Serial: ",
                    cfg_.serialPort.c_str(),
                    maxv,
                    midv,
                    minv,
                    (unsigned int)devinfo.hardware_version,
                    model.c_str());

            for (int i=0;i<16;i++)
                printf("%01X",devinfo.serialnum[i]&0xff);
            printf("\n");

            printf("[YDLIDAR INFO] Current Sampling Rate : %dK\n" , _samp_rate);

            cfg_.intensity = false;
            if(devinfo.model ==4) {
                if(cfg_.serialBaudrate == 153600) {
                    cfg_.intensity = true;
                }
            }



            float freq = 7.0f;
            if (devinfo.model == 5 || devinfo.model ==8 || devinfo.model == 9) {
                checkScanFrequency();
                checkHeartBeat();
            } else {
                printf("[YDLIDAR INFO] Current Scan Frequency : %fHz\n" , freq);
            }

            return true;
    }

    /** Retruns true if the heartbeat function is set to heart is successful, If it's not*/
    bool YDlidarDriver::checkHeartBeat(){
        bool ret = false;
        int count = 0;
        scan_heart_beat beat;
        if( cfg_.heartBeat ) {
            while(cfg_.heartBeat) {
                count++;
                if(count > 8) {
                    throw DeviceException(" check heartbeat failed, please check lidar model");
                }
                result_t ans =setScanHeartbeat(beat);
                if( ans == RESULT_OK) {
                    if( beat.enable ) {
                        ans = setScanHeartbeat(beat);
                        if( ans == RESULT_OK) {
                            if(!beat.enable) {
                                setHeartBeat(true);
                                ret = true;
                                return ret;
                            }
                        }
                    } else  {
                        setHeartBeat(true);
                        ret = true;
                        return ret;
                    }
                }
            }
        }else {
            setHeartBeat(false);
            ret = true;
        }

        return ret;
    }

    /** Retruns true if the scan frequency is set to user's frequency is successful, If it's not*/
    bool YDlidarDriver::checkScanFrequency() {
        float freq = 7.0f;
        scan_frequency _scan_frequency;
        int hz = 0;
        if (5 <= cfg_.scanFrequency && cfg_.scanFrequency <= 12) {
            result_t ans = getScanFrequency(_scan_frequency) ;
            if (ans == RESULT_OK) {
                freq = _scan_frequency.frequency/100.f;
                hz = cfg_.scanFrequency - freq;
                if (hz>0) {
                    while (hz != 0) {
                        setScanFrequencyAdd(_scan_frequency);
                        hz--;
                    }
                    freq = _scan_frequency.frequency/100.0f;
                } else {
                    while (hz != 0) {
                        setScanFrequencyDis(_scan_frequency);
                        hz++;
                    }
                    freq = _scan_frequency.frequency/100.0f;
                }
            }
            if(fabs(cfg_.scanFrequency - freq) < 1.0) {
                hz = (cfg_.scanFrequency - freq)*10;
                if (hz>0) {
                    while (hz != 0) {
                        setScanFrequencyAddMic(_scan_frequency);
                        hz--;
                    }
                    freq = _scan_frequency.frequency/100.0f;
                } else {
                    while (hz != 0) {
                        setScanFrequencyDisMic(_scan_frequency);
                        hz++;
                    }
                    freq = _scan_frequency.frequency/100.0f;
                }

            }

            if (cfg_.scanFrequency < 7 && cfg_.sampleRate>6) {
                node_counts = 1600;

            } else if ( cfg_.scanFrequency < 6 && cfg_.sampleRate == 9) {
                node_counts = 2000;

            } else if ( cfg_.scanFrequency < 6 && cfg_.sampleRate == 4) {
                node_counts = 900;
            }
        }
        printf("[YDLIDAR INFO] Current Scan Frequency : %fHz\n" , freq);

        return true;
    }

    bool YDlidarDriver::checkComms() {

        if (isconnected()) {
            return true;
        }

        // Is it COMX, X>4? ->  "\\.\COMX"
        if (cfg_.serialPort.size()>=3) {
            if ( tolower( cfg_.serialPort[0]) =='c' && tolower( cfg_.serialPort[1]) =='o' && tolower( cfg_.serialPort[2]) =='m' ) {
                // Need to add "\\.\"?
                if (cfg_.serialPort.size()>4 || cfg_.serialPort[3]>'4')
                    cfg_.serialPort = std::string("\\\\.\\") + cfg_.serialPort;
            }
        }

        // make connection...
        result_t op_result = connect(cfg_.serialPort.c_str(), cfg_.serialBaudrate);
        if (op_result != RESULT_OK) {
            fprintf(stderr, "[YDLIDAR INFO] Error, cannot bind to the specified serial port %s\n",  cfg_.serialPort.c_str() );
            return false;
        }

        return true;
    }


	/************************************************************************/
	/* get the current scan frequency of lidar                              */
	/************************************************************************/
	result_t YDlidarDriver::getScanFrequency(scan_frequency & frequency, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_GET_AIMSPEED)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 4) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}

			getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* add the scan frequency by 1Hz each time                              */
	/************************************************************************/
	result_t YDlidarDriver::setScanFrequencyAdd(scan_frequency & frequency, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_ADD)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 4) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}

			getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* decrease the scan frequency by 1Hz each time                         */
	/************************************************************************/
	result_t YDlidarDriver::setScanFrequencyDis(scan_frequency & frequency, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_DIS)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 4) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}

			getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* add the scan frequency by 0.1Hz each time                            */
	/************************************************************************/
	result_t YDlidarDriver::setScanFrequencyAddMic(scan_frequency & frequency, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_ADDMIC)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 4) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}

			getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* decrease the scan frequency by 0.1Hz each time                       */
	/************************************************************************/
	result_t YDlidarDriver::setScanFrequencyDisMic(scan_frequency & frequency, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SET_AIMSPEED_DISMIC)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 4) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}

			getData(reinterpret_cast<uint8_t *>(&frequency), sizeof(frequency));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/*  get the sampling rate of lidar                                      */
	/************************************************************************/
	result_t YDlidarDriver::getSamplingRate(sampling_rate & rate, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}

		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_GET_SAMPLING_RATE)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}

			getData(reinterpret_cast<uint8_t *>(&rate), sizeof(rate));
            m_sampling_rate=rate.rate;
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/*  the set to sampling rate                                            */
	/************************************************************************/
	result_t YDlidarDriver::setSamplingRate(sampling_rate & rate, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SET_SAMPLING_RATE)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&rate), sizeof(rate));
            m_sampling_rate=rate.rate;
		}
		return RESULT_OK;
	}

	std::string YDlidarDriver::getSDKVersion(){
		return SDKVerision;
	}


	result_t YDlidarDriver::setRotationPositive(scan_rotation & roation, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_RUN_POSITIVE)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&roation), sizeof(roation));
		}
		return RESULT_OK;
	}

	result_t YDlidarDriver::setRotationInversion(scan_rotation & roation, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_RUN_INVERSION)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&roation), sizeof(roation));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* enable lower power                                                   */
	/************************************************************************/
	result_t YDlidarDriver::enableLowerPower(function_state & state, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_ENABLE_LOW_POWER)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&state), sizeof(state));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/*  disable lower power                                                 */
	/************************************************************************/
	result_t YDlidarDriver::disableLowerPower(function_state & state, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_DISABLE_LOW_POWER)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&state), sizeof(state));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* get the state of motor                                               */
	/************************************************************************/
	result_t YDlidarDriver::getMotorState(function_state & state, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_STATE_MODEL_MOTOR)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&state), sizeof(state));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* enable constant frequency                                            */
	/************************************************************************/
	result_t YDlidarDriver::enableConstFreq(function_state & state, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_ENABLE_CONST_FREQ)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&state), sizeof(state));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/* disable constant frequency                                           */
	/************************************************************************/
	result_t YDlidarDriver::disableConstFreq(function_state & state, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_DISABLE_CONST_FREQ)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&state), sizeof(state));
		}
		return RESULT_OK;
	}

	/************************************************************************/
	/*  the save current low exposure value for S4 which has signal quality                */
	/************************************************************************/
	result_t YDlidarDriver::setSaveLowExposure(scan_exposure& low_exposure, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SAVE_SET_EXPOSURE)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&low_exposure), sizeof(low_exposure));
		}
		return RESULT_OK;


	}

	/************************************************************************/
	/*  the set to low exposure for S4 which has signal quality                */
	/************************************************************************/
	result_t YDlidarDriver::setLowExposure(scan_exposure& low_exposure, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SET_LOW_EXPOSURE)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&low_exposure), sizeof(low_exposure));
		}
		return RESULT_OK;


	}

	/************************************************************************/
	/*  add scan exposure for S4 which has signal quality                      */
	/************************************************************************/
	result_t YDlidarDriver::setLowExposureAdd(scan_exposure & exposure, uint32_t timeout){
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_ADD_EXPOSURE)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&exposure), sizeof(exposure));
		}
		return RESULT_OK;

	}

	/************************************************************************/
	/*  decrease scan exposure for S4 which has signal quality                 */
	/************************************************************************/
	result_t YDlidarDriver::setLowExposurerDis(scan_exposure & exposure, uint32_t timeout)
	{
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_DIS_EXPOSURE)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&exposure), sizeof(exposure));
		}
		return RESULT_OK;


	}

    /************************************************************************/
    /*  set heartbeat function for G4 F4Pro                  */
    /************************************************************************/
    result_t YDlidarDriver::setScanHeartbeat(scan_heart_beat& beat,uint32_t timeout)
    {
        result_t  ans;
        if (!isConnected) {
            return RESULT_FAIL;
        }
        disableDataGrabbing();
        {
            ScopedLocker l(_lock);
            if ((ans = sendCommand(LIDAR_CMD_SET_HEART_BEAT)) != RESULT_OK) {
                return ans;

            }
            lidar_ans_header response_header;
            if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
                return ans;
            }

            if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
                return RESULT_FAIL;
            }

            if (response_header.size != 1) {
                return RESULT_FAIL;
            }

            if (waitForData(response_header.size, timeout) != RESULT_OK) {
                return RESULT_FAIL;
            }
            getData(reinterpret_cast<uint8_t *>(&beat), sizeof(beat));



        }

        return RESULT_OK;

    }

	/************************************************************************/
	/*  set a circle of data fixed points for S4                  */
	/************************************************************************/
       result_t YDlidarDriver::setPointsForOneRingFlag(scan_points& points,uint32_t timeout)
       {
		result_t  ans;
		if (!isConnected) {
			return RESULT_FAIL;
		}
		disableDataGrabbing();
		{
            ScopedLocker l(_lock);
			if ((ans = sendCommand(LIDAR_CMD_SET_SETPOINTSFORONERINGFLAG)) != RESULT_OK) {
				return ans;
			}

			lidar_ans_header response_header;
			if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
				return ans;
			}

			if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
				return RESULT_FAIL;
			}

			if (response_header.size != 1) {
				return RESULT_FAIL;
			}

			if (waitForData(response_header.size, timeout) != RESULT_OK) {
				return RESULT_FAIL;
			}
			getData(reinterpret_cast<uint8_t *>(&points), sizeof(points));
		}
		return RESULT_OK;

       }

}
