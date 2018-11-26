/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     ydlidar_driver.h                                                *
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

#ifndef YDLIDAR_DRIVER_H
#define YDLIDAR_DRIVER_H
#include <stdlib.h>
#include <atomic>
#include <list>
#include <map>
#include <algorithm>
#include <condition_variable>
#include <Lidar/LIDARDriverInterface.h>
#include <Lidar/DeviceException.h>
#include <Serial/serial.h>
#include "locker.h"
#include "thread.h"



using namespace std;
using namespace serial;


namespace ydlidar{

    YDLIDAR_API class YDlidarDriver : public LIDARDriverInterface
	{
	public:

        /**
        * A constructor.
        * A more elaborate description of the constructor.
        */
         YDlidarDriver();

        /**
        * A destructor.
        * A more elaborate description of the destructor.
        */
         virtual ~YDlidarDriver();

         /**
          * @brief getLidarLists
          * @return
          */
         std::vector<std::string> getLidarList();


         /**
          * @brief UpdateLidarParamCfg
          * @param config_msg
          */
         void UpdateLidarParamCfg(const LaserParamCfg& config_msg);


         /**
          * @brief RegisterLIDARDataCallback
          * @param callback
          */
         void RegisterLIDARDataCallback(LIDARDriverDataCallback callback);


         /**
          * @brief spinOnce
          */
         void spinOnce();



         /**
          * @brief lidarPortList
          * @return
          */
         static std::vector<std::string> lidarPortList();


    private:


		/**
        * @brief Connecting Lidar \n
        * After the connection is successful，the ::disconnect function must be closed
        * @param[in] port_path    serial port
        * @param[in] baudrate    baud rate，YDLIDAR have the following baudrate：
    	*     115200 F4, G4C, S4A
    	*     128000 X4
    	*     153600 S4B
    	*     230600 F4PRO, G4
        * @retval 0     success
        * @retval < 0   failed
        * @noteAfter the connection is successful，the ::disconnect function must be closed
        * @see function YDlidarDriver::disconnect
    	*/
		result_t connect(const char * port_path, uint32_t baudrate);

		/**
        * @brief disconnect the lidar
    	*/
		void disconnect();

		/**
        * @brief Get SDK version \n
        * static function
        * @return current SDK version
    	*/
		static std::string getSDKVersion();

		/**
        * @brief scanning \n
        * @retval true     Scanning
        * @retval false    Stop Scanning
    	*/
        bool isscanning() const;

		/**
        * @brief connection \n
        * @retval true     success
        * @retval false    failed
    	*/
        bool isconnected() const;

		/**
		* @brief 设置雷达是否带信号质量 \n
    	* 连接成功后，必须使用::disconnect函数关闭
    	* @param[in] isintensities    是否带信号质量:
		*     true	带信号质量
		*	  false 无信号质量
        * @note只有S4B(波特率是153600)雷达支持带信号质量, 别的型号雷达暂不支持
    	*/
        void setIntensities(const bool& isintensities);

		/**
		* @brief 获取当前雷达掉电保护功能 \n
		* @return 返回掉电保护是否开启
    	* @retval true     掉电保护开启
    	* @retval false    掉电保护关闭
    	*/
        bool getHeartBeat() const;

		/**
		* @brief 设置雷达掉电保护使能 \n
    	* @param[in] enable    是否开启掉电保护:
		*     true	开启
		*	  false 关闭
    	* @note只有(G4, G4C, F4PRO)雷达支持掉电保护功能, 别的型号雷达暂不支持
        * 并且版本号大于等于2.0.9 才支持此功能, 小于2.0.9版本禁止开启掉电保护
    	*/
        void setHeartBeat(const bool& enable);

        /**
         * @brief 设置雷达异常自动重新连接 \n
         * @param[in] enable    是否开启自动重连:
         *     true	开启
         *	  false 关闭
         */
        void setAutoReconnect(const bool& enable);


		/**
		* @brief 获取雷达设备健康状态 \n
    	* @return 返回执行结果
    	* @retval RESULT_OK       获取成功
    	* @retval RESULT_FAILE or RESULT_TIMEOUT   获取失败
    	*/
		result_t getHealth(device_health & health, uint32_t timeout = DEFAULT_TIMEOUT);

		/**
		* @brief 获取雷达设备信息 \n
		* @param[in] info     设备信息
    	* @param[in] timeout  超时时间  
    	* @return 返回执行结果
    	* @retval RESULT_OK       获取成功
    	* @retval RESULT_FAILE or RESULT_TIMEOUT   获取失败
    	*/
		result_t getDeviceInfo(device_info & info, uint32_t timeout = DEFAULT_TIMEOUT);

		/**
		* @brief 开启扫描 \n
    	* @param[in] force    扫描模式
    	* @param[in] timeout  超时时间  
    	* @return 返回执行结果
    	* @retval RESULT_OK       开启成功
    	* @retval RESULT_FAILE    开启失败
		* @note 只用开启一次成功即可
    	*/
		result_t startScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;

		/**
		* @brief 关闭扫描 \n
    	* @return 返回执行结果
    	* @retval RESULT_OK       关闭成功
    	* @retval RESULT_FAILE    关闭失败
    	*/
		result_t stop();
        /**
        * @brief 获取激光数据 \n
        * @param[in] nodebuffer 激光点信息
        * @param[in] count      一圈激光点数
        * @param[in] timeout    超时时间
        * @return 返回执行结果
        * @retval RESULT_OK       获取成功
        * @retval RESULT_FAILE    获取失败
        * @note 获取之前，必须使用::startScan函数开启扫描
        */
        result_t grabScanData(node_info * nodebuffer, size_t & count, uint32_t timeout = DEFAULT_TIMEOUT) ;



		/**
		* @brief 补偿激光角度 \n
		* 把角度限制在0到360度之间
    	* @param[in] nodebuffer 激光点信息
		* @param[in] count      一圈激光点数
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 补偿之前，必须使用::grabScanData函数获取激光数据成功
    	*/
		result_t ascendScanData(node_info * nodebuffer, size_t count);

		/**	
		* @brief 重置激光雷达 \n
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作, 如果在扫描中调用::stop函数停止扫描
    	*/
		result_t reset(uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 打开电机 \n
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
    	*/
		result_t startMotor();

		/**	
		* @brief 关闭电机 \n
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
    	*/
		result_t stopMotor();


		/**	
		* @brief 获取激光雷达当前扫描频率 \n
		* @param[in] frequency    扫描频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t getScanFrequency(scan_frequency & frequency, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置增加扫描频率1HZ \n
		* @param[in] frequency    扫描频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t setScanFrequencyAdd(scan_frequency & frequency, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置减小扫描频率1HZ \n
		* @param[in] frequency    扫描频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t setScanFrequencyDis(scan_frequency & frequency, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置增加扫描频率0.1HZ \n
		* @param[in] frequency    扫描频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t setScanFrequencyAddMic(scan_frequency & frequency, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置减小扫描频率0.1HZ \n
		* @param[in] frequency    扫描频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t setScanFrequencyDisMic(scan_frequency & frequency, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 获取激光雷达当前采样频率 \n
		* @param[in] frequency    采样频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t getSamplingRate(sampling_rate & rate, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置激光雷达当前采样频率 \n
		* @param[in] frequency    采样频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t setSamplingRate(sampling_rate & rate, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置电机顺时针旋转 \n
		* @param[in] rotation    旋转方向
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t setRotationPositive(scan_rotation & rotation, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置电机逆顺时针旋转 \n
		* @param[in] rotation    旋转方向
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t setRotationInversion(scan_rotation & rotation, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 低功耗使能 \n
		* @param[in] state    低功耗状态
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作,低功耗关闭,关闭后 G4 在空闲模式下电\n
		* 机和测距单元仍然工作
    	*/
		result_t enableLowerPower(function_state & state, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 关闭低功耗 \n
		* @param[in] state    低功耗状态
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作,关闭后 G4 在空闲模式下电\n
		* 机和测距单元仍然工作
    	*/
		result_t disableLowerPower(function_state & state, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 获取电机状态 \n
		* @param[in] state    电机状态
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t getMotorState(function_state & state, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 开启恒频功能 \n
		* @param[in] state    	  恒频状态
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t enableConstFreq(function_state & state, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 关闭恒频功能 \n
		* @param[in] state    	  恒频状态
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作
    	*/
		result_t disableConstFreq(function_state & state, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 保存当前激光曝光值 \n
		* @param[in] low_exposure    低光功能状态
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作, 当前操作需在非低光功率模式下, \n
		* 只有S4雷达支持此功能
    	*/
		result_t setSaveLowExposure(scan_exposure& low_exposure, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置低光功率模式 \n
		* @param[in] low_exposure    扫描频率
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作, 当前操作是开关量,只有S4雷达支持此功能
    	*/
		result_t setLowExposure(scan_exposure& low_exposure, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 增加激光曝光值 \n
		* @param[in] exposure     曝光值
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作,只有S4雷达支持此功能
    	*/
		result_t setLowExposureAdd(scan_exposure & exposure, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 减小激光曝光值 \n
		* @param[in] exposure     曝光值
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作,只有S4雷达支持此功能
    	*/
		result_t setLowExposurerDis(scan_exposure & exposure, uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置雷达掉电保护状态 \n
		* @param[in] beat    	  掉电保护状态
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
        * @note 停止扫描后再执行当前操作, 当前操作是开关量, (G4, G4C, F4PRO)版本号大于等于2.0.9才支持
    	*/
        result_t setScanHeartbeat(scan_heart_beat& beat,uint32_t timeout = DEFAULT_TIMEOUT);

		/**	
		* @brief 设置扫描一圈固定激光点数 \n
		* @param[in] points    	  固定点数状态
		* @param[in] timeout      超时时间
    	* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败
		* @note 停止扫描后再执行当前操作, 当前操作是开关量,只有S4雷达支持此功能
    	*/
		result_t setPointsForOneRingFlag(scan_points& points,uint32_t timeout = DEFAULT_TIMEOUT);


	protected:

		/**
		* @brief 创建解析雷达数据线程 \n
		* @note 创建解析雷达数据线程之前，必须使用::startScan函数开启扫图成功
    	*/
		result_t createThread();


        /**
        * @brief 重新连接开启扫描 \n
        * @param[in] force    扫描模式
        * @param[in] timeout  超时时间
        * @return 返回执行结果
        * @retval RESULT_OK       开启成功
        * @retval RESULT_FAILE    开启失败
        * @note sdk 自动重新连接调用
        */

        result_t startAutoScan(bool force = false, uint32_t timeout = DEFAULT_TIMEOUT) ;

		/**
		* @brief 解包激光数据 \n
    	* @param[in] node 解包后激光点信息
		* @param[in] timeout     超时时间
    	*/
		result_t waitPackage(node_info * node, uint32_t timeout = DEFAULT_TIMEOUT);

		/**
		* @brief 发送数据到雷达 \n
    	* @param[in] nodebuffer 激光信息指针
    	* @param[in] count      激光点数大小	
		* @param[in] timeout      超时时间	
		* @return 返回执行结果
    	* @retval RESULT_OK       成功
		* @retval RESULT_TIMEOUT  等待超时
    	* @retval RESULT_FAILE    失败	
    	*/
		result_t waitScanData(node_info * nodebuffer, size_t & count, uint32_t timeout = DEFAULT_TIMEOUT);

		/**
		* @brief 激光数据解析线程 \n
    	*/
		int cacheScanData();

		/**
		* @brief 发送数据到雷达 \n
    	* @param[in] cmd 	 命名码
    	* @param[in] payload      payload	
		* @param[in] payloadsize      payloadsize	
		* @return 返回执行结果
    	* @retval RESULT_OK       成功
    	* @retval RESULT_FAILE    失败	
    	*/
		result_t sendCommand(uint8_t cmd, const void * payload = NULL, size_t payloadsize = 0);

		/**
		* @brief 等待激光数据包头 \n
    	* @param[in] header 	 包头
    	* @param[in] timeout      超时时间	
		* @return 返回执行结果
    	* @retval RESULT_OK       获取成功
		* @retval RESULT_TIMEOUT  等待超时
    	* @retval RESULT_FAILE    获取失败	
		* @note 当timeout = -1 时, 将一直等待
    	*/
		result_t waitResponseHeader(lidar_ans_header * header, uint32_t timeout = DEFAULT_TIMEOUT);

		/**
		* @brief 等待固定数量串口数据 \n
    	* @param[in] data_count 	 等待数据大小
    	* @param[in] timeout    	 等待时间	
		* @param[in] returned_size   实际数据大小	
		* @return 返回执行结果
    	* @retval RESULT_OK       获取成功
		* @retval RESULT_TIMEOUT  等待超时
    	* @retval RESULT_FAILE    获取失败	
		* @note 当timeout = -1 时, 将一直等待
    	*/
        result_t waitForData(size_t data_count,uint32_t timeout = DEFAULT_TIMEOUT, size_t * returned_size = NULL);

		/**
		* @brief 获取串口数据 \n
    	* @param[in] data 	 数据指针
    	* @param[in] size    数据大小	
		* @return 返回执行结果
    	* @retval RESULT_OK       获取成功
    	* @retval RESULT_FAILE    获取失败	
    	*/
		result_t getData(uint8_t * data, size_t size);

		/**
		* @brief 串口发送数据 \n
    	* @param[in] data 	 发送数据指针
    	* @param[in] size    数据大小	
		* @return 返回执行结果
    	* @retval RESULT_OK       发送成功
    	* @retval RESULT_FAILE    发送失败	
    	*/
		result_t sendData(const uint8_t * data, size_t size);

        /**
        * @brief 发送掉电保护命令 \n
        * @return 返回执行结果
        * @retval RESULT_OK       发送成功
        * @retval RESULT_FAILE    发送失败
        * @note只有(G4, G4C, F4PRO)雷达支持掉电保护功能, 别的型号雷达暂不支持
        */
        result_t sendHeartBeat();


		/**
        * @brief Close the data acquisition channel \n
    	*/
		void disableDataGrabbing();

		/**
        * @brief set DTR \n
    	*/
		void setDTR();

		/**
        * @brief clear DTR \n
    	*/
		void clearDTR();

        /** Returns true if the device is in good health, If it's not*/
        bool checkDeviceHealth() ;

        /** Returns true if the device information is correct, If it's not*/
        bool checkDeviceInfo() ;

        /** Retruns true if the heartbeat function is set to heart is successful, If it's not*/
        bool checkHeartBeat();

        /** Retruns true if the scan frequency is set to user's frequency is successful, If it's not*/
        bool checkScanFrequency() ;
        /**
         * @brief check serial
         * @return true if the lidar is connected , If it's not
         */
        bool checkComms() ;

        void checkTimer();


    private:
        std::atomic_bool     isConnected;  ///<
        std::atomic_bool     isScanning;   ///<
        std::atomic_bool     isHeartbeat;  ///<
        std::atomic_bool     isAutoReconnect;  ///<
        std::atomic_bool     isAutoconnting;  ///<

		enum {
            DEFAULT_TIMEOUT     = 2000,    /**< Default timeout. */
            DEFAULT_HEART_BEAT  = 1000, /**< Default heatbeat send time. */
            MAX_SCAN_NODES      = 2048,	   /**< Maximum number of scan points. */
		};
		enum { 
            YDLIDAR_F4=1, /**< F4 code. */
            YDLIDAR_T1=2, /**< T1 code. */
            YDLIDAR_F2=3, /**< F2 code. */
            YDLIDAR_S4=4, /**< S4 code. */
            YDLIDAR_G4=5, /**< G4 code. */
            YDLIDAR_X4=6, /**< X4 code. */
            YDLIDAR_F4PRO=8, /**< F4PRO code. */
            YDLIDAR_G4C=9, /**< G4C code. */

		};
        node_info               scan_node_buf[2048];  ///< Laser point information
        size_t                  scan_node_count;      ///< Laser point count
        Locker                  _cfg_lock;			  ///< paramters lock
        Locker                  _serial_lock;		 ///< serial lock
        Locker                  _lock;
        Event                   _cond;			      ///<
        Thread                  _thread;				///<




	private:
        int                                 PackageSampleBytes;             ///< laser points
        serial::Serial                      *_serial;			///< serial
        bool                                m_intensities;					///< intensity
        int                                 m_sampling_rate;					///< sample rate
        int                                 model;							///< lidar model
        uint32_t                            m_baudrate;					///< baud rate
        bool                                isSupportMotorCtrl;			///< Motor control
        uint64_t                            m_ns;						///< time stamp
        uint64_t                            m_last_ns;						///< time stamp
        uint32_t                            m_pointTime;				///< laser point time interval
        uint32_t                            trans_delay;				///< Transmission time
        uint16_t                            firmware_version;          ///< Firmware version

        node_package                        package;
        node_packages                       packages;

        uint16_t                            package_Sample_Index;
        float                               IntervalSampleAngle;
        float                               IntervalSampleAngle_LastPackage;
        uint16_t                            FirstSampleAngle;
        uint16_t                            LastSampleAngle;
        uint16_t                            CheckSun;

        uint16_t                            CheckSunCal;
        uint16_t                            SampleNumlAndCTCal;
        uint16_t                            LastSampleAngleCal;
        bool                                CheckSunResult;
        uint16_t                            Valu8Tou16;
        std::string                         serial_port;///< serial port

        LaserParamCfg                       cfg_; ///< configuration parameters
        size_t                              node_counts; ///< laser point count

        static std::map<std::string, std::string> lidar_map;

    private:
        LIDARDriverDataCallback             m_callback;

	};
}

#endif // YDLIDAR_DRIVER_H
