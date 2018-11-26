/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     test.cpp                                                        *
*  @brief    Lidar test                                                      *
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

#ifdef _WIN32
# pragma warning(disable: 4786)
#pragma  comment(lib, "ydlidar_driver.lib")
#endif
#include <iostream>
#include <string>
#include <memory>
#include <Lidar/LIDARDevice.h>
#include <config.h>
#include <simpleini/SimpleIni.h>

using namespace std;
using namespace ydlidar;

std::vector<float> split(const std::string &s, char delim) {
    std::vector<float> elems;
    std::stringstream ss(s);
    std::string number;
    while(std::getline(ss, number, delim)) {
        elems.push_back(atof(number.c_str()));
    }
    return elems;
}


void LaserScanCallback(const LaserScan& scan) {

    std::cout<< "received scan size: "<< scan.ranges.size()<<std::endl;
    std::cout<< "scan   system time: "<< scan.system_time_stamp<<std::endl;
    std::cout<< "scan     self time: "<< scan.self_time_stamp<<std::endl;
    std::cout<< "scan     frequency: "<< 1000000000.0/scan.config.scan_time << "HZ"<<std::endl;
    std::cout<< "lidar    frequency: "<< scan.scan_frequency << "HZ"<<std::endl;


    for(size_t i =0; i < scan.ranges.size(); i++) {
        // current angle
        double angle = scan.config.min_angle + i*scan.config.ang_increment;

        //current distance
        double distance = scan.ranges[i];

        //current intensity
        int intensity = scan.intensities[i];

        UNUSED(angle);
        UNUSED(distance);
        UNUSED(intensity);

    }

}

int main(int argc, char * argv[])
{
	printf(" YDLIDAR C++ TEST\n");

    ydlidar::init(argc, argv);
    CSimpleIniA ini;
    ini.SetUnicode();
    LaserParamCfg cfg;
    bool input = true;
    std::string ini_file = "lidar.ini";
    bool ini_exist = fileExists(ini_file.c_str());
    if(ini_exist ||  argc > 1) {
        if(argc > 1)
            ini_file = (std::string)argv[1];
        if(fileExists(ini_file.c_str())) {
            SI_Error rc = ini.LoadFile(ini_file.c_str());
            if(rc >= 0 ) {
                input = false;
                const char * pszValue = ini.GetValue("LIDAR", "serialPort", "");
                cfg.serialPort = pszValue;
                if(cfg.serialPort.empty()) {
                    input = true;
                }

                pszValue = ini.GetValue("LIDAR", "ignoreArray", "");

                cfg.ignoreArray = split(pszValue,',');

                cfg.serialBaudrate = ini.GetLongValue("LIDAR", "serialBaudrate", cfg.serialBaudrate);
                cfg.sampleRate = ini.GetLongValue("LIDAR", "sampleRate", cfg.sampleRate);
                cfg.scanFrequency = ini.GetLongValue("LIDAR", "scanFrequency", cfg.scanFrequency);

                cfg.intensity = ini.GetBoolValue("LIDAR", "intensity", cfg.intensity);
                cfg.autoReconnect = ini.GetBoolValue("LIDAR", "autoReconnect", cfg.autoReconnect);
                cfg.exposure = ini.GetBoolValue("LIDAR", "exposure", cfg.exposure);
                cfg.fixedResolution = ini.GetBoolValue("LIDAR", "fixedResolution", cfg.fixedResolution);
                cfg.reversion = ini.GetBoolValue("LIDAR", "reversion", cfg.reversion);
                cfg.heartBeat = ini.GetBoolValue("LIDAR", "heartBeat", cfg.heartBeat);

                cfg.maxAngle = ini.GetDoubleValue("LIDAR", "maxAngle", cfg.maxAngle);
                cfg.minAngle = ini.GetDoubleValue("LIDAR", "minAngle", cfg.minAngle);
                cfg.maxRange = ini.GetDoubleValue("LIDAR", "maxRange", cfg.maxRange);
                cfg.minRange = ini.GetDoubleValue("LIDAR", "minRange", cfg.minRange);
            }
        }

    }



    bool excep = false;
    bool input_port =false;
    try {
        LIDAR ydlidar;
        std::vector<string> ports =  ydlidar.getLidarList();
        if(ports.size() == 1) {
            if(cfg.serialPort != ports[0]) {
                std::string str;
                printf("Radar[%s] detected, whether to select current radar(yes/no)?:", ports[0].c_str());
                std::cin>>str;
                for (size_t i=0; i <str.size(); i++)
                   str[i] = tolower(str[i]);
                if(str.find("yes") != std::string::npos ||atoi(str.c_str()) == 1 ) {
                    cfg.serialPort = ports[0];
                    input_port = true;
                }else{
                    return 0;
                }
            }
        }


        if(input) {
            std::string port;
            std::string baudrate;
            std::string intensity;
            if(ports.empty()) {
                printf("Not radar delected, Please enter the radar port manually: ");
                std::cin>>port;
                cfg.serialPort = port;
                input_port = true;
            }


            if(!input_port) {

                int size = 0;
                for(std::vector<string>::iterator it = ports.begin(); it != ports.end(); it++) {
                    printf("%d. %s\n", size, (*it).c_str());
                    size++;
                }


                select_port:
                printf("Please select the lidar port:");
                std::cin>>port;
                if((size_t)atoi(port.c_str()) >= ports.size()) {
                    printf("Invalid serial number, Please re-select\n");
                    goto select_port;
                }
                cfg.serialPort = ports[atoi(port.c_str())];

            }



            std::vector<unsigned int> baud;
            baud.push_back(115200);
            baud.push_back(128000);
            baud.push_back(153600);
            baud.push_back(230400);

            for( unsigned int i = 0; i < baud.size(); i ++) {
                printf("%u. %u\n", i, baud[i]);
            }

            select_baud:
            printf("Please select the lidar baud rate:");
            std::cin>>baudrate;

            if(atoi(baudrate.c_str()) >= 4) {
                printf("Invalid serial number, Please re-select\n");
                goto select_baud;

            }
            cfg.serialBaudrate = baud[atoi(baudrate.c_str())];


            printf("0. false\n");
            printf("1. true\n");
            select_intensity:
            printf("Please select the lidar intensity:");
            std::cin>>intensity;

            if(atoi(intensity.c_str()) >= 2) {
                printf("Invalid serial number, Please re-select\n");
                goto select_intensity;

            }
            cfg.intensity = atoi(intensity.c_str()) ==0?false:true;
        }

        ini.SetValue("LIDAR", "serialPort", cfg.serialPort.c_str());

        ini.SetLongValue("LIDAR", "serialBaudrate", cfg.serialBaudrate);
        ini.SetLongValue("LIDAR", "sampleRate", cfg.sampleRate);
        ini.SetLongValue("LIDAR", "scanFrequency", cfg.scanFrequency);


        ini.SetBoolValue("LIDAR", "intensity", cfg.intensity);
        ini.SetBoolValue("LIDAR", "autoReconnect", cfg.autoReconnect);
        ini.SetBoolValue("LIDAR", "exposure", cfg.exposure);
        ini.SetBoolValue("LIDAR", "fixedResolution", cfg.fixedResolution);
        ini.SetBoolValue("LIDAR", "reversion", cfg.reversion);
        ini.SetBoolValue("LIDAR", "heartBeat", cfg.heartBeat);

        ini.SetDoubleValue("LIDAR", "maxAngle", cfg.maxAngle);
        ini.SetDoubleValue("LIDAR", "minAngle", cfg.minAngle);
        ini.SetDoubleValue("LIDAR", "maxRange", cfg.maxRange);
        ini.SetDoubleValue("LIDAR", "minRange", cfg.minRange);

        std::cout<<"SDK Version: "<< SDK_VERSION<<std::endl;
        std::cout <<"LIDAR Version: "<< YDLIDAR_VERSION <<std::endl;


        ydlidar.RegisterLIDARDataCallback(&LaserScanCallback);
        ydlidar.UpdateLidarParamCfg(cfg);

         while(ydlidar::ok()){
             try {
                 ydlidar.spinOnce();
             }catch(TimeoutException& e) {
                 std::cout<< e.what()<<std::endl;

             }catch(CorruptedDataException& e) {
                 std::cout<< e.what()<<std::endl;

             }catch(DeviceInformationException& e) {
                 std::cout<< e.what()<<std::endl;
             }catch(DeviceException& e) {
                 std::cerr<< e.what()<<std::endl;
                 excep = true;
                 break;
             }
         }

    }catch(TimeoutException& e) {
        std::cout<< e.what()<<std::endl;
        excep = true;
    }catch(CorruptedDataException& e) {
        std::cout<< e.what()<<std::endl;
        excep = true;
    }catch(DeviceInformationException& e) {
        std::cout<< e.what()<<std::endl;
        excep = true;
    }catch(DeviceException& e) {
        std::cerr<< e.what()<<std::endl;
        excep = true;
    }

    if(!excep) {
        ini.SaveFile(ini_file.c_str());
    }
    return 0;


}
