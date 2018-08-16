/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     LIDARDriverInterface.h                                          *
*  @brief    LIDAR  Interface                                                *
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
*  2018/08/09 | 1.3.7    | Tony.Yang      | Create file                     *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/
#pragma once

#include <functional>
#include <vector>
#include "Utils.h"


namespace ydlidar {

typedef std::function<void (ydlidar::LaserScan&)> LIDARDriverDataCallback;

///////////////////////////////////////////////////////////////////////////////
/// Generic LIDAR driver interface
class LIDARDriverInterface
{
    public:
        // Pure virtual functions driver writers must implement:
        virtual ~LIDARDriverInterface() {}


        /**
         * @brief get current lidar list
         * @return current online lidar list
         */
        virtual std::vector<std::string> getLidarList() = 0;
        /**
         * @brief Update lidar configuration parameters
         * @param configuration parameters
         */
        virtual void UpdateLidarParamCfg(const LaserParamCfg& config_msg) = 0;

        /**
         * @brief Registered lidar data callback
         * @param data callback function
         */
        virtual void RegisterLIDARDataCallback(LIDARDriverDataCallback callback) = 0;

        /**
         * @brief continuous access to lidar data
         */
        virtual void spinOnce() = 0;

};

} /* namespace */
