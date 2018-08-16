/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     LIDARDevice.h                                                   *
*  @brief    LIDAR Device Interface                                          *
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
#pragma once

#include <memory>
#include "LIDARDriverInterface.h"
#include "Drivers/YDLidar/ydlidar_driver.h"
#include "Drivers/YDLidar/timer.h"
#include "DeviceException.h"

namespace ydlidar {

///////////////////////////////////////////////////////////////////////////////
// Generic LIDAR device
class LIDAR : public LIDARDriverInterface
{
    public:
        ///////////////////////////////////////////////////////////////
        LIDAR() {
            YDlidarDriver* pDriver = new YDlidarDriver;
            m_LIDAR  = std::shared_ptr<LIDARDriverInterface>( pDriver );
        }

        ///////////////////////////////////////////////////////////////
        ~LIDAR() {
            Clear();
        }

        ///////////////////////////////////////////////////////////////
        void Clear() {
            m_LIDAR = nullptr;
        }

        //////////////////////////////////////////////////////////////
        /**
         * @brief get current lidar lists
         * @return current online lidar list
         */
        std::vector<std::string> getLidarList() {
            if(m_LIDAR) {
                return m_LIDAR->getLidarList();
            } else {
                throw DeviceException("error: no driver initialized!");
            }
        }
        //////////////////////////////////////////////////////////////
        /// \brief Update lidar configuration parameters
        /// \param configuration parameters
        ///
        void UpdateLidarParamCfg(const LaserParamCfg& config_msg) {
            if(m_LIDAR) {
                m_LIDAR->UpdateLidarParamCfg(config_msg);
            } else {
                throw DeviceException("error: no driver initialized!");
            }
        }


        ///////////////////////////////////////////////////////////////
        /// \brief Registered lidar data callback
        /// \param callback function
        ///
        void RegisterLIDARDataCallback(LIDARDriverDataCallback callback) {
            if( m_LIDAR ) {
                m_LIDAR->RegisterLIDARDataCallback( callback );
            } else {
                throw DeviceException("error: no driver initialized!");
            }
        }
        ///////////////////////////////////////////////////////////////
        /// \brief continuous access to lidar data
        ///
        void spinOnce() {
            if( m_LIDAR ) {
                m_LIDAR->spinOnce();
            } else {
                throw DeviceException("error: no driver initialized!");
            }
        }


protected:
    std::shared_ptr<LIDARDriverInterface>     m_LIDAR;

};

} /* namespace */
