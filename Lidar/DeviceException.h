/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     DeviceException.h                                               *
*  @brief    Device exception description                                    *
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

#include <exception>
#include <string>

namespace ydlidar {

struct DeviceException : std::exception
{
    DeviceException(std::string str) : desc(str) {}
    DeviceException(std::string str, std::string detail) {
        desc = str + "\n\t" + detail;
    }
    ~DeviceException() throw() {}
    const char* what() const throw() { return desc.c_str(); }
    std::string desc;
};

struct TimeoutException : std::exception
{
    TimeoutException(std::string str) : desc(str) {}
    TimeoutException(std::string str, std::string detail) {
        desc = str + "\n\t" + detail;
    }
    ~TimeoutException() throw() {}
    const char* what() const throw() { return desc.c_str(); }
    std::string desc;
};


struct CorruptedDataException : std::exception
{
    CorruptedDataException(std::string str) : desc(str) {}
    CorruptedDataException(std::string str, std::string detail) {
        desc = str + "\n\t" + detail;
    }
    ~CorruptedDataException() throw() {}
    const char* what() const throw() { return desc.c_str(); }
    std::string desc;
};

struct DeviceInformationException : std::exception
{
    DeviceInformationException(std::string str) : desc(str) {}
    DeviceInformationException(std::string str, std::string detail) {
        desc = str + "\n\t" + detail;
    }
    ~DeviceInformationException() throw() {}
    const char* what() const throw() { return desc.c_str(); }
    std::string desc;
};
}
