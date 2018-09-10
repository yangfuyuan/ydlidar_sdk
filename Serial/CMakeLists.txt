#/*****************************************************************************
#*  EAI TOF LIDAR DRIVER                                                      *
#*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
#*                                                                            *
#*  This file is part of EAI TOF LIDAR DRIVER.                                *
#*                                                                            *
#*  @file     CMakeLists.txt                                                  *
#*  @brief                                                                    *
#*  Details.                                                                  *
#*                                                                            *
#*  @author   Tony.Yang                                                       *
#*  @email    chushuifurong618@eaibot.com                                     *
#*  @version  1.0.0(版本号)                                                    *
#*  @date     chushuifurong618@eaibot.com                                     *
#*                                                                            *
#*                                                                            *
#*----------------------------------------------------------------------------*
#*  Remark         : Description                                              *
#*----------------------------------------------------------------------------*
#*  Change History :                                                          *
#*  <Date>     | <Version> | <Author>       | <Description>                   *
#*----------------------------------------------------------------------------*
#*  2018/08/09 | 1.3.7     | Tony.Yang      | Create file                     *
#*----------------------------------------------------------------------------*
#*                                                                            *
#*****************************************************************************/
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/impl )

set(HDRS
    LIDARDevice.h
    LIDARDriverInterface.h
    Utils.h
    DeviceException.h
)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
add_to_hal_sources(serial.cpp  )
add_to_hal_headers( common.h serial.h)

IF (WIN32)
add_to_hal_sources(impl/windows/win_serial.cpp impl/windows/list_ports_win.cpp)
add_to_hal_headers(impl/windows/win.h  impl/windows/win_serial.h)
ELSE()	
add_to_hal_sources(impl/unix/unix_serial.cpp impl/unix/list_ports_linux.cpp lock.c  )
add_to_hal_headers(impl/unix/unix.h  impl/unix/unix_serial.h lock.h)
ENDIF()



