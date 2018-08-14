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
#*  2018/08/09 | 1.0.0     | Tony.Yang      | Create file                     *
#*----------------------------------------------------------------------------*
#*                                                                            *
#*****************************************************************************/

include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )

subdirlist(SUBDIRS ${CMAKE_CURRENT_SOURCE_DIR})

foreach(subdir ${SUBDIRS})
    include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/${subdir} )

  add_subdirectory(${subdir})
endforeach()
