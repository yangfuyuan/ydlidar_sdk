/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     unix_timer.cpp                                                  *
*  @brief    time                                                            *
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
#if !defined(_WIN32)
#include "timer.h"

namespace impl{
	uint32_t getHDTimer() {
		struct timespec t;
		t.tv_sec = t.tv_nsec = 0;
		clock_gettime(CLOCK_MONOTONIC, &t);
		return t.tv_sec*1000L + t.tv_nsec/1000000L;
	}
    uint64_t getCurrentTime()
    {
#if HAS_CLOCK_GETTIME
        struct timespec  tim;
        clock_gettime(CLOCK_REALTIME, &tim);
        return  (uint64_t)(tim.tv_sec*1000000000LL + tim.tv_nsec);
#else
        struct timeval timeofday;
        gettimeofday(&timeofday,NULL);
        return  (uint64_t)( timeofday.tv_sec*1000000000LL + timeofday.tv_usec * 1000);
 #endif
    }
}
#endif
