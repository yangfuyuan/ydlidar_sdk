/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     timer.h                                                         *
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
#pragma once
#include <Utils.h>
#include <assert.h>
#include <time.h>
#include <inttypes.h>



#define BEGIN_STATIC_CODE( _blockname_ ) \
	static class _static_code_##_blockname_ {   \
	public:     \
	_static_code_##_blockname_ ()


#define END_STATIC_CODE( _blockname_ ) \
	}   _instance_##_blockname_;


#if defined(_WIN32)  
#include <windows.h>
#define delay(x)   ::Sleep(x)
#else
#include <sys/time.h>
#include <unistd.h>

static inline void delay(uint32_t ms){
	while (ms>=1000){
		usleep(1000*1000);
		ms-=1000;
	};
	if (ms!=0){
		usleep(ms*1000);
	}
}
#endif


namespace impl{

#if defined(_WIN32)  
	void HPtimer_reset();
#endif
    uint32_t getHDTimer();
    uint64_t getCurrentTime();
}


#define getms() impl::getHDTimer()
#define getTime() impl::getCurrentTime()
