/*****************************************************************************
*  EAI TOF LIDAR DRIVER                                                      *
*  Copyright (C) 2018 EAI TEAM  chushuifurong618@eaibot.com.                 *
*                                                                            *
*  This file is part of EAI TOF LIDAR DRIVER.                                *
*                                                                            *
*  @file     Utils.h                                                         *
*  @brief    Driver data type                                                *
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

#include <vector>
#include <cstdlib>
#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string>
#include <string.h>
#include <signal.h>
#include <cerrno>
#include <stdexcept>
#include <csignal>
#if defined(_MSC_VER)
#include <io.h>
#endif

#if !defined(_MSC_VER)
#include <unistd.h>
#endif

#if defined(_WIN32)
#if defined(YDLIDAR_API_STATIC)
    #define YDLIDAR_API_EXPORT
#elif defined(YDLIDAR_API_EXPORTS)
    #define YDLIDAR_API __declspec(dllexport)
#else
    #define YDLIDAR_API __declspec(dllimport)
#endif

#else
#define YDLIDAR_API
#endif // ifdef WIN32

#define UNUSED(x) (void)x

#if !defined(_MSC_VER)
#	define _access access
#endif


#if defined(_WIN32) && !defined(__MINGW32__)
typedef signed char int8_t;
typedef unsigned char uint8_t;
typedef short int16_t;  
typedef unsigned short uint16_t;  
typedef int int32_t;
typedef unsigned int uint32_t;
typedef __int64 int64_t;
typedef unsigned __int64 uint64_t;
#else

#include <stdint.h>

#endif

#define __small_endian

#ifndef __GNUC__
#define __attribute__(x)
#endif

#ifdef _AVR_
typedef uint8_t        _size_t;
#define THREAD_PROC
#elif defined (WIN64)
typedef uint64_t       _size_t;
#define THREAD_PROC    __stdcall
#elif defined (WIN32)
typedef uint32_t       _size_t;
#define THREAD_PROC    __stdcall
#elif defined (_M_X64)
typedef uint64_t       _size_t;
#define THREAD_PROC    __stdcall
#elif defined (__GNUC__)
typedef unsigned long  _size_t;
#define THREAD_PROC
#elif defined (__ICCARM__)
typedef uint32_t       _size_t;
#define THREAD_PROC
#endif

typedef _size_t (THREAD_PROC * thread_proc_t ) ( void * );

typedef int32_t result_t;

#define RESULT_OK      0
#define RESULT_TIMEOUT -1
#define RESULT_FAIL    -2

#define INVALID_TIMESTAMP (0)

#if !defined(_countof)
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#ifndef M_PI
#define M_PI 3.1415926
#endif

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*M_PI/180.)
#endif

#define LIDAR_CMD_STOP                      0x65
#define LIDAR_CMD_SCAN                      0x60
#define LIDAR_CMD_FORCE_SCAN                0x61
#define LIDAR_CMD_RESET                     0x80
#define LIDAR_CMD_FORCE_STOP                0x00
#define LIDAR_CMD_GET_EAI                   0x55
#define LIDAR_CMD_GET_DEVICE_INFO           0x90
#define LIDAR_CMD_GET_DEVICE_HEALTH         0x92
#define LIDAR_ANS_TYPE_DEVINFO              0x4
#define LIDAR_ANS_TYPE_DEVHEALTH            0x6
#define LIDAR_CMD_SYNC_BYTE                 0xA5
#define LIDAR_CMDFLAG_HAS_PAYLOAD           0x80
#define LIDAR_ANS_SYNC_BYTE1                0xA5
#define LIDAR_ANS_SYNC_BYTE2                0x5A
#define LIDAR_ANS_TYPE_MEASUREMENT          0x81
#define LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_SYNC_QUALITY_SHIFT  8
#define LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
#define LIDAR_RESP_MEASUREMENT_DISTANCE_SHIFT 2


#define LIDAR_CMD_RUN_POSITIVE             0x06
#define LIDAR_CMD_RUN_INVERSION            0x07
#define LIDAR_CMD_SET_AIMSPEED_ADDMIC      0x09
#define LIDAR_CMD_SET_AIMSPEED_DISMIC      0x0A
#define LIDAR_CMD_SET_AIMSPEED_ADD         0x0B
#define LIDAR_CMD_SET_AIMSPEED_DIS         0x0C
#define LIDAR_CMD_GET_AIMSPEED             0x0D

#define LIDAR_CMD_SET_SAMPLING_RATE        0xD0
#define LIDAR_CMD_GET_SAMPLING_RATE        0xD1
#define LIDAR_STATUS_OK                    0x0
#define LIDAR_STATUS_WARNING               0x1
#define LIDAR_STATUS_ERROR                 0x2

#define LIDAR_CMD_ENABLE_LOW_POWER         0x01
#define LIDAR_CMD_DISABLE_LOW_POWER        0x02
#define LIDAR_CMD_STATE_MODEL_MOTOR        0x05
#define LIDAR_CMD_ENABLE_CONST_FREQ        0x0E
#define LIDAR_CMD_DISABLE_CONST_FREQ       0x0F

#define LIDAR_CMD_SAVE_SET_EXPOSURE         0x94
#define LIDAR_CMD_SET_LOW_EXPOSURE          0x95
#define LIDAR_CMD_ADD_EXPOSURE       	    0x96
#define LIDAR_CMD_DIS_EXPOSURE       	    0x97

#define LIDAR_CMD_SET_HEART_BEAT        0xD9
#define LIDAR_CMD_SET_SETPOINTSFORONERINGFLAG  0xae

#define PackageSampleMaxLngth 0x100
typedef enum {
	CT_Normal = 0,
	CT_RingStart  = 1,
	CT_Tail,
}CT;


#define Node_Default_Quality (10)
#define Node_Sync 1
#define Node_NotSync 2
#define PackagePaidBytes 10
#define PH 0x55AA


#if defined(_WIN32)
#pragma pack(1)
#endif


namespace ydlidar {


struct node_info {
    uint8_t    sync_flag;
    uint16_t   sync_quality;//!信号质量
    uint16_t   angle_q6_checkbit; //!测距点角度
    uint16_t   distance_q; //! 当前测距点距离
    uint64_t   stamp; //! 时间戳
    uint8_t    scan_frequence;
} __attribute__((packed)) ;

struct PackageNode {
	uint8_t PakageSampleQuality;
	uint16_t PakageSampleDistance;
}__attribute__((packed));

struct node_package {
	uint16_t  package_Head;
	uint8_t   package_CT;
	uint8_t   nowPackageNum;
	uint16_t  packageFirstSampleAngle;
	uint16_t  packageLastSampleAngle;
	uint16_t  checkSum;
	PackageNode  packageSample[PackageSampleMaxLngth];
} __attribute__((packed)) ;

struct node_packages {
	uint16_t  package_Head;
	uint8_t   package_CT;
	uint8_t   nowPackageNum;
	uint16_t  packageFirstSampleAngle;
	uint16_t  packageLastSampleAngle;
	uint16_t  checkSum;
	uint16_t  packageSampleDistance[PackageSampleMaxLngth];
} __attribute__((packed)) ;


struct device_info{
	uint8_t   model; ///< 雷达型号
	uint16_t  firmware_version; ///< 固件版本号
	uint8_t   hardware_version; ///< 硬件版本号
	uint8_t   serialnum[16];    ///< 系列号
} __attribute__((packed)) ;

struct device_health {
	uint8_t   status; ///< 健康状体
	uint16_t  error_code; ///< 错误代码
} __attribute__((packed))  ;

struct sampling_rate {
	uint8_t rate;	///< 采样频率
} __attribute__((packed))  ;

struct scan_frequency {
	uint32_t frequency;	///< 扫描频率
} __attribute__((packed))  ;

struct scan_rotation {
	uint8_t rotation;
} __attribute__((packed))  ;

struct scan_exposure {
	uint8_t exposure;	///< 低光功率模式
} __attribute__((packed))  ;

struct scan_heart_beat {
    uint8_t enable;	///< 掉电保护状态
} __attribute__((packed));

struct scan_points {
	uint8_t flag;
} __attribute__((packed))  ;

struct function_state {
	uint8_t state;
} __attribute__((packed))  ;

struct cmd_packet {
	uint8_t syncByte;
	uint8_t cmd_flag;
	uint8_t size;
	uint8_t data;
} __attribute__((packed)) ;

struct lidar_ans_header {
	uint8_t  syncByte1;
	uint8_t  syncByte2;
	uint32_t size:30;
	uint32_t subType:2;
	uint8_t  type;
} __attribute__((packed));


//! A struct for returning configuration from the YDLIDAR
struct LaserConfig {
	//! Start angle for the laser scan [rad].  0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
	float min_angle;
	//! Stop angle for the laser scan [rad].   0 is forward and angles are measured clockwise when viewing YDLIDAR from the top.
	float max_angle;
	//! Scan resolution [rad].
	float ang_increment;
	//! Scan resoltuion [ns]
	float time_increment;
	//! Time between scans
	float scan_time;
	//! Minimum range [m]
	float min_range;
	//! Maximum range [m]
	float max_range;
	//! Range Resolution [m]
	float range_res;
};


//! A struct for returning laser readings from the YDLIDAR
//! currentAngle = min_angle + ang_increment*index
//! for( int i =0; i < ranges.size(); i++) {
//!     double currentAngle = config.min_angle + i*config.ang_increment;
//!     double currentDistance = ranges[i];
//! }
//!
//!
//!
struct LaserScan {
	//! Array of ranges
	std::vector<float> ranges;
	//! Array of intensities
	std::vector<float> intensities;
	//! Self reported time stamp in nanoseconds
	uint64_t self_time_stamp;
	//! System time when first range was measured in nanoseconds
	uint64_t system_time_stamp;

	//! Configuration of scan
	LaserConfig config;

    ///lidar scan frequency
    float scan_frequency;
};


struct LaserParamCfg
{

    LaserParamCfg(): maxRange(16.0), minRange(0.1), maxAngle(180),
        minAngle(-180),scanFrequency(7), intensity(false),
        fixedResolution(false), exposure(false), heartBeat(false),
        reversion(false), autoReconnect(true), serialBaudrate(115200),
        sampleRate(9),serialPort("") {
        ignoreArray.clear();
    }
    //! Maximum range [m]
    float maxRange;
    //! Minimum range [m]
    float minRange;
    //! Maximum angle [°]
    float maxAngle;
    //! Minimum angle [°]
    float minAngle;
    //! scan frequency [HZ]
    int   scanFrequency;
    //! intensity
    bool  intensity;
    //! fixed angle resolution
    bool  fixedResolution;
    //! low exposure model
    bool  exposure;
    //! heart beat
    bool  heartBeat;
    //! reversion
    bool  reversion;
    //! auto reconnect
    bool  autoReconnect;
    //! baud rate
    int   serialBaudrate;
    //! sample rate
    int   sampleRate;
    //! serial port
    std::string serialPort;
    //! ignore angle
    std::vector<float> ignoreArray;
};

}

// Determine if sigaction is available
#if __APPLE__ || _POSIX_C_SOURCE >= 1 || _XOPEN_SOURCE || _POSIX_SOURCE
#define HAS_SIGACTION
#endif


static volatile sig_atomic_t g_signal_status = 0;

#ifdef HAS_SIGACTION
static struct sigaction old_action;
#else
typedef void (* signal_handler_t)(int);
static signal_handler_t old_signal_handler = 0;
#endif

#ifdef HAS_SIGACTION
inline struct sigaction
set_sigaction(int signal_value, const struct sigaction & action)
#else
inline signal_handler_t
set_signal_handler(int signal_value, signal_handler_t signal_handler)
#endif
{
#ifdef HAS_SIGACTION
  struct sigaction old_action;
  ssize_t ret = sigaction(signal_value, &action, &old_action);
  if (ret == -1)
#else
  signal_handler_t old_signal_handler = std::signal(signal_value, signal_handler);
  // NOLINTNEXTLINE(readability/braces)
  if (old_signal_handler == SIG_ERR)
#endif
  {
    const size_t error_length = 1024;
    // NOLINTNEXTLINE(runtime/arrays)
    char error_string[error_length];
#ifndef _WIN32
#if (defined(_GNU_SOURCE) && !defined(ANDROID))
    char * msg = strerror_r(errno, error_string, error_length);
    if (msg != error_string) {
      strncpy(error_string, msg, error_length);
      msg[error_length - 1] = '\0';
    }
#else
    int error_status = strerror_r(errno, error_string, error_length);
    if (error_status != 0) {
      throw std::runtime_error("Failed to get error string for errno: " + std::to_string(errno));
    }
#endif
#else
    strerror_s(error_string, error_length, errno);
#endif
    // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
    throw std::runtime_error(
      std::string("Failed to set SIGINT signal handler: (" + std::to_string(errno) + ")") +
      error_string);
    // *INDENT-ON*
  }

#ifdef HAS_SIGACTION
  return old_action;
#else
  return old_signal_handler;
#endif
}

inline void trigger_interrupt_guard_condition(int signal_value) {
    g_signal_status = signal_value;
    signal(signal_value, SIG_DFL);
}

inline void
#ifdef HAS_SIGACTION
signal_handler(int signal_value, siginfo_t * siginfo, void * context)
#else
signal_handler(int signal_value)
#endif
{
  // TODO(wjwwood): remove? move to console logging at some point?
  printf("signal_handler(%d)\n", signal_value);

#ifdef HAS_SIGACTION
  if (old_action.sa_flags & SA_SIGINFO) {
    if (old_action.sa_sigaction != NULL) {
      old_action.sa_sigaction(signal_value, siginfo, context);
    }
  } else {
    if (
      old_action.sa_handler != NULL &&  // Is set
      old_action.sa_handler != SIG_DFL &&  // Is not default
      old_action.sa_handler != SIG_IGN)  // Is not ignored
    {
      old_action.sa_handler(signal_value);
    }
  }
#else
  if (old_signal_handler) {
    old_signal_handler(signal_value);
  }
#endif

  trigger_interrupt_guard_condition(signal_value);
}

namespace ydlidar {

inline void init(int argc, char *argv[]) {
    UNUSED(argc);
    UNUSED(argv);
#ifdef HAS_SIGACTION
  struct sigaction action;
  memset(&action, 0, sizeof(action));
  sigemptyset(&action.sa_mask);
  action.sa_sigaction = ::signal_handler;
  action.sa_flags = SA_SIGINFO;
  ::old_action = set_sigaction(SIGINT, action);
  set_sigaction(SIGTERM, action);

#else
  ::old_signal_handler = set_signal_handler(SIGINT, ::signal_handler);
  // Register an on_shutdown hook to restore the old signal handler.
#endif
}
inline bool ok() {
  return g_signal_status == 0;
}
inline void shutdown() {
  trigger_interrupt_guard_condition(SIGINT);
}

inline bool fileExists(const std::string filename) {
    return 0 == _access(filename.c_str(), 0x00 ); // 0x00 = Check for existence only!
}



}

