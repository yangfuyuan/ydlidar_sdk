YDLIDAR SDK PACKAGE V1.3.7
=====================================================================

SDK [test](https://github.com/yangfuyuan/ydlidar_sdk) application for YDLIDAR

Visit EAI Website for more details about [YDLIDAR](http://www.ydlidar.com/) .

How to build YDLIDAR SDK samples
=====================================================================

    $ git clone https://github.com/yangfuyuan/ydlidar_sdk

    $ cd ydlidar_sdk

    $ git checkout master

    $ cd ..

    $ mkdir build

    $ cd build

    $ cmake ../ydlidar_sdk

    $ make			###linux

    $ vs open Project.sln	###windows

How to run YDLIDAR SDK samples
=====================================================================

linux:

    $ ./ydlidar_test

    $ Please enter the lidar port:/dev/ttyUSB0

    $ Please enter the lidar baud rate:115200

    $ Please enter the lidar intensity:0


windows:

    $ ydlidar_test.exe

    $ Please enter the lidar port:COM3

    $ Please enter the lidar baud rate:115200

    $ Please enter the lidar intensity:0

=====================================================================

You should see YDLIDAR's scan result in the console:

 YDLIDAR C++ TEST

Please enter the lidar port:/dev/ttyUSB0

Please enter the lidar baud rate:230400

Please enter the lidar intensity:0

SDK Version: 1..3.7

LIDAR Version: 1.3.7

fhs_lock: creating lockfile:      14323

firmware: 521

[YDLIDAR] Connection established in [/dev/ttyUSB0]:

Firmware version: 2.0.9

Hardware version: 2

Model: G4

Serial: 2018041900000040

[YDLIDAR INFO] Current Sampling Rate : 9K

[YDLIDAR INFO] Current Scan Frequency : 7.000000Hz

Get Lidar data timeout

received data sample size:1195

frame timestamp is:1534228901618879800

received timestamp is:1534228901742776000

received data sample size:1203

frame timestamp is:1534228901741838800

received timestamp is:1534228901876750000

^Csignal_handler(2)

received data sample size:1957

frame timestamp is:1534228901875864800

received timestamp is:1534228902127891000

fhs_unlock: Removing LockFile




Lidar point data structure
=====================================================================

data structure:

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

      };

example:

    for(size_t i =0; i < scan.ranges.size(); i++) {

      // current angle
      double angle = scan.config.min_angle + i*scan.config.ang_increment;

      //current distance
      double distance = scan.ranges[i];

      //current intensity
      int intensity = scan.intensities[i];

    }

code:

      void LaserScanCallback(const LaserScan& scan) {

          std::cout<< "received scan size: "<< scan.ranges.size()<<std::endl;

    	  std::cout<< "scan   system time: "<< scan.system_time_stamp<<std::endl;

    	  std::cout<< "scan     self time: "<< scan.self_time_stamp<<std::endl;

    	  std::cout<< "scan     frequency: "<< 1000000000.0/scan.config.scan_time << "HZ"<<std::endl;

          for(size_t i =0; i < scan.ranges.size(); i++) {
            
            // current angle
            double angle = scan.config.min_angle + i*scan.config.ang_increment;

            //current distance
            double distance = scan.ranges[i];

            //current intensity
            int intensity = scan.intensities[i];

          }

        }


# Examples

samples in the file test.cpp/test1.cpp.

### SIMPLE USAGE

```c++
    try {

        LIDAR ydlidar;

    	LaserParamCfg cfg;

        ydlidar.RegisterLIDARDataCallback(&LaserScanCallback);

        ydlidar.UpdateLidarParamCfg(cfg);

         while(ydlidar::ok()){

             try {

                 ydlidar.spinOnce();

             }catch(TimeoutException& e) {

                 std::cout<< e.what()<<std::endl;

             }catch(CorruptedDataException& e) {

                 std::cout<< e.what()<<std::endl;

             }catch(DeviceException& e) {

                 std::cout<< e.what()<<std::endl;

                 break;
             }

         }

    }catch(TimeoutException& e) {

        std::cout<< e.what()<<std::endl;

    }catch(CorruptedDataException& e) {

        std::cout<< e.what()<<std::endl;

    }catch(DeviceException& e) {

        std::cout<< e.what()<<std::endl;

    }
```





Upgrade Log
=====================================================================

2018-08-14 version:1.3.7

  1.update sdk interface function.
