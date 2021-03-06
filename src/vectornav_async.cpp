#include <iostream>
#include <fstream>
#include <stack>
#include <chrono>
// These only work in Linux  builds, pthread works in all
// #include <thread>
// #include <future>
// #include <mutex>
#include <pthread.h>
#include <unistd.h>
#include <cstring>

// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"
#include "vn/compositedata.h"

// We need this file for our sleep function.
#include "vn/thread.h"

#include "include/vectornav_async.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

#if _WIN32

float M_PI=3.14159;

#endif

  const char* VN_HEADER = "IMU_ORIENTATION_X,IMU_ORIENTATION_Y,IMU_ORIENTATION_Z,IMU_ORIENTATION_W,IMU_ANGULAR_VELOCITY_X,IMU_ANGULAR_VELOCITY_Y,IMU_ANGULAR_VELOCITY_Z,IMU_LINEAR_ACCELERATION_X,IMU_LINEAR_ACCELERATION_Y,IMU_LINEAR_ACCELERATION_Z,MAG_MAGNETIC_FIELD_X,MAG_MAGNETIC_FIELD_Y,MAG_MAGNETIC_FIELD_Z,GPS_LATITUDE,GPS_LONGITUDE,GPS_ALTITUDE,ODOM_POSITION_X,ODOM_POSITION_Y,ODOM_POSITION_Z,ODOM_TWIST_LINEAR_X,ODOM_TWIST_LINEAR_Y,ODOM_TWIST_LINEAR_Z,ODOM_TWIST_ANGULAR_X,ODOM_TWIST_ANGULAR_Y,ODOM_TWIST_ANGULAR_Z,ODOM_ORIENTATION_X,ODOM_ORIENTATION_Y,ODOM_ORIENTATION_Z,TEMP_TEMPERATURE,BAROM_FLUID_PRESSURE";
  const char* VN_FORMAT = "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f";
  const size_t FORMAT_LEN = 200;

  char vec_data_cstr[FORMAT_LEN] = {0}; 

  array<float, 9> linear_accel_covariance ={ }; 
  array<float, 9> angular_vel_covariance={ };
  array<float, 9> orientation_covariance={ };
  Vector3 initial_position;
  bool initial_position_set = false;
  

  void VecNavUav::unlock_vec_data() {
    pthread_mutex_unlock(&vec_data_mut);
  }

  void VecNavUav::lock_vec_data() {
    pthread_mutex_lock(&vec_data_mut);
  }

  char* VecNavUav::get_data() {
    return vec_data_cstr;
  }

  const char* VecNavUav::get_header() { 
    return VN_HEADER;
  }

  VecNavUav::VecNavUav(uint32_t baud, string port, uint32_t sample_rate){
    // Set connection properties
    //EXAMPLES:
    //const string SensorPort = "COM1";                             // Windows format for physical and virtual (USB) serial port.
    //const string SensorPort = "/dev/ttyS12";                    // Linux format for physical serial port.
    // const string SensorPort = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
    // const string SensorPort = "/dev/tty.usbserial-FTXXXXXX";   // Mac OS X format for virtual (USB) serial port.
    // const string SensorPort = "/dev/ttyS0";                    // CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1.
    //const uint32_t SensorBaudrate = 115200;
      SensorPort = port; 
      SensorBaudrate = baud;
      SampleRate = sample_rate;
  }
  // Holds that binary data messages for output once program is terminated.
  //
  // Start the VectorNav with the given settings
  //
  void VecNavUav::start_vs ()
  {

    // Now let's create a VnSensor object and use it to connect to our sensor.
    cout << "connecting to " << SensorPort << " @ " << SensorBaudrate << endl;
    vs.setResponseTimeoutMs(1000); // Wait for up to 1000 ms for response
    vs.setRetransmitDelayMs(50);  // Retransmit every 50 ms
    vs.connect(SensorPort, SensorBaudrate);

    // Set Sample Frequency in Hz
    vs.writeAsyncDataOutputFrequency(SampleRate);

    int SensorImuRate = 800;

    // Configure binary output message
    BinaryOutputRegister bor(
            ASYNCMODE_PORT1,
            SensorImuRate / SampleRate,  // update rate [ms]
            COMMONGROUP_QUATERNION
            | COMMONGROUP_ANGULARRATE
            | COMMONGROUP_POSITION
            | COMMONGROUP_ACCEL
            | COMMONGROUP_MAGPRES,
            TIMEGROUP_NONE,
            IMUGROUP_NONE,
            GPSGROUP_NONE,
            ATTITUDEGROUP_YPRU, //<-- returning yaw pitch roll uncertainties
            INSGROUP_INSSTATUS
            | INSGROUP_POSLLA
            | INSGROUP_POSECEF
            | INSGROUP_VELBODY
            | INSGROUP_ACCELECEF
            );

	// init everything to zero
	      snprintf(vec_data_cstr, FORMAT_LEN, VN_FORMAT, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0, 
          0.0);

    vs.writeBinaryOutput1(bor);

    UserData user_data;
    user_data.device_family = vs.determineDeviceFamily();
    user_data.vec_data_cstr = vec_data_cstr;
    user_data.vec_data_mut = vec_data_mut;
    vs.registerAsyncPacketReceivedHandler(&user_data, BinaryAsyncMessageReceived);

  }

  //
  // Stop the Vectornav with the given settings
  //
  void VecNavUav::stop_vs(){
    vs.unregisterAsyncPacketReceivedHandler();
    vs.disconnect();
  }


  //
  // Callback function to process data packet from sensor
  //
  void VecNavUav::BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
  {
      VectorNavData vec_data;
      vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);
      UserData user_data = *static_cast<UserData*>(userData);

      auto now = chrono::high_resolution_clock::now();
      auto timestamp = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();

      // IMU
      Imu imu_msg;

      if (cd.hasQuaternion() && cd.hasAngularRate() && cd.hasAcceleration())
      {

          vec4f q = cd.quaternion();
          vec3f ar = cd.angularRate();
          vec3f al = cd.acceleration();

          imu_msg.orientation.x = q[0];
          imu_msg.orientation.y = q[1];
          imu_msg.orientation.z = q[2];
          imu_msg.orientation.w = q[3];

          imu_msg.angular_velocity.x = ar[0];
          imu_msg.angular_velocity.y = ar[1];
          imu_msg.angular_velocity.z = ar[2];
          imu_msg.linear_acceleration.x = al[0];
          imu_msg.linear_acceleration.y = al[1];
          imu_msg.linear_acceleration.z = al[2];

          if (cd.hasAttitudeUncertainty())
          {
              vec3f orientationStdDev = cd.attitudeUncertainty();
              imu_msg.orientation_covariance[0] = orientationStdDev[2]*orientationStdDev[2]*M_PI/180; // Convert to radians pitch
              imu_msg.orientation_covariance[4] = orientationStdDev[1]*orientationStdDev[1]*M_PI/180; // Convert to radians Roll
              imu_msg.orientation_covariance[8] = orientationStdDev[0]*orientationStdDev[0]*M_PI/180; // Convert to radians Yaw
          }

          imu_msg.angular_velocity_covariance = angular_vel_covariance;
          imu_msg.linear_acceleration_covariance = linear_accel_covariance;
          vec_data.imu = imu_msg;
      }

      // Magnetic Field
      if (cd.hasMagnetic())
      {
          vec3f mag = cd.magnetic();
          MagneticField magnet_msg;
          magnet_msg.magnetic_field.x = mag[0];
          magnet_msg.magnetic_field.y = mag[1];
          magnet_msg.magnetic_field.z = mag[2];
          vec_data.mag = magnet_msg;
      }

      // GPS
      if (user_data.device_family != VnSensor::Family::VnSensor_Family_Vn100)
      {
          vec3d lla = cd.positionEstimatedLla();

          NavSatFix gps_msg;
          gps_msg.latitude = lla[0];
          gps_msg.longitude = lla[1];
          gps_msg.altitude = lla[2];

          vec_data.gps = gps_msg;

          // Odometry
          Odometry odom_msg;
          vec3d pos = cd.positionEstimatedEcef();

          if (!initial_position_set)
          {
              initial_position_set = true;
              initial_position.x = pos[0];
              initial_position.y = pos[1];
              initial_position.z = pos[2];
          }

          odom_msg.position.x = pos[0] - initial_position.x;
          odom_msg.position.y = pos[1] - initial_position.y;
          odom_msg.position.z = pos[2] - initial_position.z;

          if (cd.hasQuaternion())
          {
              vec4f q = cd.quaternion();

              odom_msg.orientation.x = q[0];
              odom_msg.orientation.y = q[1];
              odom_msg.orientation.z = q[2];
              odom_msg.orientation.w = q[3];
          }
          if (cd.hasVelocityEstimatedBody())
          {
              vec3f vel = cd.velocityEstimatedBody();

              odom_msg.twist_linear.x = vel[0];
              odom_msg.twist_linear.y = vel[1];
              odom_msg.twist_linear.z = vel[2];
          }
          if (cd.hasAngularRate())
          {
              vec3f ar = cd.angularRate();

              odom_msg.twist_angular.x = ar[0];
              odom_msg.twist_angular.y = ar[1];
              odom_msg.twist_angular.z = ar[2];
          }
          vec_data.odom = odom_msg;
      }

      // Temperature
      if (cd.hasTemperature())
      {
          float temp = cd.temperature();

          Temperature temp_msg;
          temp_msg.temperature = temp;
          vec_data.temp = temp_msg;
      }

      // Barometer
      if (cd.hasPressure())
      {
          float pres = cd.pressure();

          FluidPressure pres_msg;
          pres_msg.fluid_pressure = pres;
          vec_data.barom = pres_msg;
      }

      pthread_mutex_lock(&user_data.vec_data_mut);
      snprintf(user_data.vec_data_cstr, FORMAT_LEN, VN_FORMAT, 
        vec_data.imu.orientation.x ,
        vec_data.imu.orientation.y ,
        vec_data.imu.orientation.z ,
        vec_data.imu.orientation.w ,
        vec_data.imu.angular_velocity.x ,
        vec_data.imu.angular_velocity.y ,
        vec_data.imu.angular_velocity.z ,
        vec_data.imu.linear_acceleration.x ,
        vec_data.imu.linear_acceleration.y ,
        vec_data.imu.linear_acceleration.z ,
        vec_data.mag.magnetic_field.x ,
        vec_data.mag.magnetic_field.y ,
        vec_data.mag.magnetic_field.z ,
        vec_data.gps.latitude ,
        vec_data.gps.longitude ,
        vec_data.gps.altitude ,
        vec_data.odom.position.x ,
        vec_data.odom.position.y,
        vec_data.odom.position.z,
        vec_data.odom.twist_linear.x ,
        vec_data.odom.twist_linear.y ,
        vec_data.odom.twist_linear.z ,
        vec_data.odom.twist_angular.x ,
        vec_data.odom.twist_angular.y ,
        vec_data.odom.twist_angular.z ,
        vec_data.odom.orientation.x ,
        vec_data.odom.orientation.y ,
        vec_data.odom.orientation.z ,
        vec_data.temp.temperature ,
        vec_data.barom.fluid_pressure );
      pthread_mutex_unlock(&user_data.vec_data_mut);
  }
