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

namespace vnuav { 

  VectorNavData vec_data;
  pthread_mutex_t vec_data_mut;
  VnSensor vs;
  array<float, 9> linear_accel_covariance ={ }; 
  array<float, 9> angular_vel_covariance={ };
  array<float, 9> orientation_covariance={ };
  Vector3 initial_position;
  bool initial_position_set = false;
  
  VectorNavData get_data() {
    return vec_data;
  }
  void unlock_vec_data() {
    pthread_mutex_unlock(&vec_data_mut);
  }

  void lock_vec_data() {
    pthread_mutex_lock(&vec_data_mut);
  }



  // Holds that binary data messages for output once program is terminated.
  //
  // Start the VectorNav with the given settings
  //
  void start_vs (uint32_t baud, string port, uint32_t sample_rate)
  {
    // Set connection properties
    string SensorPort = port;
    uint32_t SensorBaudrate = baud;

    //EXAMPLES:
    //const string SensorPort = "COM1";                             // Windows format for physical and virtual (USB) serial port.
    //const string SensorPort = "/dev/ttyS12";                    // Linux format for physical serial port.
    // const string SensorPort = "/dev/ttyUSB0";                  // Linux format for virtual (USB) serial port.
    // const string SensorPort = "/dev/tty.usbserial-FTXXXXXX";   // Mac OS X format for virtual (USB) serial port.
    // const string SensorPort = "/dev/ttyS0";                    // CYGWIN format. Usually the Windows COM port number minus 1. This would connect to COM1.
    //const uint32_t SensorBaudrate = 115200;

    // Now let's create a VnSensor object and use it to connect to our sensor.
    cout << "connecting to " << SensorPort << " @ " << SensorBaudrate << endl;
    vs.setResponseTimeoutMs(1000); // Wait for up to 1000 ms for response
    vs.setRetransmitDelayMs(50);  // Retransmit every 50 ms
    vs.connect(SensorPort, SensorBaudrate);

    // Set Sample Frequency in Hz
    vs.writeAsyncDataOutputFrequency(sample_rate);

    int SensorImuRate = 800;

    // Configure binary output message
    BinaryOutputRegister bor(
            ASYNCMODE_PORT1,
            SensorImuRate / sample_rate,  // update rate [ms]
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


    vs.writeBinaryOutput1(bor);

    UserData user_data;
    user_data.device_family = vs.determineDeviceFamily();
    vs.registerAsyncPacketReceivedHandler(&user_data, BinaryAsyncMessageReceived);

  }

  //
  // Stop the Vectornav with the given settings
  //
  void stop_vs(){
    vs.unregisterAsyncPacketReceivedHandler();
    vs.disconnect();
  }



  //
  // Callback function to process data packet from sensor
  //
  void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
  {
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
          pthread_mutex_lock(&vec_data_mut);
          vec_data.imu = imu_msg;
          pthread_mutex_unlock(&vec_data_mut);
      }

      // Magnetic Field
      if (cd.hasMagnetic())
      {
          vec3f mag = cd.magnetic();
          MagneticField magnet_msg;
          magnet_msg.magnetic_field.x = mag[0];
          magnet_msg.magnetic_field.y = mag[1];
          magnet_msg.magnetic_field.z = mag[2];
          pthread_mutex_lock(&vec_data_mut);
          vec_data.mag = magnet_msg;
          pthread_mutex_unlock(&vec_data_mut);
      }

      // GPS
      if (user_data.device_family != VnSensor::Family::VnSensor_Family_Vn100)
      {
          vec3d lla = cd.positionEstimatedLla();

          NavSatFix gps_msg;
          gps_msg.latitude = lla[0];
          gps_msg.longitude = lla[1];
          gps_msg.altitude = lla[2];

          pthread_mutex_lock(&vec_data_mut);
          vec_data.gps = gps_msg;
          pthread_mutex_unlock( &vec_data_mut );

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
          pthread_mutex_lock(&vec_data_mut);
          vec_data.odom = odom_msg;
          pthread_mutex_unlock(&vec_data_mut);
      }

      // Temperature
      if (cd.hasTemperature())
      {
          float temp = cd.temperature();

          Temperature temp_msg;
          temp_msg.temperature = temp;
          pthread_mutex_lock(&vec_data_mut);
          vec_data.temp = temp_msg;
          pthread_mutex_unlock(&vec_data_mut);
      }

      // Barometer
      if (cd.hasPressure())
      {
          float pres = cd.pressure();

          FluidPressure pres_msg;
          pres_msg.fluid_pressure = pres;
          pthread_mutex_lock(&vec_data_mut);
          vec_data.barom = pres_msg;
          pthread_mutex_unlock(&vec_data_mut);
      }
  }
}
