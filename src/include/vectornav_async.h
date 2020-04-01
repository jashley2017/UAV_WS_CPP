#ifndef _VECTORNAV_ASYNC_H_
#define _VECTORNAV_ASYNC_H_

#include <iostream>
#include <fstream>
#include <stack>
#include <chrono>
#include <thread>
#include <future>
#include <mutex>

// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"
#include "vn/compositedata.h"

// We need this file for our sleep function.
#include "vn/thread.h"

#include "yaml-cpp/yaml.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;


namespace vnuav { 

  // Method declarations for future use.
  void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index);
  void start_vs (uint32_t baud, string port, uint32_t sample_rate);
  void stop_vs();

  // Holds that binary data messages for output once program is terminated.
  struct UserData {
    int device_family;
  };

  struct Quaternion { //vec4f
    float x;
    float y;
    float z; 
    float w;
  };
  struct Vector3 { //vec3f
    float x;
    float y;
    float z; 
  };
  struct Imu{ 
    Quaternion orientation; 
    Vector3 angular_velocity;
    Vector3 linear_acceleration; 
    array<float, 9> orientation_covariance; // attitude uncertainty
    array<float, 9> linear_acceleration_covariance; // attitude uncertainty
    array<float, 9> angular_velocity_covariance; // attitude uncertainty
  };
  struct MagneticField {
    Vector3 magnetic_field;
    float magnetic_field_covariance;
  };
  struct NavSatFix {
    float longitude;
    float latitude;
    float altitude;
    float position_covariance[9];
  };
  struct Odometry {
    Quaternion orientation;
    Vector3 position;
    Vector3 twist_linear;
    Vector3 twist_angular;
  };
  struct Temperature { 
    float temperature;
  };
  struct FluidPressure {
    float fluid_pressure;
  };
  struct VectorNavData {
    MagneticField mag;
    Imu imu;
    NavSatFix gps; 
    Odometry odom; 
    Temperature temp;
    FluidPressure barom;
  };

  const char* get_header();
  char* get_data();
  void unlock_vec_data();
  void lock_vec_data();
}
#endif
