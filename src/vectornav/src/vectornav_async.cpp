#include <iostream>
#include <fstream>
#include <stack>
#include <chrono>
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

#include "yaml-cpp/yaml.h"

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

#if _WIN32

float M_PI=3.14159;

#endif

// Holds that binary data messages for output once program is terminated.
VnSensor vs;
struct UserData {
    int device_family;
};

// Method declarations for future use.
void BinaryAsyncMessageReceived(void* userData, Packet& p, size_t index);

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
          //GPSGROUP_NONE
          );

  // OLD: Setup Binary Register for Sampling
	//BinaryOutputRegister bor(
	//	ASYNCMODE_PORT1,
	//	200,
	//	COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL,	// Note use of binary OR to configure flags.
	//	TIMEGROUP_NONE,
	//	IMUGROUP_NONE,
  //  GPSGROUP_NONE,
	//	ATTITUDEGROUP_NONE,
	//	INSGROUP_NONE,
  //  GPSGROUP_NONE);

	vs.writeBinaryOutput1(bor);

  UserData user_data;
  user_data.device_family = vs.determineDeviceFamily();
	vs.registerAsyncPacketReceivedHandler(&user_data, BinaryAsyncMessageReceived);

}

void stop_vs(){
	vs.unregisterAsyncPacketReceivedHandler();
	vs.disconnect();
}

void* parallelLog(void*);

int main(int argc, char *argv[]){

  if (argc == 2) {
    cout << "Using configuration provided by: " << argv[1] << endl;
    YAML::Node config = YAML::LoadFile(argv[1]);
    YAML::Node vec_config = config["vectornav"];
    uint32_t baud = vec_config["baud"].as<int>();
    string   port = vec_config["port"].as<string>();
    uint32_t sample_rate = vec_config["sample_rate"].as<int>();
    start_vs(baud, port, sample_rate);
  } else {
    cout << "No configuration provided, will use defaults." << endl;
    start_vs(115200, "/dev/ttyS12", 2);
  }

  pthread_t my_thread; 
  pthread_create(&my_thread, NULL, parallelLog, NULL);
  sleep(10);
  std::cout << "Threads will be stopped soon...." << std::endl;
  pthread_cancel(my_thread); //join the thread with the main thread

  stop_vs();
  // SOME TESTFILE OUTPUT:
  // ofstream vec_outfile;
  // vec_outfile.open("/home/josh/vec_log.csv");
  // vec_outfile << "Time, Yaw, Pitch, Roll" << endl;
  // while (!vec_data.empty()) {
  //   Imu top_vec = vec_data.top();
  //   vec_data.pop();
  //   vec_outfile << top_vec.t << "," <<
  //     top_vec.y << "," <<
  //     top_vec.p << "," <<
  //     top_vec.r << endl;
  // }
  // vec_outfile.close();
	return 0;
}


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

VectorNavData vec_data;
pthread_mutex_t vec_data_mut;

void* parallelLog(void*){
  while(true){
    // sample at 1Hz for the time being
    sleep(1);
    pthread_mutex_lock(&vec_data_mut);
    // TODO: need to include covariance
    cout <<
    //   vec_data.imu.orientation.x << "," <<
    //   vec_data.imu.orientation.y << "," <<
    //   vec_data.imu.orientation.z << "," <<
    //   vec_data.imu.orientation.w << "," <<
    //   vec_data.imu.angular_velocity.x << "," <<
    //   vec_data.imu.angular_velocity.y << "," <<
    //   vec_data.imu.angular_velocity.z << "," <<
    //   vec_data.imu.linear_acceleration.x << "," <<
    //   vec_data.imu.linear_acceleration.y << "," <<
    //   vec_data.imu.linear_acceleration.z <<"," <<
    //   vec_data.mag.magnetic_field.x <<"," <<
    //   vec_data.mag.magnetic_field.y <<"," <<
    //   vec_data.mag.magnetic_field.z <<"," <<
    //   vec_data.gps.latitude <<"," <<
    //   vec_data.gps.longitude <<"," <<
    //   vec_data.gps.altitude <<"," <<
    vec_data.odom.position.x << "," <<
    vec_data.odom.position.y <<"," <<
    vec_data.odom.position.z <<"," <<
    //   vec_data.odom.twist_linear.x <<"," <<
    //   vec_data.odom.twist_linear.y <<"," <<
    //   vec_data.odom.twist_linear.z <<"," <<
    //   vec_data.odom.twist_angular.x <<"," <<
    //   vec_data.odom.twist_angular.y <<"," <<
    //   vec_data.odom.twist_angular.z <<"," <<
    //   vec_data.odom.orientation.x <<"," <<
    //   vec_data.odom.orientation.y <<"," <<
    //   vec_data.odom.orientation.z <<"," <<
    vec_data.temp.temperature << "," <<
    vec_data.barom.fluid_pressure << endl;
    pthread_mutex_unlock(&vec_data_mut);
  }
}

bool initial_position_set = false;
Vector3 initial_position;

//Unused covariances initilized to zero's
array<float, 9> linear_accel_covariance = { };
array<float, 9> angular_vel_covariance = { };
array<float, 9> orientation_covariance = { };

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

/* OLD
void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
{
  auto now = chrono::high_resolution_clock::now();
  auto msg_time_ms = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()).count();
	if (p.type() == Packet::TYPE_ASCII && p.determineAsciiAsyncType() == VNYPR)
	{
		vec3f ypr;
		p.parseVNYPR(&ypr);
    current_vector.t = msg_time_ms;
    current_vector.y = ypr[0];
    current_vector.p = ypr[1];
    current_vector.r = ypr[2];
		// cout << "ASCII Async YPR: " << ypr << endl;
		return;
	} else if (p.type() == Packet::TYPE_BINARY) {
		// First make sure we have a binary packet type we expect since there
		// are many types of binary output types that can be configured.
		if (!p.isCompatible(
			COMMONGROUP_TIMESTARTUP | COMMONGROUP_YAWPITCHROLL,
			TIMEGROUP_NONE,
			IMUGROUP_NONE,
      GPSGROUP_NONE,
			ATTITUDEGROUP_NONE,
			INSGROUP_NONE,
      GPSGROUP_NONE))
      // Not the type of binary packet we are expecting.
			return;
		// uint64_t timeStartup = p.extractUint64();
		vec3f ypr = p.extractVec3f();
    current_vector.t = msg_time_ms;
    current_vector.y = ypr[0];
    current_vector.p = ypr[1];
    current_vector.r = ypr[2];

	}
  // Debug out for IMU data
  cout << current_vector.t << "," << current_vector.y << "," << current_vector.p << "," << current_vector.r;
}
*/
