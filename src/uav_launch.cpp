#include "include/vectornav_async.h"
#include "yaml-cpp/yaml.h"
#include <pthread.h>

void* parallelLog(void*);
int main(int argc, char *argv[]){

  if (argc == 2) {
    cout << "Using configuration provided by: " << argv[1] << endl;
    YAML::Node config = YAML::LoadFile(argv[1]);
    YAML::Node vec_config = config["vectornav"];
    uint32_t baud = vec_config["baud"].as<int>();
    string   port = vec_config["port"].as<string>();
    uint32_t sample_rate = vec_config["sample_rate"].as<int>();
    vnuav::start_vs(baud, port, sample_rate);
  } else {
    cout << "No configuration provided, will use defaults." << endl;
    vnuav::start_vs(115200, "/dev/ttyS12", 2);
  }

  pthread_t my_thread; 
  pthread_create(&my_thread, NULL, parallelLog, NULL);
  sleep(10);
  std::cout << "Threads will be stopped soon...." << std::endl;
  pthread_cancel(my_thread); //join the thread with the main thread

  vnuav::stop_vs();
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
void* parallelLog(void*){
  while(true){
    // sample at 1Hz for the time being
    sleep(1);
    vnuav::VectorNavData vec_data = vnuav::get_data();
    vnuav::lock_vec_data();
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
    vnuav::unlock_vec_data();
  }
}
