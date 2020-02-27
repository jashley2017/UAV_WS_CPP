#include "include/vectornav_async.h"
#include "include/uldaq_async.h"
#include "yaml-cpp/yaml.h"
#include <pthread.h>
#include <unistd.h>

void* parallelLog(void*);
void log_vec_data();
void log_adc_data();

int main(int argc, char *argv[]){

  uint32_t vec_baud = 115200; 
  string   vec_port = "/dev/ttyS12"; 
  uint32_t vec_rate = 2;

  uint32_t dac_rate = 1000; 
  uint32_t dac_chans = 1;


  if (argc == 2) {
    cout << "Using configuration provided by: " << argv[1] << endl;
    YAML::Node config = YAML::LoadFile(argv[1]);

    // vectornav config
    YAML::Node vec_config = config["vectornav"];
    vec_baud = vec_config["baud"].as<int>();
    vec_port = vec_config["port"].as<string>();
    vec_rate = vec_config["sample_rate"].as<int>();

    // DAC config
    YAML::Node dac_config = config["dac"];
    dac_chans = dac_config["channel_count"].as<int>();
    dac_rate = dac_config["sample_rate"].as<int>();

  } else {
    cout << "No configuration provided, will use defaults." << endl;
  }

  // Start Sensors
  vnuav::start_vs(vec_baud, vec_port, vec_rate);
  daquav::start_daq(dac_chans, dac_rate);

  pthread_t my_thread; 
  pthread_create(&my_thread, NULL, parallelLog, NULL);
  sleep(10);
  std::cout << "Threads will be stopped soon...." << std::endl;
  pthread_cancel(my_thread); //join the thread with the main thread

  // Stop Sensors
  daquav::stop_daq();
  vnuav::stop_vs();
  

  return 0;
}
void* parallelLog(void*) {
  while(true){
    // sample at 1Hz for the time being
    sleep(1);
    log_vec_data();
    log_adc_data();
  }
}

void log_adc_data(){
    daquav::lock_adc();
    size_t size;
    float* adc_res = daquav::get_results(size);
    cout << "ADC Results";
    for(int i; i < size; i++){ 
      cout << "Channel " << i << " " << adc_res[i] << "\t";
    }
    cout << endl; 
    daquav::unlock_adc();
}

void log_vec_data(){
    vnuav::VectorNavData vec_data = vnuav::get_data();
    vnuav::lock_vec_data();
    cout << "VectorNav: " << 
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
