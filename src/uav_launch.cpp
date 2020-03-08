#include "include/vectornav_async.h"
#include "include/uldaq_async.h"
#include "yaml-cpp/yaml.h"
#include <pthread.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>


using namespace std;

void* parallelLog(void*);
void log_vec_data(stringstream& curr_file);
void log_adc_data(stringstream& curr_file);
void test_times();

bool log_run;

auto start_time = chrono::high_resolution_clock::now();

int main(int argc, char *argv[]){

  uint32_t vec_baud = 115200; 
  string   vec_port = "/dev/ttyS12"; 
  uint32_t vec_rate = 2;

  double dac_rate = 1.2345; 
  uint32_t dac_chans = 1;

  string csv_filename;

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
    dac_rate = dac_config["sample_rate"].as<double>();

    //outfile 
    csv_filename = config["outfile"].as<string>().c_str();

  } else {
    cout << "No configuration provided, will use defaults." << endl;
    csv_filename = "./test";
  }

  // Start Sensors
  vnuav::start_vs(vec_baud, vec_port, vec_rate);
  daquav::start_daq(dac_chans, dac_rate);

  pthread_t my_thread; 
  log_run = true;
  pthread_create(&my_thread, NULL, parallelLog, static_cast<void*>(&csv_filename));
  pthread_join(my_thread, NULL); //join the thread with the main thread
  std::cout << "Threads will be stopped soon...." << std::endl;

  // Stop Sensors
  daquav::stop_daq();
  vnuav::stop_vs();

  return 0;
}

void* parallelLog(void* filename) {

  long log_rotate = 2000000;
  long line_count = 0;
  int file_count = 0;
  // This part is difficulty of passing a string as a pointer. Please ignore
  std::ostringstream full_file;
  full_file << *(static_cast<string*>(filename)) << file_count << ".csv";
  ofstream curr_file; 
  curr_file.open(full_file.str());
  
  curr_file << "Time,OdomX,OdomY,OdomZ,Temp,Pressure,DAC1,DAC2,DAC3,DAC4,DAC4,DAC5,DAC6,DAC7,DAC8\n";

  stringstream buffer_str;

  while(file_count < 4){ // TODO: replace with proper thread handler
    // we should handle sleep dynamically based on time of most recent sample taken and sample rate
    usleep(20);
    auto curr_time = chrono::high_resolution_clock::now();
    long long time_now = chrono::duration_cast<chrono::milliseconds>(curr_time - start_time).count();
    buffer_str << time_now << ",";
    log_vec_data(buffer_str);
    buffer_str << ",";
    log_adc_data(buffer_str);
    buffer_str << "\n";
    curr_file << buffer_str.rdbuf();
    buffer_str.clear();
    line_count++;
    if(line_count > log_rotate) {
	curr_file.close();
	line_count = 0;
	file_count++;
	full_file.str("");
	full_file << *(static_cast<string*>(filename)) << file_count << ".csv";
	curr_file.open(full_file.str());
	curr_file << "OdomX,OdomY,OdomZ,Temp,Pressure,DAC1,DAC2,DAC3,DAC4,DAC4,DAC5,DAC6,DAC7,DAC8\n";
    }
  }
  curr_file.close();
  return NULL;
}

void log_adc_data(stringstream& curr_file) {
    size_t size;
    double* adc_res = daquav::get_results(size);
    curr_file << adc_res[0]; // precede the comma without if statements
    for(int i=1; i < size; i++){ 
      curr_file << "," << adc_res[i];
    }
    delete adc_res;
}

void log_vec_data(stringstream& curr_file) {
    vnuav::VectorNavData vec_data = vnuav::get_data();
    vnuav::lock_vec_data();
    curr_file << 
    vec_data.odom.position.x << "," <<
    vec_data.odom.position.y << "," <<
    vec_data.odom.position.z << "," <<
    vec_data.temp.temperature << "," <<
    vec_data.barom.fluid_pressure;
    //   cout << "VectorNav: " << 
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
    //   vec_data.odom.position.x << "," <<
    //   vec_data.odom.position.y <<"," <<
    //   vec_data.odom.position.z <<"," <<
    //   vec_data.odom.twist_linear.x <<"," <<
    //   vec_data.odom.twist_linear.y <<"," <<
    //   vec_data.odom.twist_linear.z <<"," <<
    //   vec_data.odom.twist_angular.x <<"," <<
    //   vec_data.odom.twist_angular.y <<"," <<
    //   vec_data.odom.twist_angular.z <<"," <<
    //   vec_data.odom.orientation.x <<"," <<
    //   vec_data.odom.orientation.y <<"," <<
    //   vec_data.odom.orientation.z <<"," <<
    //   vec_data.temp.temperature << "," <<
    //   vec_data.barom.fluid_pressure << endl;
    vnuav::unlock_vec_data();
}
