#include "include/vectornav_async.h"
#include "include/uldaq_async.h"
#include "yaml-cpp/yaml.h"
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <time.h>

using namespace std;

void* parallelLog(void*);
void log_vec_data(stringstream& curr_file);
void log_adc_data(stringstream& curr_file);
long long minimum_period_in_nanoseconds(uint32_t rate_array[], int size);

auto start_time = chrono::high_resolution_clock::now();
long long min_per = 100000; // 100 us 
volatile bool keep_logging = true;

int main(int argc, char *argv[]){

  // Set initial/default values for each param expected
  uint32_t vec_baud = 115200; 
  string   vec_port = "/dev/ttyS12"; 
  uint32_t vec_rate = 2;

  double dac_rate = 1.2345; 
  uint32_t low_chan = 1;
  uint32_t high_chan = 1;

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
    low_chan = dac_config["low_chan"].as<int>();
    high_chan = dac_config["high_chan"].as<int>();
    dac_rate = dac_config["sample_rate"].as<double>();

    //outfile 
    csv_filename = config["outfile"].as<string>().c_str();

  } else {
    cout << "No configuration provided, will use defaults." << endl;
    csv_filename = "./test";
  }

  // Start Sensors
  vnuav::start_vs(vec_baud, vec_port, vec_rate);
  daquav::start_daq(low_chan, high_chan, dac_rate);

  // Start Logging Thread
  uint32_t sample_rates[2]= {vec_rate, dac_rate};
  min_per = minimum_period_in_nanoseconds(sample_rates, 2);

  pthread_t my_thread; 
  pthread_create(&my_thread, NULL, parallelLog, static_cast<void*>(&csv_filename));

  // TODO: parallelLog is getting slept by some other thread and causing increased delay
  //        when it is set to max prio, the delay is what is expected
  struct sched_param thread_prio;
  thread_prio.sched_priority = sched_get_priority_max(SCHED_FIFO);
  int ret = pthread_setschedparam(my_thread, SCHED_FIFO, &thread_prio);

  // Hold until user says to terminate
  string _dummy;
  cout << "Press Enter to Stop Logging"; 
  cin >> _dummy;

  keep_logging = false;
  pthread_join(my_thread, NULL); //join the thread with the main thread
  std::cout << "Logging is finishing up...." << std::endl;

  // Stop Sensors
  daquav::stop_daq();
  vnuav::stop_vs();

  return 0;
}

//
// returns the minimum log period neccesary to capture all sample rates.
// Assumes sample rates are in units of Hz
//
long long minimum_period_in_nanoseconds(uint32_t rate_array[], int size){
  int max_rate = 0;
  for(int i=0; i<size; i++){
    if (rate_array[i] > max_rate) {
      max_rate = rate_array[i];
    }
  }
  long long period_ns = (double)(1/max_rate)*1000000000L;
  return period_ns;
}

//
// continuously logs the data in each sensors' buffer to a CSV file
//
void* parallelLog(void* filename) {
  long log_rotate = (long)100*(1/(min_per*0.000000001)); // this rotates the log every 100s in terms of sample rate lines
  long line_count = 0;
  int file_count = 0;
  // This part is difficulty of passing a string as a pointer. Please ignore
  std::ostringstream full_file;
  full_file << *(static_cast<string*>(filename)) << file_count << ".csv";
  ofstream curr_file; 
  curr_file.open(full_file.str());
  
  // TODO: replace with constants
  curr_file << vnuav::get_header() << "," << "DAC1,DAC2,DAC3,DAC4,DAC4,DAC5,DAC6,DAC7,DAC8\n";
  stringstream buffer_str;

  struct timespec sample_period = {0};
  sample_period.tv_sec = 0;
  sample_period.tv_nsec = min_per;
  
  while(keep_logging) {
    nanosleep(&sample_period, (struct timespec *)NULL);
    auto curr_time = chrono::high_resolution_clock::now();
    long long time_now = chrono::duration_cast<chrono::microseconds>(curr_time - start_time).count();
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
      curr_file << vnuav::get_header() << "," << "DAC1,DAC2,DAC3,DAC4,DAC4,DAC5,DAC6,DAC7,DAC8\n";
    }
  }
  curr_file.close();
  return NULL;
}

//
// retrieves buffer from ADC
//
void log_adc_data(stringstream& curr_file) {
    size_t size;
    double* adc_res = daquav::get_results(size);
    curr_file << adc_res[0]; // precede the comma without if statements
    for(int i=1; i < size; i++){ 
      curr_file << "," << adc_res[i];
    }
}

//
// retrieves buffer from vectornav
//
void log_vec_data(stringstream& curr_file) {
    vnuav::lock_vec_data();
    char* vec_data_cstr = vnuav::get_data();
    curr_file << vec_data_cstr;
    vnuav::unlock_vec_data();
}
