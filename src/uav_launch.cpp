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
void* wait_for_sig(void*);
void log_vec_data(stringstream& curr_file);
void log_adc_data(stringstream& curr_file);
long long minimum_period_in_nanoseconds(uint32_t rate_array[], int size);

auto start_time = chrono::high_resolution_clock::now();
long long min_per = 100000; // 100 us 
volatile bool keep_logging = true;
DaqUav *daq; 

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
  daq = new DaqUav(low_chan, high_chan, dac_rate); 
  daq->start_daq();

  // Start Logging Thread
  uint32_t sample_rates[2]= {vec_rate, dac_rate};
  min_per = minimum_period_in_nanoseconds(sample_rates, 2);

  pthread_t my_thread; 
  pthread_create(&my_thread, NULL, parallelLog, static_cast<void*>(&csv_filename));

  pthread_t sigwait;
  pthread_create(&sigwait, NULL, wait_for_sig, NULL);

  // TODO: parallelLog is getting slept by some other thread and causing increased delay
  //        when it is set to max prio, the delay is what is expected
  struct sched_param thread_prio;
  thread_prio.sched_priority = sched_get_priority_max(SCHED_FIFO);
  int ret = pthread_setschedparam(my_thread, SCHED_FIFO, &thread_prio);

  // Hold until user says to terminate
  cout << "Press Enter to Stop Logging"; 
  pthread_join(sigwait, NULL);
  keep_logging = false;
  cout << "Logging is finishing up...." << endl;
  pthread_join(my_thread, NULL); //join the thread with the main thread

  // Stop Sensors
  daq->stop_daq();
  vnuav::stop_vs();

  return 0;
}

void* wait_for_sig(void*){
  string _dummy;
  getline(cin, _dummy);
  return NULL;
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
  long long period_ns = (1/(double)max_rate)*1000000000L;
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
  ostringstream full_file;
  full_file << *(static_cast<string*>(filename)) << file_count << ".csv";

  // open file and buffer
  ofstream curr_file; 
  curr_file.open(full_file.str());
  stringstream buffer_str;

  // setup time variables
  chrono::high_resolution_clock::time_point curr_time;
  long long time_now;
  chrono::high_resolution_clock::time_point end_time;
  long long process_time;

  // let all sensors start up and then do an initial run to check how much time processing takes
  usleep(1000);
  curr_time = chrono::high_resolution_clock::now();

  const int test_count = 100;
  int run_count = 0;
  while(run_count < test_count) { 
    time_now = chrono::duration_cast<chrono::microseconds>(curr_time - start_time).count();
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
      curr_file << vnuav::get_header() << "," << DaqUav::get_header() << "\n";
    }
    run_count++;
  }
  end_time = chrono::high_resolution_clock::now();
  process_time = (chrono::duration_cast<chrono::microseconds>(end_time - curr_time).count())/test_count;

  // setup the time delay needed and adjust for processing time
  struct timespec sample_period = {0};
  sample_period.tv_sec = 0;
  sample_period.tv_nsec = min_per;

  if(process_time*1000>sample_period.tv_nsec){
    cout << "WARNING: cannot garuntee capture of all data at this sample rate. (too high)" << endl;
    sample_period.tv_nsec = 0;
  }
  else { 
    sample_period.tv_nsec = sample_period.tv_nsec- process_time*1000;
  }

  // start main loop
  curr_file.close();
  curr_file.open(full_file.str(), ofstream::out | ofstream::trunc);
  curr_file << "TIME," << vnuav::get_header() << "," << DaqUav::get_header() << "\n";

  while(keep_logging) {
    nanosleep(&sample_period, (struct timespec *)NULL);
    curr_time = chrono::high_resolution_clock::now();
    time_now = chrono::duration_cast<chrono::microseconds>(curr_time - start_time).count();
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
      curr_file << "TIME," << vnuav::get_header() << "," << DaqUav::get_header() << "\n";
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
    double* adc_res = daq->get_results(size);
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
