#include "include/vectornav_async.h"
#include "include/uldaq_async.h"
#include "yaml-cpp/yaml.h"
#include "logtk/logtk/logtk.hpp"
#include <pthread.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <string>
#include <time.h>

using namespace std;

void* wait_for_sig(void*);

auto start_time = chrono::high_resolution_clock::now();

int main(int argc, char *argv[]){

  // Set initial/default values for each param expected
  uint32_t vec_baud = 115200; 
  string   vec_port = "/dev/ttyS12"; 
  uint32_t vec_rate = 2;

  double dac_rate = 1.2345; 
  uint32_t low_chan = 1;
  uint32_t high_chan = 1;

  string csv_filename;
  string csv_pathname;

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
    csv_pathname = config["outpath"].as<string>().c_str();

  } else {
    cout << "No configuration provided, will use defaults." << endl;
    csv_filename = "test";
    csv_pathname = ".";
  }


  // Setup threadsafe log 
  if(!logtk::start_logger(csv_filename,csv_pathname))
  {
    cerr << "ERROR - could not start log";
    return 1;
  }
  logtk::log("Log Start");

  // Start Sensors
  vnuav::start_vs(vec_baud, vec_port, vec_rate, start_time);
  daquav::start_daq(low_chan, high_chan, dac_rate, start_time);

  // Hold until user says to terminate
  pthread_t sigwait;
  pthread_create(&sigwait, NULL, wait_for_sig, NULL);

  cout << "Press Enter to Stop Logging"; 
  pthread_join(sigwait, NULL);
  cout << "Logging is finishing up...." << endl;

  // Stop Sensors
  daquav::stop_daq();
  vnuav::stop_vs();

  logtk::stop_logger();

  return 0;
}

void* wait_for_sig(void*){
  string _dummy;
  getline(cin, _dummy);
  return NULL;
}
