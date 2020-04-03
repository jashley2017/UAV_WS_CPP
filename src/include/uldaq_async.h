#ifndef _ULDAQ_ASYNC_H_
#define _ULDAQ_ASYNC_H_

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <chrono>
#include "uldaq.h"
using namespace std;

namespace daquav {
  //
  // Retrieve current results
  //
  double* get_results(size_t&);

  const char* get_header();

  //
  // Initialize DAQ handler and start the poller
  //
  void start_daq(int, int, double, chrono::high_resolution_clock::time_point);

  //
  // Close connection and stop polling DAQ
  //
  void stop_daq();

  // event callback
  void eventCallbackFunction(DaqDeviceHandle daqDeviceHandle, DaqEventType eventType, unsigned long long eventData, void* userData);
}

#endif
