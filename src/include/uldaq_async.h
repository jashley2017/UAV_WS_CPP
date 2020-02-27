#ifndef _ULDAQ_ASYNC_H_
#define _ULDAQ_ASYNC_H_

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include "uldaq.h"
using namespace std;

namespace daquav {
  //
  // Locks mutex for adc data control
  //
  void lock_adc();

  //
  // Unlocks mutex for adc data control
  //
  void unlock_adc();

  float* get_results(size_t&);

  //
  // Polls DAQ at given sample rate
  //
  void daq_poller();

  //
  // Initialize DAQ handler and start the poller
  //
  void start_daq(int, int);

  //
  // Close connection and stop polling DAQ
  //
  void stop_daq();
}

#endif
