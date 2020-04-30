#ifndef _ULDAQ_ASYNC_H_
#define _ULDAQ_ASYNC_H_

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include "uldaq.h"
using namespace std;

  struct ScanEventParameters
  {
    double* buffer;	// data buffer
    int bufferSize;	// data buffer size
    int lowChan;	// first channel in acquisition
    int highChan;	// last channel in acquisition
    double rate; // actual sample rate
    long long index; // memory offset to start the buffer
  };

  typedef struct ScanEventParameters ScanEventParameters;

  class DaqUav {
    public:
      DaqUav(int lchan, int hchan, double rate); // init parameters for the DAQ
      void start_daq(); // start the daq in async Ain mode
      double* get_results(size_t& size); // get current results from the buffer
      void stop_daq(); // stop the async handler and suspend DAQ 
      static const char* get_header(); // get the CSV header formating for the DAQ
    private:
      static void eventCallbackFunction(DaqDeviceHandle daqDeviceHandle, DaqEventType eventType, unsigned long long eventData, void* userData); // async event handler
      ScanEventParameters scan_data; // data about the scan pass along to the static event handler
      double* buffer;
  };

#endif
