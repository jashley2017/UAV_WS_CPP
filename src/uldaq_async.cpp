#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include "uldaq.h"
#include "include/uldaq_async.h"

#define MAX_DEV_COUNT  100
#define MAX_STR_LENGTH 64

using namespace std;

namespace daquav { 

  // configuration variables for the DAQ
  unsigned int numDevs = MAX_DEV_COUNT;
  DaqDeviceDescriptor devDescriptors[MAX_DEV_COUNT];
  DaqDeviceHandle handle = 0;
  UlError err = ERR_NO_ERROR;

  // input settings for the DAQ
  int channel_nums;
  int sample_rate;
  long int sample_period; // period in nanoseconds

  // output results for each channel
  float *adc_results;

  // thread handlers
  pthread_t daq_poll_thread;
  pthread_mutex_t adc_res_mut;

  //
  // Locks mutex for adc data control
  //
  void lock_adc() {
    pthread_mutex_lock(&adc_res_mut);
  }

  //
  // Unlocks mutex for adc data control
  //
  void unlock_adc() {
    pthread_mutex_unlock(&adc_res_mut);
  }

  float* get_results(size_t& size) {
    size = channel_nums;
    return adc_results;
  }

  //
  // Polls DAQ at given sample rate
  //
  void* daq_poller(void*) { 
    double data = 0;
    while(true){
      lock_adc();
      for(int chan=0; chan<channel_nums; chan++) {
        err = ulAIn(handle, chan, AI_SINGLE_ENDED, BIP5VOLTS, AIN_FF_DEFAULT, &data);
        adc_results[chan] = data;
      }
      unlock_adc();
      usleep(sample_period);
    }
  }

  //
  // Initialize DAQ handler and start the poller
  //
  void start_daq(int chan_nums, int rate) { 
    
    channel_nums = chan_nums;
    sample_rate = rate;
    sample_period = 1000000L/rate; 
    adc_results = new float[chan_nums];

    // Get descriptors for all of the available DAQ devices
    ulGetDaqDeviceInventory(ANY_IFC, devDescriptors, &numDevs);

    // verify at least one DAQ device is detected
    if (numDevs)
    {
      // get a handle to the DAQ device associated with the first descriptor
      handle = ulCreateDaqDevice(devDescriptors[0]);

      // check if the DAQ device handle is valid
      if (handle)
      {
        // establish a connection to the DAQ device
        err = ulConnectDaqDevice(handle);
        pthread_create(&daq_poll_thread, NULL, daq_poller, NULL);
      }
    }
  }

  //
  // Close connection and stop polling DAQ
  //
  void stop_daq() {
    pthread_cancel(daq_poll_thread);
    ulDisconnectDaqDevice(handle);
    ulReleaseDaqDevice(handle);
  }
}
