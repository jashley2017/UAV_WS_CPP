#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <iostream>
#include "uldaq.h"
#include "include/utility.h"
#include "include/uldaq_async.h"

#define MAX_DEV_COUNT  100
#define MAX_STR_LENGTH 64
#define MAX_SCAN_OPTIONS_LENGTH 256

using namespace std;

namespace daquav { 

  // configuration variables for the DAQ
  unsigned int numDevs = MAX_DEV_COUNT;
  DaqDeviceDescriptor devDescriptors[MAX_DEV_COUNT];
  DaqDeviceInterface interfaceType = ANY_IFC;
  DaqDeviceHandle daqDeviceHandle = 0;
  UlError err = ERR_NO_ERROR;
  ScanStatus status;
  TransferStatus transferStatus;

  // input settings for the DAQ
  int channel_nums;
  int sample_rate;
  int sample_period; // period in microseconds

  // output results for each channel

  // thread handlers
  pthread_t daq_poll_thread;
  pthread_mutex_t adc_res_mut;

  double* buffer = NULL;

  double* get_results(size_t& size) {
    size = channel_nums;
    err = ulAInScanStatus(daqDeviceHandle, &status, &transferStatus);
    double* adc_results = new double[size];
    for (int i = 0; i < channel_nums; i++) {
      adc_results[i] = buffer[i]; 
    }
    return adc_results;
  }


  //
  // Initialize DAQ handler and start the poller
  //
  void start_daq(int chan_nums, double rate) { 
	int descriptorIndex = 0;
	unsigned int numDevs = MAX_DEV_COUNT;

	// set some variables that are used to acquire data
	channel_nums = chan_nums - 1; // zero index
	AiInputMode inputMode;
	Range range;
	int samplesPerChannel = 1;
	double sample_rate = rate;
	ScanOption scanOptions = (ScanOption) (SO_DEFAULTIO | SO_CONTINUOUS);
	AInScanFlag flags = AINSCAN_FF_DEFAULT;

	int daq_supported_channels = 0;
	int index = 0;

	char inputModeStr[MAX_STR_LENGTH];
	char rangeStr[MAX_STR_LENGTH];
	char scanOptionsStr[MAX_SCAN_OPTIONS_LENGTH];

	int chanCount = 0;

	int i = 0;
	char c;

	// Get descriptors for all of the available DAQ devices
	err = ulGetDaqDeviceInventory(interfaceType, devDescriptors, &numDevs);

	if (err != ERR_NO_ERROR) {
		stop_daq();
		return;
	}

	// verify at least one DAQ device is detected
	if (numDevs == 0)
	{
		printf("No DAQ device is detected\n");
		stop_daq();
		return;
	}

	printf("Found %d DAQ device(s)\n", numDevs);
	for (i = 0; i < (int) numDevs; i++) {
		printf("  %s: (%s)\n", devDescriptors[i].productName, devDescriptors[i].uniqueId);
	}

	// get a handle to the DAQ device associated with the first descriptor
	daqDeviceHandle = ulCreateDaqDevice(devDescriptors[descriptorIndex]);

	if (daqDeviceHandle == 0)
	{
		printf ("\nUnable to create a handle to the specified DAQ device\n");
		stop_daq();
		return;
	}

	printf("\nConnecting to device %s - please wait ...\n", devDescriptors[descriptorIndex].devString);

	// establish a connection to the DAQ device
	err = ulConnectDaqDevice(daqDeviceHandle);

	if (err != ERR_NO_ERROR) {
		stop_daq();
		return;
	}


	// allocate a buffer to receive the data
	buffer = new double[channel_nums*samplesPerChannel];

	if(buffer == NULL)
	{
		printf("\nOut of memory, unable to create scan buffer\n");
		stop_daq();
		return;
	}

	// get the first supported analog input mode
	int max_chans = 0;
	err = getAiInfoFirstSupportedInputMode(daqDeviceHandle, &max_chans, &inputMode, inputModeStr);
	// get the first supported analog input range
	err = getAiInfoFirstSupportedRange(daqDeviceHandle, inputMode, &range, rangeStr);

	// start the acquisition of 0 - chan_nums
	err = ulAInScan(daqDeviceHandle, 0, channel_nums, inputMode, range, samplesPerChannel, &sample_rate, scanOptions, flags, buffer);

	// get the initial status of the acquisition
	err = ulAInScanStatus(daqDeviceHandle, &status, &transferStatus);


  }

  //
  // Close connection and stop polling DAQ
  //
  void stop_daq() {
	// stop the acquisition if it is still running
	if (status == SS_RUNNING && err == ERR_NO_ERROR)
	{
		err = ulAInScanStop(daqDeviceHandle);
	}
	// disconnect from the DAQ device
	ulDisconnectDaqDevice(daqDeviceHandle);


	// release the handle to the DAQ device
	if(daqDeviceHandle)
		ulReleaseDaqDevice(daqDeviceHandle);

	// release the scan buffer
	if(buffer)
		free(buffer);

	if(err != ERR_NO_ERROR)
	{
		char errMsg[ERR_MSG_LEN];
		ulGetErrMsg(err, errMsg);
		printf("Error Code: %d \n", err);
		printf("Error Message: %s \n", errMsg);
	}
  }

  /*
  //
  // Initialize DAQ handler and start the poller
  //
  void start_daq(int chan_nums, double rate) { 
    
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
  */

  /*
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


  //
  // Polls DAQ at given sample rate
  //
  void* daq_poller(void*) { 
    double data = 0;
    float *adc_results_tmp = new float[channel_nums];

    while(true){
      for(int chan=0; chan<channel_nums; chan++) {
        err = ulAIn(handle, chan, AI_SINGLE_ENDED, BIP5VOLTS, AIN_FF_DEFAULT, &data);
        adc_results_tmp[chan] = data;
      }
      lock_adc();
      for(int chan=0; chan<channel_nums; chan++) {
        adc_results[chan] = adc_results_tmp[chan];

      }
      unlock_adc();
      usleep(sample_period);
    }
  }
  */

}
