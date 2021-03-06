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

DaqDeviceHandle daqDeviceHandle = 0;

DaqEventType eventTypes = (DaqEventType) (DE_ON_DATA_AVAILABLE | DE_ON_INPUT_SCAN_ERROR | DE_ON_END_OF_INPUT_SCAN);

const char* DAQ_HEADER = "DAQ1,DAQ2,DAQ3,DAQ4,DAQ5,DAQ6,DAQ7,DAQ8";

const char* DaqUav::get_header(){
  return DAQ_HEADER;
}

DaqUav::DaqUav(int lchan, int hchan, double rate) { 
  scan_data.lowChan = lchan;
  scan_data.highChan = hchan;
  scan_data.rate = rate;
  buffer = NULL;
  scan_data.index = 0;
}

void DaqUav::start_daq()
{
	int descriptorIndex = 0;
	DaqDeviceDescriptor devDescriptors[MAX_DEV_COUNT];
	DaqDeviceInterface interfaceType = ANY_IFC;
	unsigned int numDevs = MAX_DEV_COUNT;

	// set some variables that are used to acquire data
	int lowChan = scan_data.lowChan;
	int highChan = scan_data.highChan;
	AiInputMode inputMode;
	Range range;
	int samplesPerChannel = 1;
	double rate = scan_data.rate;
	AInScanFlag flags = AINSCAN_FF_DEFAULT;

	// set the scan options for a FINITE scan ... to set the scan options for
	// a continuous scan, uncomment the line that or's the SO_CONTINUOUS option
	// into to the scanOptions variable
	//
	// if this is changed to a CONTINUOUS scan, then changes will need to be made
	// to the event handler (eventCallbackFunction) to account for the buffer wrap
	// around condition
	ScanOption scanOptions = SO_DEFAULTIO;
	scanOptions = (ScanOption) (scanOptions | SO_CONTINUOUS);

	int hasAI = 0;
	int hasPacer = 0;
	int numberOfChannels = 0;
	int availableSampleCount = 0;

	char inputModeStr[MAX_STR_LENGTH];
	char rangeStr[MAX_STR_LENGTH];
	char scanOptionsStr[MAX_SCAN_OPTIONS_LENGTH];

	// allocate a buffer to receive the data
	int chanCount = 0;
	int bufferSize;
	UlError err = ERR_NO_ERROR;

	int i = 0;
	int __attribute__((unused)) ret;
	char c;

	// Get descriptors for all of the available DAQ devices
	err = ulGetDaqDeviceInventory(interfaceType, devDescriptors, &numDevs);

	if (err != ERR_NO_ERROR) {
		stop_daq();
  }

	// verify at least one DAQ device is detected
	if (numDevs == 0)
	{
		printf("No DAQ device is detected\n");
		stop_daq();
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
	}

	// verify the specified DAQ device supports analog input
	err = getDevInfoHasAi(daqDeviceHandle, &hasAI);
	if (!hasAI)
	{
		printf("\nThe specified DAQ device does not support analog input\n");
		stop_daq();
	}

	// verify the device supports hardware pacing for analog input
	err = getAiInfoHasPacer(daqDeviceHandle, &hasPacer);

	if (!hasPacer)
	{
		printf("The specified DAQ device does not support hardware paced analog input\n");
		stop_daq();
	}

	printf("\nConnecting to device %s - please wait ...\n", devDescriptors[descriptorIndex].devString);

	// establish a connection to the DAQ device
	err = ulConnectDaqDevice(daqDeviceHandle);

	if (err != ERR_NO_ERROR) {
		stop_daq();
  }

	// get the first supported analog input mode
	err = getAiInfoFirstSupportedInputMode(daqDeviceHandle, &numberOfChannels, &inputMode, inputModeStr);

	if (highChan >= numberOfChannels) {
		highChan = numberOfChannels - 1;
  }

	chanCount = highChan - lowChan + 1;

	// allocate a buffer to receive the data
	bufferSize = chanCount * samplesPerChannel;
	buffer = (double*) malloc(bufferSize * sizeof(double));
	for(i=0;i<bufferSize;i++){
		buffer[i] = 0; // pad with zeros
	}	

	if(buffer == NULL)
	{
		printf("\nOut of memory, unable to create scan buffer\n");
		stop_daq();
	}

	// store the scan event parameters for use in the callback function
	scan_data.buffer =  buffer;
	scan_data.bufferSize =  bufferSize;
	scan_data.lowChan = lowChan;
	scan_data.highChan = highChan;

	// enable the event to be notified every time 100 samples are available
	availableSampleCount = 100;
	err = ulEnableEvent(daqDeviceHandle, eventTypes, availableSampleCount, eventCallbackFunction, &scan_data);

	// get the first supported analog input range
	err = getAiInfoFirstSupportedRange(daqDeviceHandle, inputMode, &range, rangeStr);

	ConvertScanOptionsToString(scanOptions, scanOptionsStr);

	// start the finite acquisition
	err = ulAInScan(daqDeviceHandle, lowChan, highChan, inputMode, range, samplesPerChannel, &rate, scanOptions, flags, buffer);

	scan_data.rate = rate;

}

double* DaqUav::get_results(size_t& size) {
  // TODO: restructure the handler to allow for buffer data protection w/ mutex
  size = scan_data.bufferSize;
  return buffer+scan_data.index;
}

void DaqUav::stop_daq() {
	UlError err = ERR_NO_ERROR;
  err = ulAInScanStop(daqDeviceHandle);

	// disable events
	ulDisableEvent(daqDeviceHandle, eventTypes);

	// disconnect from the DAQ device
	ulDisconnectDaqDevice(daqDeviceHandle);

	// release the handle to the DAQ device
	if(daqDeviceHandle)
		ulReleaseDaqDevice(daqDeviceHandle);

	// release the scan buffer
	if(buffer) {
		free(buffer);
  }

	if(err != ERR_NO_ERROR)
	{
		char errMsg[ERR_MSG_LEN];
		ulGetErrMsg(err, errMsg);
		printf("Error Code: %d \n", err);
		printf("Error Message: %s \n", errMsg);
	}
}


/*
 * Event handler for when data is ready from the samples
 */
void DaqUav::eventCallbackFunction(DaqDeviceHandle daqDeviceHandle, DaqEventType eventType, unsigned long long eventData, void* userData)
{
	char eventTypeStr[MAX_STR_LENGTH];
	UlError err = ERR_NO_ERROR;
	DaqDeviceDescriptor activeDevDescriptor;
	int i;

	ScanEventParameters* scanEventParameters = (ScanEventParameters*) userData;
	int chanCount = scanEventParameters->highChan  - scanEventParameters->lowChan + 1;
	int newlineCount = chanCount + 7;

	if (eventType == DE_ON_DATA_AVAILABLE)
	{
		long long scanCount = eventData;
		long long totalSamples = scanCount * chanCount;

		// TODO: if this example is changed to a CONTINUOUS scan, then you will need
		// to maintain the index of where the data is being written to the buffer
		// to handle the buffer wrap around condition
		scanEventParameters->index = (totalSamples - chanCount) % scanEventParameters->bufferSize;
	}

	if(eventType == DE_ON_INPUT_SCAN_ERROR)
	{
		for (i = 0; i < newlineCount; i++)
			putchar('\n');

		err = (UlError) eventData;
		char errMsg[ERR_MSG_LEN];
		ulGetErrMsg(err, errMsg);
		printf("Error Code: %d \n", err);
		printf("Error Message: %s \n", errMsg);
	}


	if (eventType == DE_ON_END_OF_INPUT_SCAN)
	{
		for (i = 0; i < newlineCount; i++)
			putchar('\n');

		printf("\nThe scan using device %s (%s) is complete \n", activeDevDescriptor.productName, activeDevDescriptor.uniqueId);
	}
}
