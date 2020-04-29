# Running 

Command to run with best results:

```sh
sudo nice --20 ./build/uav_launch master_config.yml
```

# Developing 

## To build

Top level CMake is in src, so the following will build the necessary libraries
```sh 
cd build
cmake ../src
make -j4
cd ../
```

## Software Layout Notes

* The sensors are designed to asynchronously update and log all together. Each sensor has an async file the holds the initialization and event methods for the sensor.

* Main under uav_launch handles the initialization calls and logging of each sensor.


# TODO 
* The async sensor files should become classes/objects with the event handler becoming a static member function
* The logger should become its own class and become more generic to the sensors/delays etc. 
* All async sensors should inherit from a generic sensor class with the following method expectiations
	* start_sensor
	* stop_sensor
	* sensor_event_handler
	* get_sensor_value



