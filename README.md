# cav-sumo

### Work in progress

Example of using TRACI/SUMOLIB for overriding acceleration behavior of certain agents in SUMO microsimulation

<!-- Requires **I24scenario/** folder to be populated from other project <https://github.com/yanb514/CorridorCalibration> -->

Requires shared library **.so / .dll** files to be generated from another project <https://github.com/tylerard33/cav_codegen_examples> and needs the accompanying **cwrapper.py** and **cppwrapper.py** interfaces. To run, pull these files into the current working directory

## TODO Microsim
Add traffic light scenario - base it on Peachtree and Chicago connected corridor locations and timings

Add traffic light CAV eco-approach control interface

Add microsim Quality Control script

Add microsim result analysis script

Add mixed traffic scenarios (done through adjusting CAV penetration rate)

Add a scripted way to adjust the scenario CAV penetration rate (done)

## TODO XIL Testing
(Optional but nice to have) Add a scripted way to adjust the traffic light positions and timings

(Optional but nice to have) Add a scripted way to build the road network based on desired X-Y coordinates

Add a scripted way to inject vehicles that are externally-controlled apart from SUMO (such as a Roadrunner vehicle (SIL), a real vehicle driving on Dyno (DIL) or a real vehicle driving on a test track (VIL))

Add a ROS binding