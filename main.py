#!/usr/bin/env python3

#######################################
### Tyler Ard                       ###
### Yaozhong Zhang                  ###
### Vehicle Mobility Systems Group  ###
### tard(at)anl(dot)gov             ###
#######################################

### https://sumo.dlr.de/docs/Tutorials/Hello_SUMO.html

# Running microsimulations
### https://sumo.dlr.de/docs/sumo.html
### https://sumo.dlr.de/docs/TraCI.html
### https://sumo.dlr.de/docs/Libsumo.html

### https://sumo.dlr.de/docs/TraCI/Vehicle_Value_Retrieval.html
### https://sumo.dlr.de/docs/TraCI/Traffic_Lights_Value_Retrieval.html
### https://sumo.dlr.de/docs/Definition_of_Vehicles%2C_Vehicle_Types%2C_and_Routes.html

# Outputting data
### https://sumo.dlr.de/docs/Tools/Xml.html#xml2csvpy

# Creating networks
### https://sumo.dlr.de/docs/netconvert.html
### https://sumo.dlr.de/docs/Netedit/index.html

import numpy as np
import time
import os
import argparse
from math import fmod

from sumolib import checkBinary
# import traci # traci has full API options
import libsumo as traci # libsumo is faster but no GUI support and select API options are limited

from _settings import *
from _agents import PCC

# -------------------------------------------------------------------------------------------------------

class sumoAPI():
    def __init__(self):
        ### Initialize microsimulation
        # Handle input arguments
        parser = argparse.ArgumentParser('Python SUMO Simulation')
        
        parser.add_argument('--gui',
                            help='Flag to launch GUI of SUMO. Default false.',
                            default=False, action='store_true')
        
        parser.add_argument('--realtime',
                            help='Flag to run real-time SUMO simulation. Default false so as to compute sim as fast as possible.', 
                            default=False, action='store_true')

        args = parser.parse_args()

        # Check command line arguments
        if args.realtime:
            self.is_realtime = True # Simulation should run in realtime

        else:
            self.is_realtime = False # Simulation shoud execute as fast as possible

        # Timer parameters
        self.dt = 0.50 # [s] time between each simulation frame - needed for realtime option

        self.rt_tic = time.time() # [unix] Realtime tic timer keeping track of time when last frame was executed

        ### Set up SUMO
        # Check library imports
        if traci.isLibsumo():
            print('Using lib sumo.')

        # SUMO scenario input and output settings
        SUMO_OUT_DIR = "data"
        if not os.path.exists(SUMO_OUT_DIR): # Make the data directory if it does not exist
            os.makedirs(SUMO_OUT_DIR)
        
        SUMO_CFG_FILE = "sumo_scenarios/I24/I24_scenario.sumocfg"
        SUMO_OUT_FILE = os.path.join(SUMO_OUT_DIR, "fcd.xml")

        if args.gui: # GUI is requested and traci is imported - libsumo does not support GUI
            SUMO_BINARY = checkBinary('sumo-gui')
        
        else:
            SUMO_BINARY = checkBinary('sumo')

        # Set up SUMO start command
        SUMO_CMD = [
            SUMO_BINARY, 
            '-c', SUMO_CFG_FILE,
            
            # Additional commands that override the .sumocfg
            '--fcd-output', SUMO_OUT_FILE, # Log floating car data every simulation step
            '--fcd-output.acceleration', # Add acceleration to output

            '--step-log.period', '50', # Every n simulation steps TRACI output to command line
            '--step-length', str(self.dt) # [s] Time interval between simulation steps
        ]

        print(f'Starting SUMO with configuration file: {SUMO_CFG_FILE}.')
        print(f'Outputting additional SUMO data to file: {SUMO_OUT_FILE}.')

        ### Initialize AV controller 
        self.ego = PCC() # Predictive cruise controller

        ### Start SUMO using traci/libsumo
        traci.start(SUMO_CMD)

    def __del__(self):
        print('Closing SUMO.')
        
        traci.close()

    def step(self):
        '''Run simulation step of currently connected SUMO network'''
        ## Run simulation step
        traci.simulationStep()

        ## Process microsimulation data
        # Get simulation status
        sim_time = traci.simulation.getTime()

        # Get vehicle data
        vehicle_ids = traci.vehicle.getIDList()

        type_gen = (ego_id for ego_id in vehicle_ids if traci.vehicle.getTypeID(ego_id) == 'trial') # Generator to get vehicles that match a given vType string
        for ego_id in type_gen:
            ## Get ego states
            ego_distance = traci.vehicle.getDistance(ego_id) # From starting point - TODO behavior if multiple starting points?
            ego_speed = traci.vehicle.getSpeed(ego_id)
            ego_accel = traci.vehicle.getAcceleration(ego_id)

            ego_min_gap = traci.vehicle.getMinGap(ego_id)
            
            # Get preceding vehicle states
            leader = traci.vehicle.getLeader(ego_id)
            if leader is not None and leader[0] != "":
                lead_id = leader[0] # string of "flow id.veh id"
                
                lead_rel_distance = leader[1] # (Front bumper + min_gap) to rear bumper
                lead_speed = traci.vehicle.getSpeed(lead_id)
                lead_accel = traci.vehicle.getAcceleration(lead_id)

            else: # No leader, so choose default sensor values
                lead_id = -1 # Default values
            
                lead_rel_distance = 2000. # Bumper to bumper gap [m]
                lead_speed = 0. # [m/s]
                lead_accel = 0. # [m/s2]

            # Get traffic intersection states - TODO
            TLs = traci.vehicle.getNextTLS(ego_id) # Return list of upcoming traffic lights [(tlsID, tlsIndex, distance, state), ...]

            ## Call compiled controller
            desired_acceleration = self.ego.getCommand(sim_time, ego_accel, ego_speed, ego_distance, lead_accel, lead_speed, lead_rel_distance)
            
            # Set ego acceleration
            num_steps = 1 # How many simulation steps the command is applied for
            traci.vehicle.setAcceleration(ego_id, desired_acceleration, num_steps)

            # Set ego color
            color = (80, 255, 80, 0) # RGBA: 0-255 RGB scale, 0-1 Alpha scale
            traci.vehicle.setColor(ego_id, color)

        # Rate limit if using the --realtime command line argument
        self.rate()

        ### Print output
        if fmod(sim_time, 1) == 0: # Print output every n seconds of simulation time
            print('t: {:0.1f}'.format(sim_time))

    def rate(self):
        '''Sleep program until next time to execute another frame if running with real-time option'''
        if self.is_realtime:
            while time.time()-self.rt_tic < self.dt:
                pass

            self.rt_tic = time.time()

    def sim(self):
        try:
            while traci.simulation.getMinExpectedNumber() > 0: # While there are active vehicles/pedestrians in network
                self.step()

        except KeyboardInterrupt:
            pass

# -------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    # Run simulation
    sumo = sumoAPI()
    sumo.sim()