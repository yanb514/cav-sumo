#! /usr/bin/env python3
################################
### Tyler Ard                ###
### Argonne National Lab     ###
### Vehicle Mobility Systems ###
### tard(at)anl(dot)gov      ###
################################

from math import sin, cos, sqrt, tan, atan, atan2, fmod, pi, inf, nan

import os
import numpy as np

from _settings import *
from _cppwrapper import cpp_api
# from _cwrapper import c_api

class _vehicle:
    '''Virtual vehicle super class'''
    used_ids = []

    def __init__(self, t0 = -1., vd = 20., id = -1):
        ### Default controller properties
        self.vd = vd # Desired speed [m/s]
        self.d = 3.6 # Desired standstill gap distance [m]
        self.T = 1.2 # Desired time headway [s]

        # Default control actions
        self.ua = 0. # Desired acceleration [m/s2]
        self.ul = 0 # Desired lane change direction - left is >0 right is <0 [-]

        # Unique ID - If an ID is assigned (positive-valued) then it must be unique as checked against used_ids list
        self.id = id

        if id > 0: # ID is being assigned for this particular vehicle object - check that it is unique
            assert id not in _vehicle.used_ids, 'Non-unique ID attempted to be assigned for a specific vehicle object! ID {:d} already used.'.format(id)
            _vehicle.used_ids.append(id)

        # Intersection behavior
        self.is_stopping_light = False # Flag to indicate if ego is braking to stopping at intersection - initialize as False in constructor

        self.stop_dwell_time = 3. # Time to wait at a stop sign before continuing
        self.dt_stop = 0. # Time before moving through a stop sign - must be <0 before moving through is allowed
        self.is_stopping_sign = False # Flag to indicate if ego is braking to stopping at stop sign - initialize as False in constructor
        self.is_waiting_sign = False # Flag to indicate if ego is waiting at stop sign - initialize as False in constructor
        self.is_moving_sign = False # Flag to indiciate if ego is moving through stop sign - initialize as False in constructor

        self.is_acc_after_dest = 1 # If ego should stop at stop sign and then travel through, or wait permanently

        # Simulation properties
        self.entering_time = t0 # Time when the vehicle enters the simulation [s]
        
        self.is_exists = False # Flag to detect if the vehicle has entered the simulation or not yet
        self.is_exited = False # Flag to detect if the vehicle has safely exited the network

class Basic(_vehicle):
    '''Basic adaptive cruise control driver for virtual automated vehicles'''    
    def __init__(self, t0 = -1., vd = 20., id = -1, active=False):
        super().__init__(t0, vd, id)

        self.type == VEHTYPE.BASIC

        # Parameters
        self.vd = vd # Desired velocity [m/s]
        self.active = active # True/false to allow lane changes to activate
        
        # Controls
        self.ua = 0. # Acceleration command
        self.ul = 0 # Lane command - which lane change direction to make

    def getCommand(ego_speed, ego_accel, lead_distance, lead_speed, lead_accel):
        desired_time_gap = 1.5
        min_gap = 2.0
        max_accel = 2.0
        max_decel = 4.5

        desired_gap = max(min_gap, ego_speed*desired_time_gap)
        gap_diff = lead_distance - desired_gap
        relative_speed = ego_speed - lead_speed

        if gap_diff > 0:
            desired_acceleration = min(max_accel, gap_diff - relative_speed)

        else:
            desired_acceleration = max(max_decel, gap_diff - relative_speed)

        return desired_acceleration

class PCC(_vehicle):
    '''PCC driver for virtual automated vehicles'''
    def __init__(self, t0 = -1., vd = 20., id = -1, active=False):
        super().__init__(t0, vd, id)

        self.type = VEHTYPE.PCC

        # Parameters
        self.vd = vd # Desired velocity [m/s]
        self.active = active # True/false to allow lane changes to activate
        
        # Controls
        self.ua = 0. # Acceleration command
        self.ul = 0 # Lane change command

        # Autogen Simulink Model Interface
        if os.name == 'nt':
            libraryname = 'pcc_so'
        
        elif os.name == 'posix':
            libraryname = 'libpcc_so'
        
        elif os.name == 'darwin':
            libraryname == 'libpcc_so'

        else:
            raise ValueError('PCC class cannot determine system type!')
        
        self.api = cpp_api(libraryname)

    def predAcc(self, t, lead_accel, lead_speed, lead_rel_distance, ego_distance, v_max):
        '''Run a constant acceleration prediction from the current measurements of the preceding vehicle'''
        # Current measurements
        dt_pred = 0.1 # Prediction stage step size
        t_pred = t

        k = 0 # Current PV state measurements
        self.api.inputs_p.contents.acc_pred[k] = lead_accel
        self.api.inputs_p.contents.vel_pred[k] = lead_speed
        self.api.inputs_p.contents.pos_pred[k] = ego_distance + lead_rel_distance
        self.api.inputs_p.contents.time_pred[k] = t

        for k in range(1,51): # Predicted PV states
            if lead_speed > v_max:
                lead_speed = v_max
                lead_accel = 0.

            elif lead_speed < 0:
                lead_accel = 0.
                lead_speed = 0.

            # Propagate kinematic constant acceleration prediction
            t_pred += dt_pred

            self.api.inputs_p.contents.acc_pred[k] = lead_accel
            self.api.inputs_p.contents.vel_pred[k] = max(lead_speed + dt_pred*lead_accel, 0)
            self.api.inputs_p.contents.pos_pred[k] = ego_distance + lead_rel_distance + dt_pred*(max(lead_speed + dt_pred*lead_accel, 0) + dt_pred*lead_accel)
            self.api.inputs_p.contents.time_pred[k] = t_pred

            lead_rel_distance = self.api.inputs_p.contents.pos_pred[k]
            lead_speed = self.api.inputs_p.contents.vel_pred[k]

    def getCommand(self, t, ego_accel, ego_speed, ego_distance, lead_accel, lead_speed, lead_rel_distance):
        ### Control constraint settings
        s_max = 5000
        v_max = 20 # put speed limit here

        ### Set the input signals
        self.api.inputs_p.contents.t = t

        self.api.inputs_p.contents.ego_state[0] = ego_distance  # Frenet front bumper position
        self.api.inputs_p.contents.ego_state[1] = ego_speed  # Frenet forward velocity
        self.api.inputs_p.contents.ego_state[2] = ego_accel  # Frenet forward acceleration

        # Run a prediction of the preceding vehicle motion - constant acceleration assumption here
        self.predAcc(t, lead_accel, lead_speed, lead_rel_distance, ego_distance, v_max)

        # Set constraints
        self.api.inputs_p.contents.pos_max = s_max
        self.api.inputs_p.contents.vel_max = v_max

        ### Run the controller
        self.api.step_inputs()
        self.api.step_controller()
        self.api.step_outputs()

        ### Assign outputs struct properties
        acc_des = self.api.outputs_p.contents.acc_des

        state_trajectory = self.api.outputs_p.contents.state_trajectory
        control_trajectory = self.api.outputs_p.contents.control_trajectory
        time_trajectory = self.api.outputs_p.contents.time_trajectory
        slacks = self.api.outputs_p.contents.slacks
        reference = self.api.outputs_p.contents.reference
        constraint = self.api.outputs_p.contents.constraint
        cost = self.api.outputs_p.contents.cost
        exitflag = self.api.outputs_p.contents.exitflag

        # Assign the control
        self.ua = acc_des
        
        return acc_des

class CAV(_vehicle):
    '''CAV driver for virtual automated vehicles'''
    def __init__(self, t0 = -1., vd = 20., id = -1, active=False):
        super().__init__(t0, vd, id)

        ### TODO
        print('CAVs for traffic light approach to be implemented')
        pass

        self.type = VEHTYPE.CAV

        # Parameters
        self.vd = vd # Desired velocity [m/s]
        self.active = active # True/false to allow lane changes to activate
        
        # Controls
        self.ua = 0. # Acceleration command
        self.ul = 0 # Lane change command

        self.acc_des, self.vel_des, self.pos_des = 0., 0., 0.

        # Additional Controls
            # The flags are:
            # -4: constant braking for stops (at very low speeds)
            # -1: Stops (no LT traj)
            # -2: Stops (no LT traj)
            # 1: LT free-flow
            # 2: ST car-following
            # 3: constant braking for stops
            # Others: stops

        self.flag = 0

        self.time_LTtraj = [None]*1000
        self.acc_LTtraj = [None]*1000
        self.vel_LTtraj = [None]*1000
        self.pos_LTtraj = [None]*1000
        
        self.time_STtraj = [None]*100
        self.acc_STtraj = [None]*100
        self.vel_STtraj = [None]*100
        self.pos_STtraj = [None]*100

        # Autogen Simulink Model Interface
        if os.name == 'nt':
            libraryname = 'cav_so'
        
        elif os.name == 'posix':
            libraryname = 'libcav_so'
        
        elif os.name == 'darwin':
            libraryname == 'libcav_so'

        else:
            raise ValueError('CAV class cannot determine system type!')
        
        self.api = c_api(libraryname)

    def getCommand(self, t, ego_accel, ego_speed, ego_distance, lead_accel, lead_speed, lead_rel_distance):
        '''Set the control commands, for example desired acceleration and desired lane'''

        # Run control for connected vehicle acceleration
        smax = self.s+1e4
        vmax = self.vd

        ### Assign inputs vectors
        # Intersections
        max_n_tls = 5
        IntscInfo = [0]*max_n_tls*6

        if len(tls)>max_n_tls:
            tls = getnLatestTLs(max_n_tls, self.s, self.len, tls)

        for k in range(0, max_n_tls):
            IntscInfo[0 + k*6] = smax
            IntscInfo[1 + k*6] = LIGHTTYPE.LIGHT
            IntscInfo[2 + k*6] = 90
            IntscInfo[3 + k*6] = 80
            IntscInfo[4 + k*6] = 5
            IntscInfo[5 + k*6] = 5

        for k in range(0, len(tls)):
            if tls[k].type == LIGHTTYPE.STOP:
                i = max_n_tls-1 # If a stop sign we are forcing to write to the last column of IntscInfo
            else:
                i = k

            IntscInfo[0 + i*6] = tls[k].s
            IntscInfo[1 + i*6] = tls[k].type
            IntscInfo[2 + i*6] = tls[k].cycle
            IntscInfo[3 + i*6] = tls[k].green
            IntscInfo[4 + i*6] = tls[k].amber
            IntscInfo[5 + i*6] = tls[k].phase

        IntscInfo[0 + (max_n_tls-1)*6] -= 0.4 # Move stopping distance backwards
        IntscInfo[1 + (max_n_tls-1)*6] = LIGHTTYPE.STOP # Force last to be a stop

        assert len(tls) <= max_n_tls, 'Max number of requested traffic lights not supported by CAVs!'

        # Speed limit
        SpdLimInfo = [0]*4

        SpdLimInfo[0] = 0
        SpdLimInfo[1] = vmax
        SpdLimInfo[2] = smax
        SpdLimInfo[3] = vmax
        
        # Control inputs
        CtrlInfo = [0]*14

        CtrlInfo[0] = t
        
        CtrlInfo[4] = self.s+self.len # Front bumper position - s is rear bumper position -> add length to get front bumper
        CtrlInfo[5] = self.v
        CtrlInfo[6] = self.a
        CtrlInfo[1:4] = CtrlInfo[4:7]

        ds, vr, kv = getFrontVeh(svs, self.x, self.y, self.len, self.theta, self.lane)
        if kv >= 0:
            CtrlInfo[7] = ds # sps - ss - d_min0;
            CtrlInfo[8] = vr
            CtrlInfo[9] = svs[kv].a

        else:
            CtrlInfo[7] = smax # sps - ss - d_min0;
            CtrlInfo[8] = 0.
            CtrlInfo[9] = 0.
        
        lightds, tlstatus, tlind = getFrontTL(self.s, self.len, tls)
        lighttype = tls[tlind].type
        if tlind >= 0:
            CtrlInfo[10] = tls[tlind].s # IntscPos
            CtrlInfo[11] = tls[tlind].type # IntscType
            CtrlInfo[12] = tls[tlind].status # TrfLghtState

        else:
            CtrlInfo[10] = smax # IntscPos
            CtrlInfo[11] = LIGHTTYPE.STOP # IntscType
            CtrlInfo[12] = LIGHTSTATUS.RED # TrfLghtState

        CtrlInfo[13] = vmax # current maximum speed limit
        
        # CAV controller parameter setup
        CtrlPar = [0]*15

        CtrlPar[0] = CONN_RANGE # ConnRange, m
        CtrlPar[1] = 10 # NxtIntscCntDstMrgn, m
        CtrlPar[2] = 200 # VrtPtDstMrgn, m
        CtrlPar[3] = 1 # dtGrnIniTimeMrgn, s
        CtrlPar[4] = 2 # dtGrnEndTimeMrgn, s
        CtrlPar[5] = vmax*0.95 # SpdDes, m/s
        CtrlPar[6] = 1 # dtMin, s
        CtrlPar[7] = 1 # dtMax, s
        CtrlPar[8] = 1 # CFopt, 0 or 1
        CtrlPar[9] = 3 # tHrznCF, s
        CtrlPar[10] = 1.2 # tau_d, s
        CtrlPar[11] = 3.2 # d_min, m
        CtrlPar[12] = 1.5 # aDes, m/s^2
        CtrlPar[13] = 1.5 # bDes, m/s^2
        CtrlPar[14] = self.is_acc_after_dest # IsAccAfterDes, 0 or 1
        
        # Stop sign behavior
        self.dt_stop -= dt

        if (lighttype == LIGHTTYPE.STOP and lightds < ds and self.dt_stop < -3.*self.stop_dwell_time): # Stop sign is a new object ego is approaching and we haven't stopped recently for a sign
            self.is_stopping_sign = True
            self.is_going_sign = False

        if self.is_waiting_sign and self.dt_stop < 0 and self.is_acc_after_dest: # Finished stopping and dwelling for this sign
            self.is_stopping_sign = False
            self.is_waiting_sign = False
            self.is_moving_sign = True

        if self.is_stopping_sign and self.v < 0.1:
            if not self.is_waiting_sign: # Have not detected a stop for this sign yet
                self.is_waiting_sign = True # Detected the ego has now stopped
                self.dt_stop = dt + self.stop_dwell_time # Start the counter

        if self.is_waiting_sign:
            StopInfo = [-1, 3] # % stop mode (Stopped -1, Nearby 0, Moving 1) AND waiting time
        else:
            StopInfo = [1, 3] # % stop mode (Stopped -1, Nearby 0, Moving 1) AND waiting time

        if self.is_moving_sign:
            CtrlInfo[10] = nan
            CtrlInfo[11] = nan
            CtrlInfo[12] = LIGHTSTATUS.GREEN

        # Record the current gaps
        self.dgap = ds
        if tlstatus == LIGHTSTATUS.RED:
            self.tlgap = lightds

        ### Run the autogen controller
        self.acc_des, self.vel_des, self.pos_des, self.flag, self.time_LTtraj, self.acc_LTtraj, self.vel_LTtraj, self.pos_LTtraj, self.time_STtraj, self.acc_STtraj, self.vel_STtraj, self.pos_STtraj = self.api.step_controller(IntscInfo, SpdLimInfo, CtrlInfo, CtrlPar, StopInfo)
        
        # Assign the control
        self.ua = self.acc_des

class EXT(_vehicle):
    '''CAV driver placeholder for externally controlled vehicles placed into microsimulation'''
    def __init__(self, x=0., y=0., s=0., v=0., a=0., theta=0., lane=1):
        super().__init__(x, y, s, v, a, theta, lane, 0., 1, [], [])

        self.type = VEHTYPE.EXT

        # Controls
        self.ua = 0. # Acceleration command
        self.ul = 0 # Lane change command

        # Simulation properties
        self.is_exists = True
        self.is_exited = False

    def getCommand(self, svs, tls, t, dt):
        pass # EXT Wont have any command to compute since it is an external

class SAS(CAV):
    '''Eco Speed Advisory driver placeholder for externally controlled vehicles placed into microsimulation'''
    def __init__(self, x=0., y=0., s=0., v=0., a=0., theta=0., lane=1, entering_time = 0., ad = [], bd = [], thd = [], numLanes=1, obs=[], divs=[], vd = 10., active=False):
        super().__init__(x, y, s, v, a, theta, lane, entering_time, numLanes)

        self.type = VEHTYPE.SAS

        # Simulation properties
        self.is_exists = True
        self.is_exited = False

    def getCommand(self, svs, tls, t, dt):
        super().getCommand(svs, tls, t, dt)
        
class TrafficLight:
    '''Traffic light fixed signal phase and timing for virtual intersections'''
    id = 1

    def __init__(self, s, x, y, theta, type, cycle, green, amber, phase):
        # Light position
        self.s = s 
        self.x = x
        self.y = y
        self.theta = theta

        # Light timings and current status
        self.status = LIGHTSTATUS.RED
        self.type = type
        self.cycle = cycle
        self.green = green
        self.amber = amber
        self.phase = phase

        # Advanced light timings
        self.time_to_red = -1. # [s] Positive time if in amber phase and countdowns til the light changes to red
        self.time_to_green = -1. # [s] Positive time if in red phase and countdowns til the light changes to green

        # Unique light identifier
        self.id = TrafficLight.id
        TrafficLight.id += 1

    def setCommand(self, t):
        '''Get traffic intersection status'''
        lt = fmod(t + self.phase, self.cycle) # Light timing for how far into the green-amber-red cycle we are

        if self.type == LIGHTTYPE.LIGHT:
            if lt < self.green:
                self.status = LIGHTSTATUS.GREEN
            elif lt < self.green + self.amber:
                self.status = LIGHTSTATUS.AMBER
            else:
                self.status = LIGHTSTATUS.RED

        elif self.type == LIGHTTYPE.STOP:
            self.status = LIGHTSTATUS.RED

        else:
            raise RuntimeError('Unknown light type')
        
        # Set the time_to_ vars
        if self.type == LIGHTTYPE.STOP:
            self.time_to_red = -1.
            self.time_to_green = -1.

        elif self.type == LIGHTTYPE.LIGHT:
            if self.status == LIGHTSTATUS.AMBER:
                self.time_to_green = -1.
                self.time_to_red = max(self.green + self.amber - lt, 0)

            elif self.status == LIGHTSTATUS.RED:
                self.time_to_green = max(self.cycle - lt, 0)
                self.time_to_red = -1.
            
            else:
                self.time_to_green = -1.
                self.time_to_red = -1.

        else:
            raise RuntimeError('Unknown light type')
