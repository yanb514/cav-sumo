#! /usr/bin/env python3
################################
### Tyler Ard                ###
### Argonne National Lab     ###
### Vehicle Mobility Systems ###
### tard(at)anl(dot)gov      ###
################################

### Enums type
def enum(**enums):
    return type('Enum', (), enums)

# Vehicle and intersection types and statuses
LIGHTSTATUS = enum(GREEN=2, AMBER=1, RED=0) # Roadrunner enums for light status
LIGHTTYPE = enum(LIGHT=10, STOP=20) # Roadrunner enums for intersection type in RD matrix - traffic light or stop sign

VEHSTATUS = enum(NONE=0, FREE=1, LOWUTIL=2, TLINTAC=3) # Enums for MOBIL status on its current driving pattern
VEHTYPE = enum(SAS=-3, EXT=-2, NONE=-1, MOBIL=0, CAV=1, PCC=2, BASIC=3) # Enums for type of vehicle as assigned in SIM matrix

### Settings
# Simulation settings
# RANDSEED = 44 # Random seed used to fix the pseudo random generation. None to use system time

# TEND = 350.0 # time til simulation ends automatically [s]

# Vehicle parameter settings
VEHWIDTH = 1.90 # assumed vehicle width [m]
VEHLENGTH = 4.20 # assumed vehicle length [m]

# Road parameter settings
LANEWIDTH = 3.70 # width of lanes [m]
LANEWIDTH_INV = 1/LANEWIDTH # Inverse of lane width - precalculated to avoid division in loops

# Connectivity settings
CONN_RANGE = 450 # [m] Reliable connectivity range based on Chicago testing
