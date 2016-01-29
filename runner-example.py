#!/usr/bin/env python

#@file runner.py

import os
import sys
import optparse
import subprocess
import random

# import python modules from $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
# the port used for communicating with your sumo instance
PORT = 8873

# designates the phases, one for each direction and turn type, this is for intersection 13
NSGREEN = "GGGgrrrrGGGrrrr"
NSYELLOW = "yyygrrrryyyrrrr"
TURN1 = "rrrGrrrrrrrrrrr"
CLEAR1 = "rrryrrrrrrrrrrr"
WEGREEN = "rrrrGGGgrrrGGGg"
WEYELLOW = "rrrryyygrrryyyg"
TURN2 = "rrrrrrrGrrrrrrG"
CLEAR2 = "rrrrrrryrrrrrry"

# An example of a potential program
PROGRAM = [NSGREEN, NSGREEN, NSGREEN, NSGREEN, NSGREEN, NSYELLOW, NSYELLOW, TURN1, CLEAR1, CLEAR1, WEGREEN, WEGREEN, WEGREEN,
	WEGREEN, WEGREEN, WEYELLOW, WEYELLOW, TURN2, CLEAR2, CLEAR2]

# Runs the simulation, and allows you to change traffic phase
def run():
    ## execute the TraCI control loop
    traci.init(PORT)
    programPointer = len(PROGRAM) - 1 # initiates at end
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep() # advance a simulation step. Give target time as value
        
        # sets next phase in the program cycle
        programPointer = min(programPointer + 1, len(PROGRAM) - 1) 
        
        # gets number of vehicles in the induction area in the last step, this is currently not being used
        # numPriorityVehicles = traci.inductionloop.getLastStepVehicleNumber("0")
        # can be used to simulate emergency vehicles, which would be given a special car type
        numPriorityVehicles = 1 
        # gets the value for the detector defined in huntcol.det.xml
        
        if numPriorityVehicles > 0: #if whatever value found above is true
            if programPointer == len(PROGRAM) - 1:
                # we are in the WEGREEN phase. start the priority phase
                # sequence to reset/allow train to pass
                programPointer = 0
            elif PROGRAM[programPointer] != WEYELLOW:
                # horizontal traffic is already stopped. restart priority phase
                # sequence at green
                programPointer = 3
            else:
                # we are in the WEYELLOW phase. continue sequence
                pass
                
        # sets traffic light at intersection 13 at the phase indicated
        #traci.trafficlights.setRedYellowGreenState("13", PROGRAM[programPointer])
        # this would set the traffic light to only ever allow one direction, for example
        #traci.trafficlights.setRedYellowGreenState("13", "GGGgrrrrGGGrrrr")
        step += 1
    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    sumoProcess = subprocess.Popen([sumoBinary, "-c", "data/huntcol.sumocfg", "--tripinfo-output",
                                    "tripinfo.xml", "--remote-port", str(PORT)], stdout=sys.stdout, stderr=sys.stderr)
    run()
    sumoProcess.wait()
