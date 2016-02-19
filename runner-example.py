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
        __file__), '..', '..', '..', '..', "tools"))
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))
    from sumolib import checkBinary
except ImportError:
    sys.exit("please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci
PORT = 8873 # the port used for communicating with your sumo instance

# designates the phases, one for each direction and turn type, this is for intersection 13
NSGREEN = "GGGgrrrrGGGrrrr"
NSYELLOW = "yyygrrrryyyrrrr"
TURN1 = "rrrGrrrrrrrrrrr"
CLEAR1 = "rrryrrrrrrrrrrr"
WEGREEN = "rrrrGGGgrrrGGGg"
WEYELLOW = "rrrryyygrrryyyg"
TURN2 = "rrrrrrrGrrrrrrG"
CLEAR2 = "rrrrrrryrrrrrry"

# An example of a potential program, total length of 18, 3 seconds each
# NS pass goes during i=0-9 and WE pass goes during i=16-33
PROGRAM = [NSGREEN, NSGREEN, NSGREEN, NSGREEN, NSGREEN, NSGREEN, NSGREEN, NSGREEN, NSGREEN, NSGREEN,
	NSYELLOW, TURN1, TURN1, TURN1, TURN1, CLEAR1, 
	WEGREEN, WEGREEN, WEGREEN, WEGREEN, WEGREEN, WEGREEN, WEGREEN, WEGREEN, WEGREEN, WEGREEN, 
	WEGREEN, WEGREEN, WEGREEN, WEGREEN, WEGREEN, WEGREEN, WEGREEN, WEGREEN, 
	WEYELLOW, TURN2, TURN2, TURN2, CLEAR2]

# Runs the simulation, and allows you to change traffic phase
def run():
    ## execute the TraCI control loop
    traci.init(PORT)
    programPointer = 0 # initiates at start # len(PROGRAM) - 1 # initiates at end
    step = 0
    # Keeps track of current queue length in each direction
    queue_east = 0
    queue_north = 0
    queue_west = 0
    queue_south = 0
    # Flow counters, currently double counts cars
    flow_east = 0
    flow_north = 0
    flow_west = 0
    flow_south = 0
    # Keeps track of the last car through each sensor
    last_east1 = ""
    last_north1 = ""
    last_west1 = ""
    last_south1 = ""
    last_east2 = ""
    last_north2 = ""
    last_west2 = ""
    last_south2 = ""
    last_east3 = ""
    last_east_t1 = ""
    last_north_t1 = ""
    last_west_t1 = ""
    last_south_t1 = ""
    last_east_t2 = ""
    last_north_t2 = ""
    last_west_t2 = ""
    last_south_t2 = ""
    
    while traci.simulation.getMinExpectedNumber() > 0 and step <= 60*60: # one hour
        traci.simulationStep() # advance a simulation step
        
        # sets next phase in the program cycle
        if step % 3 == 0: # advances program pointer every three steps
        	programPointer = (programPointer + 1) % len(PROGRAM)
        	#print programPointer
        
        # gets number of vehicles in the induction area in the last step, this is currently not being used
        # numPriorityVehicles = traci.inductionloop.getLastStepVehicleNumber("south_inner1")
        
        ###################################### SOUTH ######################################
        structure1 = traci.inductionloop.getVehicleData("south_inner1")
        structure2 = traci.inductionloop.getVehicleData("south_inner2")
        structure3 = traci.inductionloop.getVehicleData("south_outer1")
        structure4 = traci.inductionloop.getVehicleData("south_outer2")
        structure5 = traci.inductionloop.getVehicleData("south_check1")
        structure6 = traci.inductionloop.getVehicleData("south_check2")

	    # Detecting a full queue using method 1
        if (structure3 and structure3[0][0] == last_south_t1 and structure3[0][3] == -1) or (structure5 and structure5[0][0] == last_south_t1 and structure5[0][3] == -1): # in case we detect the back is still
	        if (structure4 and structure4[0][0] == last_south_t2 and structure4[0][3] == -1) or (structure6 and structure6[0][0] == last_south_t2 and structure6[0][3] == -1):
		        if structure1 and structure2 and structure1[0][0] == last_south1 and structure2[0][0] == last_south2: # in case we detect the front is still
					# use getLastStepMeanSpeed instead?
					if (structure1[0][3] == -1) and (structure2[0][3] == -1): # all four cars are still
					#if queue_south > 24: # we are already almost full (one car didn't get detected), method 2
						#print "South Queue Full"
						queue_south = 26
						
        for car in (structure1):
	        if structure1 and car[0] != last_south1 and car[0] != last_south2:
	        	last_south1 = car[0]
	        	queue_south -= 1
	        	flow_south += 1
        for car in (structure2):
	        if structure2 and car[0] != last_south1 and car[0] != last_south2:
	        	last_south2 = car[0]
	        	queue_south -= 1
	        	flow_south += 1
        for car in (structure3):
	        if structure3 and car[0] != last_south_t1 and car[0] != last_south_t2:
	        	last_south_t1 = car[0]
	        	queue_south += 1
	        	flow_south += 1
        for car in (structure4):
	        if structure4 and car[0] != last_south_t1 and car[0] != last_south_t2:
	        	last_south_t2 = car[0]
	        	queue_south += 1
	        	flow_south += 1
	        	
        if queue_south < 0:
	        queue_south = 0
	    
        
        ###################################### WEST ######################################
        structure1 = traci.inductionloop.getVehicleData("west_inner1")
        structure2 = traci.inductionloop.getVehicleData("west_inner2")
        structure3 = traci.inductionloop.getVehicleData("west_outer1")
        structure4 = traci.inductionloop.getVehicleData("west_outer2")
        structure5 = traci.inductionloop.getVehicleData("west_check1")
        structure6 = traci.inductionloop.getVehicleData("west_check2")
        
	    # Detecting a full queue using method 1
        if (structure3 and structure3[0][0] == last_west_t1 and structure3[0][3] == -1) or (structure5 and structure5[0][0] == last_west_t1 and structure5[0][3] == -1): # in case we detect the back is still
	        if (structure4 and structure4[0][0] == last_west_t2 and structure4[0][3] == -1) or (structure6 and structure6[0][0] == last_west_t2 and structure6[0][3] == -1):
		        if structure1 and structure2 and structure1[0][0] == last_west1 and structure2[0][0] == last_west2: # in case we detect the front is still
					if (structure1[0][3] == -1) and (structure2[0][3] == -1): # all four cars are still
					#if queue_west > 24: # we are already almost full (one car didn't get detected), method 2
						#print "West Queue Full"
						queue_west = 26
					
        for car in (structure1):
            if structure1 and car[0] != last_west1 and car[0] != last_west2:
        	    last_west1 = car[0]
        	    queue_west -= 1
        	    flow_west += 1
        for car in (structure2):
	        if structure2 and car[0] != last_west1 and car[0] != last_west2:
	        	last_west2 = car[0]
	        	queue_west -= 1
	        	flow_west += 1
        for car in (structure3):
            if structure3 and car[0] != last_west_t1 and car[0] != last_west_t2:
	        	last_west_t1 = car[0]
	        	queue_west += 1
	        	flow_west += 1
        for car in (structure4):
	        if structure4 and car[0] != last_west_t1 and car[0] != last_west_t2:
	        	last_west_t2 = car[0]
	        	queue_west += 1
	        	flow_west += 1
	    
        if queue_west < 0:
	        queue_west = 0
        	
        ###################################### NORTH ######################################
        structure1 = traci.inductionloop.getVehicleData("north_inner1")
        structure2 = traci.inductionloop.getVehicleData("north_inner2")
        structure3 = traci.inductionloop.getVehicleData("north_outer1")
        structure4 = traci.inductionloop.getVehicleData("north_outer2")

        if structure1 and structure1[0][0] != last_north1:
        	last_north1 = structure1[0][0]
        	queue_north -= 1
        	flow_north += 1
        if structure2 and structure2[0][0] != last_north2:
        	last_north2 = structure2[0][0]
        	queue_north -= 1
        	flow_north += 1
        if structure3 and structure3[0][0] != last_north_t1:
        	last_north_t1 = structure3[0][0]
        	queue_north += 1
        	flow_north += 1
        if structure4 and structure4[0][0] != last_north_t2:
        	last_north_t2 = structure4[0][0]
        	queue_north += 1
        	flow_north += 1
        	
        if queue_north < 0:
	        queue_north = 0
        	
        ###################################### EAST ######################################
        structure1 = traci.inductionloop.getVehicleData("east_inner1")
        structure2 = traci.inductionloop.getVehicleData("east_inner2")
        structure3 = traci.inductionloop.getVehicleData("east_outer1")
        structure4 = traci.inductionloop.getVehicleData("east_outer2")
        structure5 = traci.inductionloop.getVehicleData("east_branch")

        for car in (structure1):
            if structure1 and car[0] != last_east1 and car[0] != last_east2:
                last_east1 = car[0]
                queue_east -= 1
                flow_east += 1
        for car in (structure2):
	        if structure2 and car[0] != last_east1 and car[0] != last_east2:
	        	last_east2 = car[0]
	        	queue_east -= 1
	        	flow_east += 1
        for car in (structure3):
	        if structure3 and car[0] != last_east_t1:
	        	last_east_t1 = car[0]
	        	queue_east += 1
	        	flow_east += 1
        for car in (structure4):
	        if structure4 and car[0] != last_east_t2:
	        	last_east_t2 = car[0]
	        	queue_east += 1
	        	flow_east += 1
        for car in (structure5):
	        if structure5 and [0] != last_east3:
	        	last_east3 = structure5[0][0] # branch
	        	queue_east -= 1
	        	flow_east += 1
	        	
        if queue_east < 0:
	        queue_east = 0
        
        ###################################### LIGHT CONTROL ######################################
        light_control = False
        
        if light_control:
	        if (queue_east + queue_west) < (queue_north + queue_south): # if the vertical pressure is higher
	            if programPointer == 9:
	            	#print "restarting NS"
	                # NS is currently ending, go back
	                programPointer = 7
	            #elif programPointer > 15:
	            #    # WE is currently active, skip to end of phase
	            #    programPointer = max(33, programPointer)
	        elif (queue_east + queue_west) > (queue_north + queue_south): # then horizontal pressure is higher
	            if programPointer == 33:
	            	#print "restarting WE"
	                # WE is currently ending, restart
	                programPointer = 31
	            #elif programPointer < 9:
	            #    # NS is currently active, skip to end of phase
	            #    programPointer = 9
       	
       	if step % 600 == 0:
 	      	print "----"
 	      	#print(str(flow_east) + " " + str(flow_west) + " " + str(flow_north) + " " + str(flow_south))
    	   	print (flow_east)
       		print (flow_west)
       		print (flow_north)
       		print (flow_south)
       		
        ################################# WEST PLATOONING #################################
	    # If the signal is about to open, create platoons
        platooning = False
        
        if platooning:
	        if (programPointer >= 15 and programPointer <= 16):
	            cars = traci.lane.getLastStepVehicleIDs("G5_0")
				# iterate through cars in order of closest to light
	            #print "-------------------------------------"+str(step)
	            for car in reversed(cars):
					#if traci.vehicle.getPosition(car): # potential check to add to see if car is past a certain point; not necessary
					#print traci.vehicle.getRoute(car)
					
					# check if the vehicle is automatic
	                type = traci.vehicle.getTypeID(car)
	                if type == "CarA":
						# If the car is automatic, add to platoon here
						#print ("Reducing minimum gap and increasing speed")
						#print traci.vehicle.getPosition(car)
						traci.vehicle.setMinGap(car, 0.1) # temporarily set its minimum gap, or other property
						traci.vehicle.setSpeed(car, 60) # set its speed
	                else:
						# if it is manual, stop making the platoon, since no cars behind can accelerate anyways
						break
                
        # sets traffic light at intersection 13 at the phase indicated
        traci.trafficlights.setRedYellowGreenState("13", PROGRAM[programPointer])
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

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    sumoProcess = subprocess.Popen([sumoBinary, "-c", "network/huntcol.sumocfg", "--step-length", "1.0", "--tripinfo-output",
                                    "tripinfo.xml", "--remote-port", str(PORT)], stdout=sys.stdout, stderr=sys.stderr)
    run()
    sumoProcess.wait()
