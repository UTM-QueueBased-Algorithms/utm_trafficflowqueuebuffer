# main.py
#
#  Queue UTM package drone simulation code
import sys, os
import csv
import math
#
import drone                #main drone class - vehicle kinematics and control
import tbov                 #main tbov class - airspace constraints and route network
import customer_time        #handles customer requests
import data_queue           #stores useful customer/drone data
import network_schedular    #handles network schedular algorithms

#------------------------------------------------
dir_sim_input   = "./sim_input/"
dir_sim_output  = "./sim_output/"


# read conf.txt input file ----------------------
f_log = open(dir_sim_output+"log.txt", "w+") #capture everything
try:
    f_conf = open(dir_sim_input+"conf.txt","r")
    reader = csv.reader(f_conf)
    #read data
    f_distance_matrix       = dir_sim_input + reader.__next__()[1].replace(" ","") #remove whitespace
    f_log.write("distance matrix file, %s\n"%f_distance_matrix)
    print("distance matrix file = ",f_distance_matrix)
    reader.__next__()
    #------------------------
    #
    nominal_gnd_spd         = float( reader.__next__()[1] )
    f_log.write("drone nominal gnd spd (km/hr), %f\n"%nominal_gnd_spd)
    print("ideal drone gnd spd (km/hr) = ",nominal_gnd_spd)
    #convert to (m/sec) for use in sim
    nominal_gnd_spd_m_sec = nominal_gnd_spd * 1000/(60*60)
    #
    drone_setup_time        = float( reader.__next__()[1] )
    f_log.write("drone setup time delay (sec), %f\n"%drone_setup_time)
    print("ideal drone setup time (sec) = ",drone_setup_time)
    #
    poisson_client_arr_rate = float( reader.__next__()[1] )
    f_log.write("client arr rate (orders/hr), %f\n"%poisson_client_arr_rate)
    print("customer request parameter (orders/hr) = ",poisson_client_arr_rate)
    #convert to (orders/sec) for use in sim
    poisson_client_arr_rate = poisson_client_arr_rate / (60*60)
    #
    waypoint_clearance      = int( reader.__next__()[1] )
    drone.Drone.WAYPOINT_MAX_DISTANCE = waypoint_clearance
    f_log.write("reached waypoint distance (meter), %d\n"%waypoint_clearance)
    reader.__next__()
    #------------------------
    #
    Ts                      = float( reader.__next__()[1] )
    f_log.write("time step (sec), %f\n"%Ts)
    print("Time step (sec) = ",Ts)
    #
    tot_icnt                = int( reader.__next__()[1] )
    f_log.write("total time steps (count), %d\n"%tot_icnt)
    print("Total number of time steps = ",tot_icnt)
    #
    print_rate              = int( reader.__next__()[1] )
    f_log.write("print output every (count), %d\n"%print_rate)
    print("Output print rate = ",print_rate)
    reader.__next__()
    #------------------------
    #
    drones_per_depot        = int( reader.__next__()[1] )  #ideal, might change later
    f_log.write("ideal drone amount per depot (count), %d\n"%drones_per_depot)
    print("ideal drone amount per depot = ",drones_per_depot)
    #
    drone_ideal_width       = int( reader.__next__()[1] )
    f_log.write("ideal drone width (meter), %d\n"%drone_ideal_width)
    print("ideal drone width (meter) = ",drone_ideal_width)
    #
    drone_ideal_length     = int( reader.__next__()[1] )
    f_log.write("ideal drone length (meter), %d\n"%drone_ideal_length)
    print("ideal drone length (meter) = ",drone_ideal_length)
    reader.__next__()
    #------------------------
    #
    TBOV_width              = int( reader.__next__()[1] )
    f_log.write("TBOV width from drone center (meter), %d\n"%TBOV_width)
    print("TBOV width from drone center (meter) = ",TBOV_width)
    #
    TBOV_length             = int( reader.__next__()[1] )
    f_log.write("TBOV length from drone center (meter), %d\n"%TBOV_length)
    print("TBOV length from drone center (meter) = ",TBOV_length)
    #
    TBOV_Ts                 = float( reader.__next__()[1] )
    f_log.write("TBOV update rate (sec), %f\n"%TBOV_Ts)
    print("TBOV update rate (sec) = ",TBOV_Ts)
    #
    depot_TBOV_clearance    = int( reader.__next__()[1] )
    #drone.TBOV.IGNORE_DEPOT_DISTANCE = depot_TBOV_clearance
    f_log.write("depot TBOV ignore distance (meter), %d\n"%depot_TBOV_clearance)
    print("depot TBOV ignore distance (meter) = ",depot_TBOV_clearance)
    reader.__next__()
    #------------------------
    #
    flag_const_flow         = ( int(reader.__next__()[1]) == 1 )
    f_log.write("max flow on (True=1/False=0), %s\n"%flag_const_flow)
    print("max flow on (True=1/False=0) = ",flag_const_flow)
    #
    flag_flow_control       = ( int(reader.__next__()[1]) == 1 )
    f_log.write("flow control on (True=1/False=0), %s\n"%flag_flow_control)
    print("flow control on (True=1/False=0) = ",flag_flow_control)
    #
    seed                    = int( reader.__next__()[1] )
    f_log.write("seed, %d\n"%seed)
    print("seed = ",seed)
    reader.__next__()
    #------------------------
    #
    depot_input_model       = reader.__next__()[1].replace(" ","").replace("\n","")
    f_log.write("depot_input_model (FCFS Priority ...), %s\n"%depot_input_model)
    print("depot_input_model = ",depot_input_model)
    #
    depot_output_model       = reader.__next__()[1].replace(" ","").replace("\n","")
    f_log.write("depot_output_model (FCFS Round-Robin ...), %s\n"%depot_output_model)
    print("depot_output_model = ",depot_output_model)

except:
    e = sys.exc_info()[0]
    print("ERROR: %s" %e )
    sys.exit(1)
f_conf.close()

#various initializations ------------------------
#--- setup graph / mission planner
graphFile = "sim_input/graph_matrix.csv"
testGraph = drone.DroneMissionPlanner()
assert( testGraph.loadGraph(graphFile) )
#
pathsFile = "sim_input/paths_s.txt"
assert( testGraph.loadPaths(pathsFile) )
#
depotLocFile = "sim_input/depot_loc.txt"
assert( testGraph.loadDepots(depotLocFile) )
f_log.write("Number of Depots,%d\n" %testGraph.num_dept)
print("Number of Depots = ",testGraph.num_dept)
custLocFile = "sim_input/customer_loc.txt"
assert( testGraph.loadCustomers(custLocFile) )
f_log.write("Number of Customers,%d\n" %testGraph.num_cust)
print("Number of Customers = ",testGraph.num_cust)
#--- end setup graph / mission planner

#--- setup locations
# Depot locations data
xdata_dept_list = []
ydata_dept_list = []
for idata in testGraph.Dept_Loc:
    xdata_dept_list.append( idata[1] )
    ydata_dept_list.append( idata[2] )
# Customer locations data
xdata_cust_list = []
ydata_cust_list = []
for idata in testGraph.Cust_Loc:
    xdata_cust_list.append( idata[1] )
    ydata_cust_list.append( idata[2] )
# Buffer locations data
xdata_buff_list = []
ydata_buff_list = []
for idata in testGraph.Cros_Loc:
    xdata_buff_list.append( 1000*idata[0] )
    ydata_buff_list.append( 1000*idata[1] )
#--- end setup locations

#--- setup customer data input
if(not flag_const_flow):
    cust = customer_time.CustomerSimTime(poisson_client_arr_rate, testGraph.num_cust,testGraph.num_dept, 
                                     seed=12345, datafile="sim_input/customer_requests_1000.txt",readfile=True)
#--- end setup customer data input

cust_list = []
for idept in range(testGraph.num_dept):   #TODO: currently assume each depot has same input process
    cust_list.insert(idept, data_queue.DataDepot(depot_input_model))
    print("Depot customer ,%d,%s" %(idept,depot_input_model))
dron_list = []
for idept in range(testGraph.num_dept):   #TODO: currently assume each depot has same number of drones
    dron_list.insert(idept, data_queue.DataDrone(drones_per_depot))
    print("Depot drone ,%d,%d" %(idept,drones_per_depot))
cros_list = []
for icros in range(testGraph.num_wayt):
    buffer_input_model = depot_input_model
    cros_list.insert(icros, data_queue.DataBuffer(buffer_input_model))
    print("Crossing ,%d,%s" %(icros,buffer_input_model))
#

#flow initializations
f_log.write("Flows:\n")
Flow = []   #item=(idept,icust,flow,delay)
with open(dir_sim_input+"log_flow.txt","r") as f_flow:
    reader = csv.reader(f_flow)
    pline = ""
    for line in reader:
        if(pline == "Solution rates from depot to customer routes"):
            #next lines are data we seek
            if(line[0] != ""): #ignore empty lines
                Flow.append( (int(line[0]),int(line[1]),int(line[2]),0) )
                f_log.write(str(Flow[-1])+"\n")
                #print(Flow[-1])
        else:
            pline = line[0]
#exit()

#sim initializations
f_log.write("---START SIMULATION---\n")
curr_time = 0
if(not flag_const_flow):
    (icust_cnt,ideptid,icustid,next_cust_time) = cust.readNextTime()
#
tot_cust_cnt = 1 #one customer request
tot_dron_cnt = 0 #no drones sent yet
tot_dron_done_cnt = 0 #no drones sent yet, so no drones finished mission
tot_conflict_cnt = 0
# MAIN SIMULATION LOOP --------------------------
try:
    for icnt in range(tot_icnt):


        #enqueue - customer request
        if( flag_const_flow ):
            for idept in range(testGraph.num_dept):
                for icust in range(testGraph.num_cust):
                    #check schedule
                    flow = -1
                    for iflow in Flow:
                        if( (iflow[0]==(idept+1)) and (iflow[1]==icust) ):
                            flow = iflow[2]
                    if( int(curr_time) % flow == 0 ):
                        assert(cust_list[idept].addCustomerId(icust))
                        next_cust_time = curr_time+flow
                        f_log.write("Customer,%.2f,%.2f,%d,%d,%.2f\n" %(curr_time,next_cust_time+flow, icust, 
                                                                        cust_list[idept].getCurrCustomerSize(), next_cust_time) )
                        tot_cust_cnt += 1
        #variable input ---
        elif( next_cust_time <= curr_time ):
            if(-1 < icustid and -1 < ideptid):
                assert(cust_list[ideptid].addCustomerId(icustid))
            (icust_cnt,ideptid,icustid,next_cust_time) = cust.readNextTime()
            print("Customer-%d" %icust_cnt)
            #log data (sim_time,cust_service_time, cust_id, service_queue_size, cust_service_call_time)
            f_log.write("Customer,%.2f,%.2f,%d,%d,%.2f\n" %(curr_time,next_cust_time, icustid, 
                                                            cust_list[ideptid].getCurrCustomerSize(), curr_time) )
            #set next customer event
            next_cust_call_time = curr_time
            next_cust_time = cust.getNextExpTime(next_cust_call_time)
            #
            tot_cust_cnt += 1
        #end - enqueue - customer request


        #dequeue - depot sends drone
        #TODO: currently assume each depot has same items customer wants, no optimization among depots
        for idept in range(testGraph.num_dept):
            for icust in range(testGraph.num_cust):
                #make sure there are customers
                cust_size = cust_list[idept].getCurrCustomerSize()
                #print("num customers = %d\n" %cust_size)
                if(0 < cust_size):
                    status_drone = False
                    curr_cust_id = cust_list[idept].getNextCustomerId()

                    if(flag_flow_control):
                        #check flow-schedule
                        flow = -1
                        delay = 0
                        for iflow in Flow:
                            if( (iflow[0]==(idept+1)) and (iflow[1]==icust) ):
                                flow = iflow[2]  
                                delay = iflow[3]             
                        if( (curr_cust_id==icust) and (int(curr_time-delay)%flow==0) ):
                            if(delay>0): print(idept,icust,delay)
                            status_drone = True
                        #end check flow
                    else:
                        status_drone = True

                    #successful? Then send drone to service customer
                    if( status_drone ):
                        testroute = testGraph.generateRoute(idept,curr_cust_id,nominal_gnd_spd_m_sec,drone_setup_time)
                        x = testroute[0][0]
                        y = testroute[1][0]
                        #
                        assert(dron_list[idept].groundDroneAdd(testroute,curr_cust_id,drone_setup_time,nominal_gnd_spd_m_sec,x,y))
                        cust_list[idept].clearNextCustomerId() #launched drone, so can clear request for next customer
                #end customer check
        #end - dequeue - depot sends drone


        #simulate all active drones in FLY
        for idept in range(testGraph.num_dept):
            #check drones
            if( dron_list[idept].flyDroneRemoveAllDone() ): #remove method is loop
                for idrone in dron_list[idept].drones_list_fly_done_recent:
                    #log data (sim_time, dept_id, sta, drone_id)
                    f_log.write("Drone End,%.2f,%d,%.2f,%d\n" %(curr_time,idept,idrone.list_ETA[-1],idrone.id) )
                    tot_dron_done_cnt += 1
            for testdrone in dron_list[idept].drones_list_fly:
                x = testdrone.x
                y = testdrone.y
                #
                #advance step for fly drone
                #1. check if need to put into buffer (if not doing flow control at depot)
                if(not flag_flow_control):
                    icntwaypt=0
                    for iwaypt in testGraph.Cros_Loc:
                        id = cros_list[icntwaypt].getNextDroneId()
                        if(id==testdrone.id):
                            #already waiting here, free it
                            cros_list[icntwaypt].clearNextDroneId()
                            break
                        else:
                            #not waiting here, should we add it?
                            ixw = float(iwaypt[0])
                            iyw = float(iwaypt[1])
                            ir = math.sqrt( math.pow(x-ixw,2.0)+math.pow(y-iyw,2.0) )
                            #check if not waiting and within range
                            if( ir<=50 and 0<testdrone.speed ):
                                testdrone.pause(Ts)
                                cros_list[icntwaypt].addDroneId(testdrone.id)
                                tot_conflict_cnt += 1
                                break
                        #
                        icntwaypt += 1
                #end
                #2. apply motion
                (status,messageMove) = testdrone.moveTs(Ts)
                assert(status)
                (status,messageRoute) = testdrone.updateRouteStatus()
                assert(status)
                #log data (sim_time,x,y,heading, drone_id)
                f_log.write("Drone FLY,%.2f,%.2f,%.2f,%.2f,%d\n" 
                        %(curr_time,testdrone.x,testdrone.y,testdrone.heading, testdrone.id) )
                #
                #print("T=%f MOVE>\t %s:%s" %(itime,messageMove,messageRoute))
            for testdrone in dron_list[idept].drones_list_ground:
                #advance step for wait drone
                (status,messageMove) = testdrone.moveTs(Ts)
                assert(status)
                (status,messageRoute) = testdrone.updateRouteStatus()
                assert(status)
                #print("T=%f WAIT>\t %s:%s" %(itime,messageMove,messageRoute))
                if(testdrone.status == drone.Drone.GROUND_READY):
                    assert(dron_list[idept].groundDroneLaunch())
                    #log data (sim_time,x,y,heading, cust_id,dept_id, eta, free_drones, drone_id)
                    f_log.write("Drone Start,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%d,%d\n" 
                        %(curr_time,testdrone.x,testdrone.y,testdrone.heading, testdrone.getCustomerId(),idept, 
                        testdrone.list_ETA[-1], dron_list[idept].groundVertiportAvialableSize(),dron_list[idept].flyDroneGetRecentIdLaunch()) )
                    #
                    tot_dron_cnt += 1
        #end - simulate all active drones in FLY

        if(icnt % print_rate == 0):
            f_log.write("SIM step: %d/%d, TIME=%.2f\n" %(icnt,tot_icnt,curr_time))
            print("sim done: %d/%d" %(icnt,tot_icnt))
        #end, update time
        curr_time += Ts

except:
    exc_type, exc_obj, exc_tb = sys.exc_info()
    fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
    print(exc_type, fname, exc_tb.tb_lineno)
    #e = sys.exc_info()[0]
    #print("ERROR: %s" %e )

#END
if(not flag_const_flow):
    cust.closeFile()
#------------------------------------------------
f_log.write("---END SIMULATION---\n")
f_log.write("SIM DONE %d/%d, TOTAL TIME=%.2f\n" %(icnt+1,tot_icnt,curr_time) )
f_log.write("Total number of customer requests = %d\n" %tot_cust_cnt)
f_log.write("Total number of drones finished   = %d\n" %tot_dron_done_cnt)
f_log.write("Total number of drones flew       = %d\n" %tot_dron_cnt)
f_log.write("Total number of delay updates     = %d\n" %tot_conflict_cnt)
f_log.close()

print("DONE")
