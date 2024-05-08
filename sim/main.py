# main.py
#
#  Queue UTM package drone simulation code
import sys, os
import csv
import math
#
import drone as ua                #main drone class - vehicle kinematics and control
import tbov as ov                 #main tbov class - airspace constraints and route network
import customer_time as ct        #handles customer requests
import data_queue as dq           #stores useful customer/drone data
import network_schedular as net   #handles network schedular algorithms

#for testing
import matplotlib.pyplot as plt   #main plotting
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Times New Roman'
fig, ax = plt.subplots()

#################################################
#------------------------------------------------
dir_sim_input   = "./sim_input/"
dir_sim_output  = "./sim_output/"


# read conf.txt input file ----------------------
f_log = open(dir_sim_input+"log.txt", "w+") #capture everything
try:
    f_conf = open(dir_sim_input+"conf.txt","r")
    reader = csv.reader(f_conf)
    #read data
    dir_graph_output       = reader.__next__()[1].replace(" ","") #remove whitespace
    f_log.write("graph output folder, %s\n"%dir_graph_output)
    print("graph output folder = ",dir_graph_output)
    reader.__next__()
    #------------------------
    #
    nominal_gnd_spd         = float( reader.__next__()[1] )
    f_log.write("ideal drone gnd spd (km/hr), %s\n"%nominal_gnd_spd)
    print("ideal drone gnd spd (km/hr) = ",nominal_gnd_spd)
    #convert to (m/sec) for use in sim
    nominal_gnd_spd_m_sec = nominal_gnd_spd * 1000/(60*60)
    #
    drone_setup_time        = float( reader.__next__()[1] )
    f_log.write("ideal drone setup time (sec), %s\n"%drone_setup_time)
    print("ideal drone setup time (sec) = ",drone_setup_time)
    #
    poisson_client_arr_rate = float( reader.__next__()[1] )
    f_log.write("customer request parameter (orders/hr), %s\n"%poisson_client_arr_rate)
    print("customer request parameter (orders/hr) = ",poisson_client_arr_rate)
    #convert to (orders/sec) for use in sim
    poisson_client_arr_rate = poisson_client_arr_rate / (60*60)
    #
    waypoint_clearance      = int( reader.__next__()[1] )
    f_log.write("reached waypoint distance (meter), %s\n"%waypoint_clearance)
    reader.__next__()
    #------------------------
    #
    Ts                      = float( reader.__next__()[1] )
    f_log.write("Time step (sec), %s\n"%Ts)
    print("Time step (sec) = ",Ts)
    #
    tot_icnt                = int( reader.__next__()[1] )
    f_log.write("Total number of time steps, %s\n"%tot_icnt)
    print("Total number of time steps = ",tot_icnt)
    #
    print_rate              = int( reader.__next__()[1] )
    f_log.write("Output print rate, %s\n"%print_rate)
    print("Output print rate = ",print_rate)
    reader.__next__()
    #------------------------
    #
    drones_per_depot        = int( reader.__next__()[1] )  #ideal, might change later
    f_log.write("ideal drone amount per depot, %s\n"%drones_per_depot)
    print("ideal drone amount per depot = ",drones_per_depot)
    #
    drone_ideal_width       = int( reader.__next__()[1] )
    f_log.write("ideal drone width (meter), %s\n"%drone_ideal_width)
    print("ideal drone width (meter) = ",drone_ideal_width)
    #
    drone_ideal_length     = int( reader.__next__()[1] )
    f_log.write("ideal drone length (meter), %s\n"%drone_ideal_length)
    print("ideal drone length (meter) = ",drone_ideal_length)
    reader.__next__()
    #------------------------
    #
    TBOV_width              = int( reader.__next__()[1] )
    f_log.write("TBOV width from drone center (meter), %s\n"%TBOV_width)
    print("TBOV width from drone center (meter) = ",TBOV_width)
    #
    TBOV_length             = int( reader.__next__()[1] )
    f_log.write("TBOV length from drone center (meter), %s\n"%TBOV_length)
    print("TBOV length from drone center (meter) = ",TBOV_length)
    #
    TBOV_Ts                 = float( reader.__next__()[1] )
    f_log.write("TBOV update rate (sec), %s\n"%TBOV_Ts)
    print("TBOV update rate (sec) = ",TBOV_Ts)
    #
    depot_TBOV_clearance    = int( reader.__next__()[1] )
    f_log.write("depot TBOV ignore distance (meter), %d\n"%depot_TBOV_clearance)
    reader.__next__()
    #------------------------
    #
except Exception as e:
    print("%s" %e )
    sys.exit(1)
f_conf.close()
f_log.write("Creating files for each subgraph...\n") #save individual log per igraph below

#various initializations ------------------------
size = 1            #drone box size in meters
BUFFER_size = 25    #buffer box size in meters 
flag_const_flow = False
#---
flag_flow_control = True
flag_buffer_enable = True
#---
flag_test1 = False   #TBOV and network
flag_test2 = False   #Customer Data
#---
flag_sim = True    #start main sim


#################################################
#read depot data --------------------------------
igraph = 0
Dept_Loc = []   #item=(dept_id,x,y)
f_depot_data = open(dir_graph_output+"depot_loc.txt", "r")
reader = csv.reader(f_depot_data)
for iline in reader:
    #new sub-graph?
    if( "Depot" == iline[0] ):
        idepot = int( iline[1] )
        ix     = float( iline[2] )
        iy     = float( iline[3] )
        Dept_Loc[igraph-1].append( (idepot, ix,iy) )
    else:
        #yes, new sub-graph...
        Dept_Loc.insert(igraph,[])
        igraph = igraph + 1
f_depot_data.close()

#data to extract --------------------------------
#---TBOV network/graph information
route_list=[]
route_boxlist=[]
TBOVbuffers = ov.TBOVbuffer()
#---
flow_list = []  #item=(idept,icust,flow,delay)
cust_input_list  = []   # list of waiting customers/requests
cust_output_list = []   # list of server (processor) models
cros_list = []  # list of buffers (servers/processors)
dron_list = []  # holder of drones per depot (on mission from that depot)
#---depot locations
xdata_dept_list = []
ydata_dept_list = []
#---customer locations
xdata_cust_list = []
ydata_cust_list = []
#run through all sub-graphs ---------------------
num_graphs = igraph
for igraph in range(num_graphs):
    #obtain sub-folder for igraph
    subgraph = "G"+str(igraph)+"/"
    dir_subgraph = dir_graph_output + subgraph
    #create sub-folder for sim igraph data
    dir_sim_input2 = dir_sim_input + subgraph
    print()
    try:
        os.mkdir(dir_sim_input2) #--> should go to except if created already..
        print("Error: Directory <"+dir_sim_input2+"> created, need to run flowopt_uni.py first!")
        os.rmdir(dir_sim_input2)
        sys.exit(1)
    except:
        print("Directory <"+dir_sim_input2+"> exists, all good!")
    #---
    f_log_G = open(dir_sim_input2+"log_G.txt", "w+") #capture everything for this igraph
    f_log_G.write("RUNNING SETUP for Directory <"+dir_sim_input2+"> :: <"+subgraph+">\n")
    f_log.write("RUNNING SETUP for Directory <"+dir_sim_input2+"> :: <"+subgraph+"> :: check corresponding f_log_G \n")

    #--- setup graph / mission planner
    graphFile = dir_subgraph+"graph_matrix.csv"
    graphMission = ua.DroneMissionPlanner()
    (status,message) = graphMission.loadGraph(graphFile)
    f_log_G.write(message+"\n")
    assert( status )
    pathsFile = dir_subgraph+"paths_s.txt"
    (status,message) = graphMission.loadPaths(pathsFile)
    f_log_G.write(message+"\n")
    assert( status )
    depotLocFile = dir_subgraph+"depot_loc.txt"
    (status,message) = graphMission.loadDepots(depotLocFile)
    f_log_G.write(message+"\n")
    assert( status )
    custLocFile = depotLocFile
    (status,message) = graphMission.loadCustomers(custLocFile)
    f_log_G.write(message+"\n")
    assert( status )
    #generate some routes
    for idept in range(graphMission.num_dept):
        for icust in range(graphMission.num_cust):
            testroute = graphMission.generateRoute(idept,icust,nominal_gnd_spd_m_sec,drone_setup_time)
            route_list.append( (igraph,idept,icust,testroute) ) #subgraph,dept,cust,route_info
            f_log_G.write("\tRoute["+str(igraph)+":"+str(idept)+"->"+str(icust)+"] <"+str(testroute)+">\n")
    for iroute in route_list:
        iTBOV = ov.TBOV(iroute[3],size,TBOV_length,TBOV_width)
        iTBOV.updatePts()
        boxlist=[]
        (status,boxlist) = iTBOV.getBoxes()
        f_log_G.write("\tRoute Pts<"+str(boxlist)+">\n")
        assert(status)
        flag=True #assume not exist
        #check to see if there is already OI from previous
        for ioi in route_boxlist:
            if(ioi[0]==igraph and ioi[1]==iroute[1] and ioi[2]==iroute[2]):
                flag=False #exists(same subgraph,dept,cust) so exit
                break
        if(flag):
            route_boxlist.append( (iroute[0],iroute[1],iroute[2],boxlist) ) #subgraph,dept,cust,{4pt-rect}
    #--- end setup graph / mission planner

    #--- setup locations
    # Depot locations data
    for idata in graphMission.Dept_Loc:
        flag=True #assume not exist
        #check to see if there is already this location from previous
        for iloc in range(len(xdata_dept_list)):
            if(xdata_dept_list[iloc]==idata[1] and ydata_dept_list[iloc]==idata[2]):
                flag=False #exists so exit
                break
        if(flag):
            xdata_dept_list.append( idata[1] )
            ydata_dept_list.append( idata[2] )
    f_log_G.write("\tNumber of Depots <"+str(len(graphMission.Dept_Loc))+">\n")
    # Customer locations data
    for idata in graphMission.Cust_Loc:
        flag=True #assume not exist
        #check to see if there is already this location from previous
        for iloc in range(len(xdata_cust_list)):
            if(xdata_cust_list[iloc]==idata[1] and ydata_cust_list[iloc]==idata[2]):
                flag=False #exists so exit
                break
        if(flag):
            xdata_cust_list.append( idata[1] )
            ydata_cust_list.append( idata[2] )
    f_log_G.write("\tNumber of Customers <"+str(len(graphMission.Cust_Loc))+">\n")
    # Buffer locations data
    bufferptslist = graphMission.getCrossings()
    f_log_G.write("\tNumber of Crossings <"+str(len(bufferptslist))+"> :"+str(bufferptslist)+"\n")
    for i in range(len(bufferptslist)):
        Loc = ( bufferptslist[i][0] , bufferptslist[i][1] )
        TBOVbuffers.placeBuffer(Loc,BUFFER_size)
        f_log_G.write("\t\tBuffer <"+str(TBOVbuffers.buffers[-1])+">\n")
    #endfor
    (status,bufferlist) = TBOVbuffers.getBuffers()
    f_log_G.write("\tNumber of Buffers <"+str(len(bufferptslist))+">\n")
    assert(status)
    #--- end setup locations

    #--- Queuing Models set-up
    cust_output_list.insert(igraph,[]) #add list for this igraph
    for idept in range(graphMission.num_dept):   #TODO: currently assume each depot has same input process
        depot_input_model = "FCFS"
        cust_output_list[igraph].append( dq.DataDepot(depot_input_model) )
        f_log.write("\tDepot customer ,%d,%s\n" %(idept,depot_input_model))
    dron_list.insert(igraph,[]) #add list for this igraph
    for idept in range(graphMission.num_dept):   #TODO: currently assume each depot has same number of drones
        drones_per_depot = 100
        dron_list[igraph].append( dq.DataDrone(drones_per_depot) )
        f_log.write("\tDepot drone ,%d,%d\n" %(idept,drones_per_depot))
    cros_list.insert(igraph,[]) #add list for this igraph
    for icros in range(graphMission.num_wayt):
        buffer_input_model = "FCFS"
        buffer_output_model = "FCFS"
        num_exits = 2
        serivce_time = BUFFER_size / nominal_gnd_spd_m_sec #travel time across buffer in sec 
        cros_list[igraph].append((icros,dq.DataBuffer(buffer_input_model),dq.DataBuffer(buffer_input_model),net.SchedularNode(num_exits,serivce_time,buffer_output_model), (bufferptslist[icros][0],bufferptslist[icros][1]) ))
        f_log.write("\tCrossing ,%d,%s\n" %(icros,buffer_input_model))
    #--- end Queue set-up

    #--- setup customer data input
    custfile=dir_sim_input2+"customer_requests.txt"
    cust_read = ct.CustomerSimTime(poisson_client_arr_rate, graphMission.num_cust,graphMission.num_dept, 
                                seed=12345, datafile=custfile,readfile=True)
    cust_input_list.append( (igraph,cust_read) )
    f_log.write("\tCustomer requests added<"+custfile+">\n")
    #--- end setup customer data input

    #--- flow initializations
    f_log_G.write("\tFlows:\n")
    flowfile=dir_sim_input2+"log_flow_G.txt"
    with open(flowfile,"r") as f_flow:
        reader = csv.reader(f_flow)
        pline = ""
        for line in reader:
            if(pline == "Solution rates from depot to depot routes"):
                #next lines are data we seek
                if(line[0] != ""): #ignore empty lines
                    flow_list.append( (igraph,int(line[0]),int(line[1]),int(line[2]),int(float(line[3]))) )
                    f_log_G.write("\t\t"+str(flow_list[-1])+"\n")
                    #print(Flow[-1])
            else:
                pline = line[0]
    f_log.write("\tFlow rates added<"+flowfile+">\n")
    #--- end flow init.

    f_log_G.close() #end for this igraph
#end run through all sub-graphs -----------------
print()

#run pre-test 1
if(flag_test1):
    f_log.write("RUNNING TEST 1: TBOV and network\n")
    print("RUNNING TEST 1: TBOV and network")
    #plot values
    h = 150 
    color_location = 'orange'
    color_TBOV = 'blue'
    color_BUFFER = 'green'
    #init print
    plt.cla()
    xmin = min(xdata_dept_list)
    xmax = max(xdata_dept_list)
    ymin = min(ydata_dept_list)
    ymax = max(ydata_dept_list)
    plt.axis([xmin-h,xmax+h,ymin-h,ymax+h])
    # 2D top-view of OIs
    for iroute_box in route_boxlist:
        boxlist = iroute_box[3]
        for ibox in boxlist:
            #plt.scatter(ibox[0],ibox[1], c=color_TBOV,s=15, edgecolors='black')
            plt.plot( [ibox[0],ibox[2]],[ibox[1],ibox[3]], c=color_TBOV, alpha=0.1 )
            plt.plot( [ibox[2],ibox[4]],[ibox[3],ibox[5]], c=color_TBOV, alpha=0.1 )
            plt.plot( [ibox[4],ibox[6]],[ibox[5],ibox[7]], c=color_TBOV, alpha=0.1 )
            plt.plot( [ibox[6],ibox[0]],[ibox[7],ibox[1]], c=color_TBOV, alpha=0.1 )
        #end box plot
    #end OI
    # 2D top-view of simulation map
    plt.scatter(xdata_dept_list,ydata_dept_list, c=color_location,s=80, edgecolors='none', label='Location')
    for icnt in range(len(xdata_dept_list)):
        plt.text(xdata_dept_list[icnt]+BUFFER_size,ydata_dept_list[icnt]+BUFFER_size,"L"+str(icnt))
    #2D top-view of buffers
    (status,bufferlist) = TBOVbuffers.getBuffers()
    f_log.write("\tNumber of Buffers <"+str(len(bufferlist))+">\n")
    assert(status)
    icnt=0
    for ibuffer in bufferlist:
        x0 = ibuffer[0][2]
        y0 = ibuffer[0][3]
        x1 = ibuffer[1][2]
        y1 = ibuffer[1][3]
        x2 = ibuffer[2][2]
        y2 = ibuffer[2][3]
        x3 = ibuffer[3][2]
        y3 = ibuffer[3][3]
        plt.text(ibuffer[0][0]+BUFFER_size,ibuffer[0][1]+BUFFER_size,"B"+str(icnt))
        plt.scatter(ibuffer[0][0],ibuffer[0][1], c=color_BUFFER,s=BUFFER_size, edgecolors='black', label='Buffer')
        plt.plot( [x0,x1],[y0,y1], c=color_BUFFER )
        plt.plot( [x1,x2],[y1,y2], c=color_BUFFER )
        plt.plot( [x2,x3],[y2,y3], c=color_BUFFER )
        plt.plot( [x3,x0],[y3,y0], c=color_BUFFER )
        icnt=icnt+1
    #end buffers
    #end, init print
    #---
    f_log.write("\tCreating plot\n")
    plt.legend()
    plt.grid(True)
    plt.show()
    f_log.write("Finished TEST 1\n")
    print("Finished TEST 1")
#end pre-test 1
#run pre-test 2
if(flag_test2):
    f_log.write("RUNNING TEST 2: Customer Data\n")
    print("RUNNING TEST 2: Customer Data")
    # read data and print it out
    for icust in cust_input_list:
        igraph = icust[0]
        cust_read = icust[1]
        try:
            f_log.write("Graph["+str(igraph)+"]\ncnt \t ids \t time \n")
            while(True):
                (icnt, ideptid,icustid,it) = cust_read.readNextTime()
                f_log.write("%d \t(%d %d \t %f)\n" %(icnt, ideptid,icustid,it))
        except:
            f_log.write("done with Graph["+str(igraph)+"]\n")
        cust_read.closeFile()
    #---
    f_log.write("Finished TEST 2\n")
    print("Finished TEST 2")


# DONE with tests.. should we continue with sim?-
if(flag_test1 or flag_test2):
    print("Ran Tests, so exit now..")
    sys.exit(0)
if(not flag_sim):
    print("Not running sim, so exit now..")
    sys.exit(0)

#sim initializations
f_log.write("---START SIMULATION---\n")
curr_time = 0
tot_cust_cnt = 0 #no customer requests yet
tot_dron_cnt = 0 #no drones sent yet
tot_dron_done_cnt = 0 #no drones sent yet, so no drones finished mission
tot_conflict_cnt = 0
# pre-data readings
cust_data_list = []
for icust in cust_input_list:
    igraph = icust[0]
    cust_read = icust[1]
    (icnt, ideptid,icustid,it) = cust_read.readNextTime()
    cust_data_list.insert(igraph, [icnt, ideptid,icustid,it] )
    #new customer request added.. log it..
    f_log.write("Customer,%.2f,%.2f,%d,%d,%.2f,%d\n" %(curr_time,it, icustid, cust_output_list[igraph][ideptid].getCurrCustomerSize(), curr_time,igraph) )
    tot_cust_cnt += 1 
# MAIN SIMULATION LOOP --------------------------
try:
    for icnt in range(tot_icnt):

        #enqueue - customer request
        #variable input ---
        for icust in cust_input_list:
            igraph = icust[0]
            cust_read = icust[1]
            ideptid = cust_data_list[igraph][1]
            icustid = cust_data_list[igraph][2]
            next_cust_time = cust_data_list[igraph][3]
            #check if time..
            if( next_cust_time<=curr_time and -1<icustid and -1<ideptid):
                assert(cust_output_list[igraph][ideptid].addCustomerId(icustid))
                try:
                    (icust_cnt,ideptid,icustid,next_cust_time) = cust_read.readNextTime()
                    cust_data_list[igraph][0] = icust_cnt
                    cust_data_list[igraph][1] = ideptid
                    cust_data_list[igraph][2] = icustid
                    cust_data_list[igraph][3] = next_cust_time
                    #log data (sim_time,cust_service_time, cust_id, service_queue_size, cust_service_call_time,igraph)
                    f_log.write("Customer,%.2f,%.2f,%d,%d,%.2f,%d\n" %(curr_time,next_cust_time, icustid, 
                                                            cust_output_list[igraph][ideptid].getCurrCustomerSize(), curr_time,igraph) )
                    tot_cust_cnt += 1
                except:
                    print("Graph-%d Customer-%d DONE=%s" %(igraph,cust_data_list[igraph][0],str(cust_data_list[igraph])) )
                    cust_data_list[igraph][0] = -1
                    cust_data_list[igraph][1] = -1
                    cust_data_list[igraph][2] = -1
                    cust_data_list[igraph][3] = tot_icnt*Ts
        #end - enqueue - customer request


        #dequeue - depot sends drone
        #TODO: currently assume each depot has same items customer wants, no optimization among depots
        for igraph in range(num_graphs):
            for idept in range(len(cust_output_list[igraph])):
                #make sure there are customers
                cust_size = cust_output_list[igraph][idept].getCurrCustomerSize()
                #print("num customers = %d\n" %cust_size)
                if(0 < cust_size):
                    status_drone = False
                    curr_cust_id = cust_output_list[igraph][idept].getNextCustomerId()

                    if(flag_flow_control):
                        #check flow-schedule
                        for iflow in flow_list:
                            _graph = iflow[0] #(igraph,idept,icust,flow,delay)
                            _idept = iflow[1]
                            _icust = iflow[2]
                            _flow  = 0
                            _delay = 0
                            if( (_graph==igraph) and (_idept==idept+1) and (_icust==curr_cust_id) ): #depot is offset by one, might need to fix later?
                                _flow  = iflow[3]  
                                _delay = iflow[4]  
                                if( (int(curr_time-_delay)%_flow==0) ):
                                    if(_delay>0): print(_idept,_icust,_delay)
                                    status_drone = True
                                    break #found the correct one.. 
                        #end check flow
                    else:
                        status_drone = True

                    #successful? Then send drone to service customer
                    if( status_drone ):
                        #find corresponding route
                        for iroute in route_list:
                            #---(igraph,idept,icust,testroute) #subgraph,dept,cust,route_info
                            t_graph = iroute[0]
                            t_dept = iroute[1]
                            t_cust = iroute[2]
                            if( igraph==t_graph and idept==t_dept and curr_cust_id==t_cust ):
                                testroute = iroute[3]
                                x = testroute[0][0]
                                y = testroute[1][0]
                                #
                                assert(dron_list[igraph][idept].groundDroneAdd(testroute,curr_cust_id,drone_setup_time,nominal_gnd_spd_m_sec,x,y))
                                cust_output_list[igraph][idept].clearNextCustomerId() #launched drone, so can clear request for next customer
                                break #exit for-loop (no need to look for others)
                #end customer check
        #end - dequeue - depot sends drone


        #simulate all active drones in FLY
        for igraph in range(num_graphs):
            for idept in range(len(cust_output_list[igraph])):
                #check drones
                if( dron_list[igraph][idept].flyDroneRemoveAllDone() ): #remove method is loop
                    for idrone in dron_list[igraph][idept].drones_list_fly_done_recent:
                        #log data (sim_time, dept_id, sta, drone_id)
                        f_log.write("Drone End,%.2f,%d,%.2f,%d,%d\n" %(curr_time,idept,idrone.list_ETA[-1],idrone.id,igraph) )
                        tot_dron_done_cnt += 1
                for testdrone in dron_list[igraph][idept].drones_list_fly:
                    x = testdrone.x
                    y = testdrone.y
                    #
                    #advance step for fly drone
                    #1. check if need to put into buffer 
                    iflag=False
                    cros_igraph_list = cros_list[igraph]
                    for ibuffer in cros_igraph_list:
                        icros = ibuffer[0] #crossing number label
                        iqueu = ibuffer[1] #list of waiting drones at crossing
                        iqueu_passed = ibuffer[2] #list of approved drones to fly in buffer
                        ische = ibuffer[3] #schedular for processing waiting drones

                        checkexit=1 #TODO: include exit checking...
                        (status,openexit) = ische.checkNextExit(checkexit,curr_time)
                        id = iqueu.getNextDroneId()
                        if(id==testdrone.id):
                            #already waiting here, should we free it?
                            if(status):
                                #good to-go
                                iqueu_passed.addDroneId(iqueu.clearNextDroneId())
                            else:
                                #keep holding it here
                                flag = True
                                testdrone.pause(Ts)
                                tot_conflict_cnt += 1 #confict since waiting here for more time..
                            break
                        else:
                            #not waiting here, should we add it?
                            ixw = ibuffer[4][0]
                            iyw = ibuffer[4][1]
                            ir = math.sqrt( math.pow(x-ixw,2.0)+math.pow(y-iyw,2.0) )
                            #check if not waiting and within range
                            if( flag_buffer_enable and ir<=BUFFER_size and 0<testdrone.speed ): #TODO: not necessary a conflict but for simplicity hold all here for one time step
                                #check if approved to fly inside
                                _d_list = iqueu_passed.getList()
                                _flag = True
                                for _id in _d_list:
                                    if(_id==testdrone.id):
                                        _flag = False
                                        break
                                if(_flag):
                                    flag = True
                                    testdrone.pause(Ts)
                                    iqueu.addDroneId(testdrone.id)
                                    break
                        #endif
                    #endfor
                    #if(flag):
                    #    plt.scatter(x,y, c='red',s=20, edgecolors='none', label='wait UAV')
                    #else:
                    #    plt.scatter(x,y, c='purple',s=20, edgecolors='none', label='UAV')
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
                for testdrone in dron_list[igraph][idept].drones_list_ground:
                    #advance step for wait drone
                    (status,messageMove) = testdrone.moveTs(Ts)
                    assert(status)
                    (status,messageRoute) = testdrone.updateRouteStatus()
                    assert(status)
                    #print("T=%f WAIT>\t %s:%s" %(itime,messageMove,messageRoute))
                    if(testdrone.status == ua.Drone.GROUND_READY):
                        assert(dron_list[igraph][idept].groundDroneLaunch())
                        #log data (sim_time,x,y,heading, cust_id,dept_id, eta, free_drones, drone_id)
                        f_log.write("Drone Start,%.2f,%.2f,%.2f,%.2f,%d,%d,%.2f,%d,%d,%d\n" 
                            %(curr_time,testdrone.x,testdrone.y,testdrone.heading, testdrone.getCustomerId(),idept, 
                            testdrone.list_ETA[-1], dron_list[igraph][idept].groundVertiportAvialableSize(),dron_list[igraph][idept].flyDroneGetRecentIdLaunch(),igraph) )
                        #
                        tot_dron_cnt += 1
        #end - simulate all active drones in FLY

        if(icnt % print_rate == 0):
            f_log.write("SIM step: %d/%d, TIME=%.2f\n" %(icnt,tot_icnt,curr_time))
            print("SIM step: %d/%d, TIME=%.2f" %(icnt,tot_icnt,curr_time))
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
    for icust in cust_input_list:
        cust_read = icust[1]
        cust_read.closeFile()
#------------------------------------------------
f_log.write("---END SIMULATION---\n")
f_log.write("SIM DONE %d/%d, TOTAL TIME=%.2f\n" %(icnt+1,tot_icnt,curr_time) )
f_log.write("Total number of customer requests = %d\n" %tot_cust_cnt)
f_log.write("Total number of drones finished   = %d\n" %tot_dron_done_cnt)
f_log.write("Total number of drones flew       = %d\n" %tot_dron_cnt)
f_log.write("Total number of delay updates     = %d\n" %tot_conflict_cnt)
f_log.close()

print("DONE, check log..")
