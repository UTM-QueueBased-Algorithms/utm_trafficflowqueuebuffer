# network_schedular.py 
#
#   Queue schedular with different algorithms for handling network management.
import random

#####################################################################
class SchedularNode:
    '''
    Handling different algorithms for handling network management
    via queue models. Here the schedular is placed at a node in the
    network {depot or buffer}

    numExits == number of exiting edges from node
    type     == FCFS, Round-Robin, Random, ...
    '''

    def __init__(self,numExits,serviceTime=0,type="FCFS"):
        self.numExits = numExits
        self.type = type
        #
        self.currExit = 0
        self.currTime = 0
        self.delayTime = serviceTime #delay between services (travel time for a drone to pass intersection)
        #
        self.seed = 111
        self.rand = random.Random(self.seed)
    
    def checkNextExit(self,exit,curr_time):
        status = False
        #check if new process allowed, ie time passed
        tmpTs = curr_time - self.currTime
        if( tmpTs > self.delayTime ):
            #
            #check schedular type
            if(   "FCFS" == self.type ):
                status = True
                self.currExit = exit #assign openExit to first check
            elif( "Round-Robin" == self.type ):
                if( exit == self.currExit ):
                    status = True
                else:
                    self.currExit = self.currExit + 1 #goto next exit
                    if( (self.numExits-1) < self.currExit ): #wrap-around
                        self.currExit = 0
            elif( "Random" == self.type ):
                if( exit == self.currExit ):
                    status = True
                else:
                    self.currExit = self.rand.randint(0,self.numExits-1) #random pick next exit
            #end check schedular
            #
            self.currTime = curr_time #update new last processed time
        #end check process
        return (status,self.currExit)

#####################################################################
# TESTING MODULE
#################
if __name__ == "__main__":
    import drone as ua          #main drone class - vehicle kinematics and control
    import tbov as ov           #main tbov class - airspace constraints and route network
    import customer_time as ct  #handles customer requests
    import data_queue as dq     #stores useful customer/drone data

    #  Uses mathplotlib. `python -m pip install -U matplotlib`
    import matplotlib.pyplot as plt             #main plotting
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = 'Times New Roman'
    fig, ax = plt.subplots()

    import math #for 'math'
    import sys, os #for error handling stuff

    #---
    speed = 20  #drone speed in meters/sec
    setup = 10 #sec
    Ts = 1 #sec (time step)
    size = 1 #drone box size in meters
    TBOV_l = 50 #ov size along heading in meters
    TBOV_w = 20 #ov size perpendicular to heading in meters
    BUFFER_size = 25 #buffer box size in meters 
    #plot values
    h = 150 
    color_depot = 'orange'
    color_customer = 'red'
    color_TBOV = 'blue'
    color_BUFFER = 'green'
    #--- setup graph / mission planner
    graphFile = "sim_input/_test/graph_matrix.csv"
    testGraph = ua.DroneMissionPlanner()
    assert( testGraph.loadGraph(graphFile) )
    #
    pathsFile = "sim_input/_test/paths_s.txt"
    assert( testGraph.loadPaths(pathsFile) )
    #
    depotLocFile = "sim_input/_test/depot_loc.txt"
    assert( testGraph.loadDepots(depotLocFile) )
    custLocFile = "sim_input/_test/customer_loc.txt"
    assert( testGraph.loadCustomers(custLocFile) )
    #generate some routes
    numRoutes=(testGraph.num_dept)*(testGraph.num_cust)
    testroute_list=[]
    for idept in range(testGraph.num_dept):
        for icust in range(testGraph.num_cust):
            testroute = testGraph.generateRoute(idept,icust,speed,setup)
            testroute_list.append( (idept,icust,testroute) ) #dept,cust,route_info
            print("Route["+str(idept)+"->"+str(icust)+"] <"+str(testroute)+">" )
    testroute_boxlist=[]
    for iroute in testroute_list:
        iTBOV = ov.TBOV(iroute[2],size,TBOV_l,TBOV_w)
        boxlist=[]
        (status,boxlist) = iTBOV.getBoxes()
        assert(status)
        testroute_boxlist.append( (iroute[0],iroute[1],boxlist) )
    #--- end setup graph / mission planner

    #--- setup locations
    # Depot locations data
    xdata_dept_list = []
    ydata_dept_list = []
    for idata in testGraph.Dept_Loc:
        xdata_dept_list.append( idata[1] )
        ydata_dept_list.append( idata[2] )
    print("Number of Depots <"+str(len(testGraph.Dept_Loc))+">")
    # Customer locations data
    xdata_cust_list = []
    ydata_cust_list = []
    for idata in testGraph.Cust_Loc:
        xdata_cust_list.append( idata[1] )
        ydata_cust_list.append( idata[2] )
    print("Number of Customers <"+str(len(testGraph.Cust_Loc))+">")
    # Buffer locations data
    bufferptslist = testGraph.getCrossings()
    testTBOVbuffers = ov.TBOVbuffer()
    print("Number of Crossings <"+str(len(bufferptslist))+"> :"+str(bufferptslist))
    for i in range(len(bufferptslist)):
        Loc = ( bufferptslist[i][0] , bufferptslist[i][1] )
        testTBOVbuffers.placeBuffer(Loc,BUFFER_size)
        print("Buffer <"+str(testTBOVbuffers.buffers[-1])+">")
    #endfor
    (status,bufferlist) = testTBOVbuffers.getBuffers()
    assert(status)
    print("Number of Buffers <"+str(len(bufferlist))+">")
    #--- end setup locations

    #--- setup customer data input
    poisson_client_arr_rate = 100 #orders/hour
    scale = poisson_client_arr_rate / (60*60) #convert to (orders/sec) for use in sim
    cust = ct.CustomerSimTime(scale, testGraph.num_cust,testGraph.num_dept, seed=12345)
    #--- end setup customer data input

    #--- Queuing Models set-up
    cust_list = []
    for idept in range(testGraph.num_dept):   #TODO: currently assume each depot has same input process
        depot_input_model = "FCFS"
        cust_list.insert(idept, dq.DataDepot(depot_input_model))
        print("Depot customer ,%d,%s" %(idept,depot_input_model))
    dron_list = []
    for idept in range(testGraph.num_dept):   #TODO: currently assume each depot has same number of drones
        drones_per_depot = 100
        dron_list.insert(idept, dq.DataDrone(drones_per_depot))
        print("Depot drone ,%d,%d" %(idept,drones_per_depot))
    cros_list = []
    for icros in range(testGraph.num_wayt):
        buffer_input_model = "FCFS"
        buffer_output_model = "FCFS"
        num_exits = 2
        serivce_time = BUFFER_size / speed #travel time across buffer in sec 
        cros_list.insert(icros, (icros,dq.DataBuffer(buffer_input_model),SchedularNode(num_exits,serivce_time,buffer_output_model)) )
        print("Crossing ,%d,%s" %(icros,buffer_input_model))
    #--- end Queue set-up
    
    #--- preliminary MAIN sim set-up other
    itime = 0
    it = 0
    #
    icustid = -1
    ideptid = -1
    #
    icnt = 0
    tot_dron_done_cnt = 0
    #--- MAIN SIM
    while(True):
        tot_fly_cnt = 0

        #init print
        plt.cla()
        plt.axis([-h,1000+h,-h,1000+h])
        # 2D top-view of simulation map
        plt.scatter(xdata_dept_list,ydata_dept_list, c=color_depot,s=80, edgecolors='none', label='depot')
        plt.scatter(xdata_cust_list,ydata_cust_list, c=color_customer,s=80, edgecolors='none', label='customer')
        # 2D top-view of OIs
        for iroute_box in testroute_boxlist:
            boxlist = iroute_box[2]
            for ibox in boxlist:
                #plt.scatter(ibox[0],ibox[1], c=color_TBOV,s=15, edgecolors='black')
                plt.plot( [ibox[0],ibox[2]],[ibox[1],ibox[3]], c=color_TBOV, alpha=0.1 )
                plt.plot( [ibox[2],ibox[4]],[ibox[3],ibox[5]], c=color_TBOV, alpha=0.1 )
                plt.plot( [ibox[4],ibox[6]],[ibox[5],ibox[7]], c=color_TBOV, alpha=0.1 )
                plt.plot( [ibox[6],ibox[0]],[ibox[7],ibox[1]], c=color_TBOV, alpha=0.1 )
            #end box plot
        #end OI
        #2D top-view of buffers
        icnt=0
        for ibuffer in bufferlist:
            icnt=icnt+1
            x0 = ibuffer[0][2]
            y0 = ibuffer[0][3]
            x1 = ibuffer[1][2]
            y1 = ibuffer[1][3]
            x2 = ibuffer[2][2]
            y2 = ibuffer[2][3]
            x3 = ibuffer[3][2]
            y3 = ibuffer[3][3]
            plt.text(ibuffer[0][0]+BUFFER_size,ibuffer[0][1]+BUFFER_size,"B"+str(icnt))
            #plt.scatter(ibuffer[0][0],ibuffer[0][1], c=color_BUFFER, edgecolors='black')
            plt.plot( [x0,x1],[y0,y1], c=color_BUFFER )
            plt.plot( [x1,x2],[y1,y2], c=color_BUFFER )
            plt.plot( [x2,x3],[y2,y3], c=color_BUFFER )
            plt.plot( [x3,x0],[y3,y0], c=color_BUFFER )
        #end buffers
        #end, init print

        try:
            #read next free time for takeoff
            if( it <= itime ):
                if(-1 < icustid and -1 < ideptid):
                    assert(cust_list[ideptid].addCustomerId(icustid))
                it = cust.getNextExpTime(itime)
                icustid = cust.getCustomerId()
                ideptid = cust.getDepotId()
                icnt = icnt + 1
                cust_size0 = cust_list[0].getCurrCustomerSize()
                cust_size1 = cust_list[1].getCurrCustomerSize()
                print("\nCUSTOMER CNT = "+str(icnt))
                print("   depot<"+str(cust_size0)+":"+str(list(cust_list[0].customer_queue.queue))+">")
                print("   depot<"+str(cust_size1)+":"+str(list(cust_list[1].customer_queue.queue))+">")
            #---

            for idept in range(testGraph.num_dept):
                #make sure there are customers
                cust_size = cust_list[idept].getCurrCustomerSize()
                #print("num customers = %d\n" %cust_size)
                if(0 < cust_size):
                    curr_cust_id = cust_list[idept].getNextCustomerId()
                    #find the route to assign drone to...
                    for iroute in testroute_list:
                        if(idept==iroute[0] and curr_cust_id==iroute[1]):
                            x = iroute[2][0][0]
                            y = iroute[2][1][0]
                            #
                            assert(dron_list[idept].groundDroneAdd(iroute[2],curr_cust_id,size,speed,x,y))
                            break #found route.. no need to check others
                    cust_list[idept].clearNextCustomerId() #launched drone, so can clear request for next customer
                #check drones
                if( dron_list[idept].flyDroneRemoveAllDone() ): #remove method is loop
                    tot_dron_done_cnt += len( dron_list[idept].drones_list_fly_done_recent )
                for testdrone in dron_list[idept].drones_list_fly:
                    tot_fly_cnt += 1
                    x = testdrone.x
                    y = testdrone.y
                    #
                    #advance step for fly drone
                    #1. check if need to put into buffer
                    flag=False
                    for ibuffer in cros_list:
                        icros = ibuffer[0] #crossing number label
                        iqueu = ibuffer[1] #list of waiting drones at crossing
                        ische = ibuffer[2] #schedular for processing waiting drones

                        checkexit=1 #TODO: include exit checking...
                        (status,openexit) = ische.checkNextExit(checkexit,itime)
                        id = iqueu.getNextDroneId()
                        if(id==testdrone.id):
                            #already waiting here, should we free it?
                            if(status):
                                #good to-go
                                iqueu.clearNextDroneId()
                            else:
                                #keep holding it here
                                flag = True
                                testdrone.pause(Ts)
                            break
                        else:
                            #not waiting here, should we add it?
                            ixw = bufferptslist[icros][0]
                            iyw = bufferptslist[icros][1]
                            ir = math.sqrt( math.pow(x-ixw,2.0)+math.pow(y-iyw,2.0) )
                            #check if not waiting and within range
                            if( ir<=BUFFER_size and 0<testdrone.speed ):
                                flag = True
                                testdrone.pause(Ts)
                                iqueu.addDroneId(testdrone.id)
                                break
                        #endif
                    #endfor
                    if(flag):
                        plt.scatter(x,y, c='red',s=20, edgecolors='none', label='wait UAV')
                    else:
                        plt.scatter(x,y, c='purple',s=20, edgecolors='none', label='UAV')
                    plt.text(x+testdrone.size,y+testdrone.size,testdrone.id)
                    #end
                    #2. apply motion
                    (status,messageMove) = testdrone.moveTs(Ts)
                    assert(status)
                    (status,messageRoute) = testdrone.updateRouteStatus()
                    assert(status)
                    #print("T=%f MOVE>\t %s:%s" %(itime,messageMove,messageRoute))
                for testdrone in dron_list[idept].drones_list_ground:
                    #advance step for wait drone
                    (status,messageMove) = testdrone.moveTs(Ts)
                    assert(status)
                    (status,messageRoute) = testdrone.updateRouteStatus()
                    assert(status)
                    #print("T=%f WAIT>\t %s:%s" %(itime,messageMove,messageRoute))
                    if(testdrone.status == ua.Drone.GROUND_READY):
                        assert(dron_list[idept].groundDroneLaunch())
            #---

            #update time
            itime += Ts

        except:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, exc_tb.tb_lineno)
            break

        #end, now print
        plt.text(400,1200, "Time= "+str(itime)+"/"+str(it)+" - Fly= "+str(tot_fly_cnt))
        plt.pause(1)
    #end
    cust.closeFile()

