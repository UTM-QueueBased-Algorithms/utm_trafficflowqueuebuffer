# drone.py
#
#  Module for storing drone information status.
import csv
import math
from scipy.integrate import solve_ivp   #for ode kinematics solving

#####################################################################
class Drone:
    '''
    Drone status info:
        all distance is (meter)
        all time is (sec)
    '''
    WAYPOINT_MAX_DISTANCE = 10  #min acceptable distance for "reaching" waypoint
    DRONE_ID = 0                #next free drone's info id (auto inc)

    GROUND_NO_OPS = -1  #at depot, no ops
    GROUND_WAIT = 0     #at depot, assigned ops, waiting for deploy
    GROUND_READY = 1    #at depot, assigned ops, ready to launch
    FLY_MOVE = 2        #flying mission
    FLY_WAIT = 3        #flying mission, temp pause

    def ode_fun(t, X):
        '''
        ode system solver for drone modeled as deterministic
        point-mass with constant heading and speed for t sec
            dx      = speed*cos(heading)dt
            dy      = speed*sin(heading)dt
            dheading= 0
            dspeed  = 0
        '''
        f = [   X[3]*math.cos(X[2]), 
                X[3]*math.sin(X[2]),
                0,
                0
            ]
        return f

    '''
    Specific drone methods:
    '''

    def __init__(self, route_info, service_customer, drone_size, drone_speed, x,y):
        '''
        default init method:
            Creates drone with size and speed modeled as ode_fun.
            Gives a unique DRONE_ID (upto int numbers range).
        '''
        #desired waypoints, route_info
        self.customer_id = service_customer
        self.curr_index = 0
        self.list_xw = route_info[0]
        self.list_yw = route_info[1]
        self.list_ETA = route_info[2]
        self.list_head = route_info[3]
        #service time
        self.time = 0
        self.eta = self.list_ETA[self.curr_index]
        #drone status here
        self.status = Drone.GROUND_WAIT
        #drone characteristics
        self.size = drone_size
        self.speed = drone_speed
        self.prev_speed = self.speed
        self.pause_time = 0
        #pos id
        self.heading = math.atan2( (self.list_yw[0]-y),(self.list_xw[0]-x) )
        self.x = x
        self.y = y
        #information id
        self.id = Drone.DRONE_ID
        Drone.DRONE_ID += 1

    def getCustomerId(self):
        return self.customer_id

    def getRouteInfo(self):
        return [self.list_xw[self.curr_index:], self.list_yw[self.curr_index:], self.list_ETA[self.curr_index:], self.list_head[self.curr_index:]]

    def getStatus(self):
        return self.status

    def pause(self, Ts):
        '''
        Temporary pause drone (set speed=0) for Ts seconds.
        Can be used for basic conflict resolution.
        '''
        if(Drone.GROUND_WAIT==self.status or Drone.FLY_WAIT==self.status):
            return #no need to pause, as already paused
        #end, now pause
        self.pause_time = Ts
        self.speed = 0
        #now update etas with added delay
        length = self.list_ETA.__len__()
        for icnt in range(self.curr_index, length):
            self.list_ETA[icnt] += Ts        

    def moveTs(self, Ts):
        '''
        Drone moves in heading dir for Ts seconds
        '''
        status = False
        message = ""+str(self.id)
        #TODO: 3D motion
        try:
            #now update pause time
            if( 0 < self.pause_time ):
                self.pause_time -= Ts
                message += "- paused"
            elif( 0 == self.speed ):
                self.speed = self.prev_speed
            #end
            if(Drone.GROUND_READY==self.status):
                (status,tmpmessage) = self.updateRouteStatus() #make sure it is going in correct dir
                assert(status)
                self.status = Drone.FLY_MOVE #instant transition
                message += "- launched"
            #make sure it is in moving state
            if(Drone.FLY_MOVE==self.status):
                sol = solve_ivp(Drone.ode_fun, [0,Ts], [self.x,self.y,self.heading,self.speed])
                self.x = sol.y.T[-1, 0]
                self.y = sol.y.T[-1, 1]
                self.heading = sol.y.T[-1, 2]
                self.speed = sol.y.T[-1, 3]
                #
                message += "- moving<"+str(self.x)+","+str(self.y)+">"
            else:
                message += "- waiting<"+str(self.status)+">"
            #end, update time
            self.time += Ts
            status = True
        except Exception as e:
            print("%s" %e)
        #end
        return (status,message)

    def updateRouteStatus(self):
        '''
        Not changing route info, but updating status along
        the route, i.e. changing waypoints, heading
        '''
        status = False
        message = ""
        #
        try:
            x = self.x
            y = self.y
            xw = self.list_xw[self.curr_index]
            yw = self.list_yw[self.curr_index]
            self.eta = self.list_ETA[self.curr_index]
            message += "- time<"+str(self.time)+"/"+str(self.eta)+">"
            self.heading = math.atan2( (yw-y),(xw-x) )  # update heading, since might overshoot
            
            #make sure it is not waiting on ground nor not assigned mission
            if(not Drone.GROUND_WAIT==self.status and not Drone.GROUND_NO_OPS==self.status):
                #check if need to change route waypoint based on reaching it or expiring on time
                rw = math.sqrt( math.pow(xw-x,2)+math.pow(yw-y,2) )
                if( Drone.WAYPOINT_MAX_DISTANCE >= rw or self.eta<=self.time ): #TODO:
                    self.curr_index += 1
                    message += "- nextIndex"
                #check if done with route 
                if( self.list_ETA.__len__() <= self.curr_index):
                    self.curr_index = 0
                    self.status = Drone.GROUND_NO_OPS #TODO
                    message += "- Grounded"
                #end
            #otherwise, check if it is less than set-up time
            elif( Drone.GROUND_WAIT==self.status and self.eta<=self.time ):
                self.curr_index += 1
                self.status = Drone.GROUND_READY
                message += "- startRouteIndex"
            #end
            
            status = True
        except Exception as e:
            message += "- ERROR updateRoute<"+str(e)+">" 
        #end
        return (status,message)

#####################################################################
class DroneMissionPlanner:
    '''
    Simple Drone Mission Planner: 
        creates 4d trajectory (3d space, 1d time).
        Mission is from depot->depot (depot is either warehouse or customer)
    '''

    def __init__(self):
        self.Xr=[]
        self.Yr=[]
        self.Tr=[]
        self.Hr=[]
        #
        self.G=[] #connections
        self.Path=[] #distance values to crossings
        self.Cros_Loc=[]
        self.Dept_Loc=[]
        self.Cust_Loc=[]
        #
        self.num_cust = -1
        self.num_dept = -1
        self.num_wayt = -1

    '''Initialization methods'''

    def loadGraph(self, file):
        status = False
        print("DroneMissionPlanner: load graph connection file: "+file)
        try:
            f = open(file,"r")
            reader = csv.reader(f)
            #now try reading graph file
            reader.__next__() 
            (M,N,P) = reader.__next__()
            num_cust = int( N )
            if( self.num_cust<0 ):
                self.num_cust = num_cust
            else:
                assert(self.num_cust==num_cust)
            num_dept = int( M )
            if( self.num_dept<0 ):
                self.num_dept = num_dept
            else:
                assert(self.num_dept==num_dept)
            num_wayt = int( P )
            if( self.num_wayt<0 ):
                self.num_wayt = num_wayt
            else:
                assert(self.num_wayt==num_wayt)
            #
            reader.__next__()
            for idept in range(self.num_dept):
                tmpL = reader.__next__()[1:(self.num_cust+self.num_wayt+1)]
                print("Dept-%d: %s" %(idept,tmpL))
                self.G.append( tmpL ) #ignore first col
            for iwayt in range(self.num_wayt):
                tmpL = reader.__next__()[1:(self.num_cust+self.num_wayt+1)]
                print("Wayt-%d: %s" %(iwayt,tmpL))
                self.G.append( tmpL ) #ignore first col
            #
            f.close()
            #everything went well...
            status = True
        except Exception as e:
            print("%s" %e )
        return status

    def loadPaths(self, file):
        #TODO: assume distance is in kilo-meters, convert to meters
        status = False
        print("DroneMissionPlanner: load paths file: "+file)
        try:
            f = open(file,"r")
            reader = csv.reader(f)
            #path/crossing waypoints initializations
            self.Path = []   #item=(x,y,idept,jdept,icust,jcust)
            self.Cros_Loc = []
            for line in reader:
                self.Path.append( list(line) )
                print(str(self.Path[-1]))
                ixw = float(self.Path[-1][0]) #in km
                iyw = float(self.Path[-1][1])
                flag=True
                if( 0<len(self.Cros_Loc) ):
                    for iwaypt in self.Cros_Loc:
                        x = float(iwaypt[0])/1000 #convert to km from m
                        y = float(iwaypt[1])/1000
                        ir = math.sqrt( math.pow(x-ixw,2.0)+math.pow(y-iyw,2.0) )
                        if( ir<=0.000005 ):
                            #no new point, km works better than m
                            flag=False
                if(flag):
                    self.Cros_Loc.append( (float(ixw)*1000,float(iyw)*1000) )
            #
            f.close()
            #everything went well
            status = True
        except Exception as e:
            print("%s" %e )
        return status
    
    def loadDepots(self, file):
        #TODO: assume distance is in kilo-meters, convert to meters
        status = False
        print("DroneMissionPlanner: load depots file: "+file)
        #  - depot log data (1=dept_id, 2=x, 3=y)
        with open(file,"r") as f_dept:
            reader = csv.reader(f_dept)
            for line in reader:
                self.Dept_Loc.append( (line[1],float(line[2])*1000,float(line[3])*1000) )
                print(str(self.Dept_Loc[-1]))
            status = True
        return status

    def loadCustomers(self, file: str):
        #TODO: assume distance is in kilo-meters, convert to meters
        status = False
        print("DroneMissionPlanner: load customers file: "+file)
        #  - customer log data (1=cust_id, 2=x, 3=y)
        with open(file,"r") as f_cust:
            reader = csv.reader(f_cust)
            for line in reader:
                self.Cust_Loc.append( (line[1],float(line[2])*1000,float(line[3])*1000) )
                print(str(self.Cust_Loc[-1]))
            status = True
        return status
    
    '''Create Routes'''

    def generateRoute(self, idept,icust, drone_speed,drone_setup_time) -> list:
        route = (-1,-1,-1,-1)
        try:
            assert( 0 < self.Path.__len__() ) #check if loaded paths file
            assert( 0 < self.Dept_Loc.__len__() )
            assert( 0 < self.Cust_Loc.__len__() )
            #initializations
            Xr=[]
            Yr=[]
            Tr=[]
            Hr=[]
            #start, mission planner
            Xr.append(self.Dept_Loc[idept][1])
            Yr.append(self.Dept_Loc[idept][2])
            Tr.append(drone_setup_time)
            xt = self.Cust_Loc[icust][1]
            yt = self.Cust_Loc[icust][2]
            head = math.atan2(yt-Yr[0],xt-Xr[0])
            Hr.append(head)
            #intermediate pts forward
            for path in self.Path:   #x,y,id,jd,ic,jc
                ix = path[0]
                iy = path[1]
                if(path[2] == idept):
                    idistance = math.sqrt( math.pow(ix-Xr[-1],2.0)+math.pow(iy-Yr[-1],2.0) )
                    iservice_time = Tr[-1] + idistance/drone_speed
                    head = math.atan2(yt-Yr[-1],xt-Xr[-1])
                    Xr.append(ix)
                    Yr.append(iy)
                    Tr.append(iservice_time)
                    Hr.append(head)
            #end intermediate pts
            idistance = math.sqrt( math.pow(xt-Xr[-1],2.0)+math.pow(yt-Yr[-1],2.0) )
            iservice_time = Tr[-1] + idistance/drone_speed
            head = math.atan2(yt-Yr[-1],xt-Xr[-1])
            Xr.append(xt)
            Yr.append(yt)
            Tr.append(iservice_time)
            Hr.append(head)
            route = [
                Xr, #list_xw
                Yr, #list_yw
                Tr, #list_ETA
                Hr  #list_head
            ]
            #end, mission planner
        except Exception as e:
            print("%s" %e )
        #return completed route or [-1,-1,-1,-1]
        return route
    
    '''Return/Wrapper Methods'''

    def getCrossings(self):
        return self.Cros_Loc

#####################################################################
# TESTING MODULE
#################
if __name__ == "__main__":
    #  Uses mathplotlib. `python -m pip install -U matplotlib`
    import matplotlib.pyplot as plt             #main plotting
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = 'Times New Roman'
    fig, ax = plt.subplots()

    #--- test mission planner
    graphFile = "sim_input/_test/graph_matrix.csv"
    testGraph = DroneMissionPlanner()
    assert( testGraph.loadGraph(graphFile) )
    #
    pathsFile = "sim_input/_test/paths_s.txt"
    assert( testGraph.loadPaths(pathsFile) )
    print("Crossing Pts <"+str(testGraph.Cros_Loc)+">" )
    #exit()
    #
    depotLocFile = "sim_input/_test/depot_loc.txt"
    assert( testGraph.loadDepots(depotLocFile) )
    custLocFile = "sim_input/_test/customer_loc.txt"
    assert( testGraph.loadCustomers(custLocFile) )
    idept = 1
    icust = 1
    speed = 20  #m/s
    setup = 10 #sec
    testroute = testGraph.generateRoute(idept,icust,speed,setup)
    print("Route <"+str(testroute)+">" )
    #--- end test mission planner

    #setup initializations
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
    #
    h = 150

    #--- test drone 
    size = 1 #meters
    x = testroute[0][0]
    y = testroute[1][0]
    print("Drone <"+str(x)+","+str(y)+">")
    Ts = 5 #sec, update rate
    testdrone = Drone(testroute,icust,size,speed,x,y)
    cnt = 0
    cnt_tot = 20
    while( cnt<cnt_tot and (not Drone.GROUND_NO_OPS==testdrone.getStatus()) ):
        #start loop iter
        print("===Cnt[%d/%d]===" %(cnt,cnt_tot))

        messageMove = ""
        messageRoute = ""
        #init print
        plt.cla()
        plt.axis([-h,1000+h,-h,1000+h])
        # 2D top-view of simulation map
        plt.scatter(xdata_dept_list,ydata_dept_list, c='orange',s=80, edgecolors='none', label='depot')
        plt.scatter(xdata_cust_list,ydata_cust_list, c='blue',s=20, edgecolors='none', label='customer')
        #end, init print

        try:
            (status,messageMove) = testdrone.moveTs(Ts)
            assert(status)
            (status,messageRoute) = testdrone.updateRouteStatus()
            assert(status)
            x = testdrone.x
            y = testdrone.y
        except Exception as e:
            print("ERROR DroneTest<%s>" %e )

        #end, now print
        message = messageMove + ":" + messageRoute
        print(message)
        plt.scatter(x,y, c='purple',s=20, edgecolors='none', label='UAV')
        plt.pause(1)

        #update loop iter
        cnt += 1
    #--- end test drone

    #plt.legend()
    #plt.grid(True)
    #plt.show()
