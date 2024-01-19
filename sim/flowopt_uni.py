# flowopt.py
#
#  Solves convex optimization graph flow problem for drone package
#  as a circulation problem (network flow with flow limits optimization)
import cvxpy as cp
import math
#
import csv
import sys
import os

#------------------------------------------------
dir_sim_input   = "./sim_input/"
dir_sim_output  = "./sim_output/"
test = False

# read conf.txt input file ----------------------
f_log = open(dir_sim_input+"log_flow.txt", "w+") #capture everything
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
    #
    #------------------------
    #
except Exception as e:
    print("%s" %e )
    sys.exit(1)
f_conf.close()
f_log.close() #save individual log per igraph below

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
        os.mkdir(dir_sim_input2)
        print("Directory <"+dir_sim_input2+"> created")
    except:
        print("Directory <"+dir_sim_input2+"> exists")
    #---
    f_log_G = open(dir_sim_input2+"log_flow_G.txt", "w+") #capture everything for this igraph

    #---
    P = 0
    theta = []
    visited_waypoints = []
    with open(dir_subgraph+"paths_s.txt","r") as f_path: #(ix,iy, idept,jdept,icust,jcust)
        reader = csv.reader(f_path)
        cwaypoint = 0
        for line in reader:
            tx = float(line[0])
            ty = float(line[1])
            idept = int(line[2])
            jdept = int(line[3])
            icust = int(line[4])
            jcust = int(line[5])
            #
            saw_flag = False
            for ipt in visited_waypoints:
                if( ipt[1]==tx and ipt[2]==ty ):
                    #previous visited waypoint
                    saw_flag = True
                if(saw_flag):
                    break
            if(not saw_flag):
                cwaypoint += 1 #new waypoint
                visited_waypoints.append( (cwaypoint,tx,ty) )
                #
                ix = Dept_Loc[igraph][idept][1]
                iy = Dept_Loc[igraph][idept][2]
                ihead = math.atan2(ty-iy,tx-ix)
                jx = Dept_Loc[igraph][jdept][1]
                jy = Dept_Loc[igraph][jdept][2]
                jhead = math.atan2(ty-jy,tx-jx)
                #
                theta.append(abs(jhead-ihead))
                P += 1 #new crossing
            #end
    for ipt in visited_waypoints:
        print(ipt)

    # Create problem setup --------------------------
    customer_rate = poisson_client_arr_rate #orders per time, i.e. lambda
    N = len(Dept_Loc[igraph]) #number of depots
    M = len(Dept_Loc[igraph]) #number of customers (depots)
    #P = #number of crossings (found above)
    f_log_G.write("Depots = %d, Waypoints = %d\n" %(N,P))
    print("Depots = %d, Waypoints = %d\n" %(N,P))

    #---
    G_constraint = []
    for irow in range(N+P):
        L = []
        for icol in range(M+P):
            #assume no connection
            L.append(0)
        G_constraint.append(L)
    for irow in range(N):
        for icol in range(M):
            #assume depot to customer connection
            G_constraint[irow][icol] = 1
    #---
    with open(dir_subgraph+"paths.txt","r") as f_path: 
        reader = csv.reader(f_path,delimiter=";")
        route_num = 1 # "1" is special depot to customer, "0" is no connection
        for line in reader:
            route_num += 1 #skip "1"
            path_list = []
            for ipt in line:
                ipt_list = ipt.split(",") #splits (ix,iy, idept,jdept,icust,jcust)
                if( ipt_list[0]=="" ):
                    break
                path_list.append( (float(ipt_list[0]),float(ipt_list[1]),
                            int(ipt_list[2]),int(ipt_list[3]),int(ipt_list[4]),int(ipt_list[5])) )
            #check if path exists
            if( len(path_list)==0 ):
                continue
            #start and end waypoints
            x1 = path_list[0][0]
            y1 = path_list[0][1]
            for iwaypt in visited_waypoints:
                if( abs(x1-iwaypt[1])<0.00001 and abs(y1-iwaypt[2])<0.00001 ):
                    w1 = iwaypt[0]
                    break
            x2 = path_list[-1][0]
            y2 = path_list[-1][1]
            for iwaypt in visited_waypoints:
                if( abs(x2-iwaypt[1])<0.00001 and abs(y2-iwaypt[2])<0.00001 ):
                    w2 = iwaypt[0]
                    break
            #
            idept = path_list[0][2] #depot count starts 1
            icust = path_list[0][4] #customer count starts 0
            #                       #waypoint count starts 1
            
            #build graph connection matrix ---
            G_constraint[idept-1][icust] = 0 #waypoint in between, so no direct connection
            #going away depot
            G_constraint[N+w2-1][icust] = route_num
            G_constraint[idept-1][M+w1-1] = route_num
            pw = w1
            #
            for ipt in path_list: #item=(ix,iy, idept,jdept,icust,jcust)
                cwx = ipt[0]
                cwy = ipt[1]
                for iwaypt in visited_waypoints:
                    if( abs(cwx-iwaypt[1])<0.00001 and abs(cwy-iwaypt[2])<0.00001 ):
                        cw = iwaypt[0]
                        break
                if( cw!=pw ): #no self-connections
                    G_constraint[N+pw-1][M+cw-1] = route_num #previous waypoint to waypoint
                pw = cw
    #---
    for irow in range(N+P+1):
        for icol in range(M+P+1):
            if(irow==0 and icol==0):
                f_log_G.write('  ')
                print(' ',end=' ') #dummy space (formating)
            #start header
            elif(irow==0 and icol>0):
                if(icol<M+1):
                    f_log_G.write('c ')
                    print("d",end=' ')
                else:
                    f_log_G.write('w ')
                    print("w",end=' ')
            elif(irow>0 and icol==0):
                if(irow<N+1):
                    f_log_G.write('d ')
                    print("d",end=' ')
                else:
                    f_log_G.write('w ')
                    print("w",end=' ')
            #end header
            elif(irow>0 and icol>0):
                f_log_G.write(str(G_constraint[irow-1][icol-1])+' ')
                print(G_constraint[irow-1][icol-1],end=' ') #connections
        f_log_G.write("\n")
        print()
    #exit(0)

    #crossing waypoint constraints
    #np.random.seed(1)
    V = nominal_gnd_spd_m_sec #speed
    L = TBOV_length*drone_ideal_length #TBOV length
    W = TBOV_width*drone_ideal_width #TBOV width

    # Define and solve CVXPY problem ----------------
    G = cp.Variable((N+P, M+P)) #uses numpy array, 
                                #so index is [irow,icol] NOT [irow][icol]

    objective = cp.Maximize(cp.sum(G)) 
    constraints = [ cp.sum(G[0:N-1,:]) >= 0 ] #dummy since other constraints are added to this one...
    #crossing waypoint constraints
    for i in range(P):
        constraints += [ cp.sum(G[:,M+i]) == cp.sum(G[N+i,:]) ] #input-output flow conservation
    for i in range(P):
        theta_t = theta[i]
        theta_a = (math.pi-theta_t)/2
        if( theta_t>math.pi/2 ):
            delta_t = 2*(L/V) + max( W*(1-math.cos(theta_t))/(2*V*math.sin(theta_t)) , W/(2*V*math.tan(theta_a)) )
        else:
            delta_t = max( 2*(L/V)+W*math.sin(theta_t)/(2*V) , 2*(L/V)-W/(2*V*math.tan(theta_a)) )
        constraints += [ cp.sum(G[:,M+i]) <= 1/(delta_t+(L/V)) ] #crossing angle flow limits
    #end crossing
    #connection constraints
    for irow in range(N+P):
        for icol in range(M+P):
            if(G_constraint[irow][icol] == 0):
                constraints += [ G[irow,icol] == 0 ]
            else:
                if(irow<N and icol<M):
                    #depot to customer connection
                    constraints += [ G[irow,icol] == V/L ]
                else:
                    #waypoint connections
                    #max/min flow constraints
                    constraints += [ G[irow,icol] <= V/L ]
                    constraints += [ G[irow,icol] >= 0 ] #make sure it is positive flow
                #route connections
                for jrow in range(N+P):
                    for jcol in range(M+P):
                        if( G_constraint[irow][icol] == G_constraint[jrow][jcol] ): #same route so same flow
                            constraints += [ G[irow,icol] == G[jrow,jcol] ]

    #end connections
    prob = cp.Problem(objective,constraints)
    prob.solve(solver='SCS',verbose=False) #['CVXOPT', 'ECOS', 'ECOS_BB', 'GLPK', 'GLPK_MI', 'OSQP', 'SCIPY', 'SCS']
                                            #only 'SCS' and 'SCIPY' work

    # Print result ----------------------------------
    f_log_G.write("The optimal value is %f\n" %prob.value)
    print("The optimal value is", prob.value)
    f_log_G.write("A solution G is\n%s\n" %str(G.value))
    print("A solution G is")
    print(G.value)
    f_log_G.write("A solution for direct input rates of G is\n")
    print("A solution for direct input rates of G is")
    for idept in range(N):
        for icust in range(M):
            f_log_G.write("Depot %d to Depot %d is %f\n" %(idept,icust,G[idept,icust].value))
            print("Depot "+str(idept+1)+" to Customer "+str(icust)+" is",G[idept,icust].value)
        for iwayp in range(P):
            f_log_G.write("Depot %d to CrossPts %d is %f\n" %(idept,iwayp,G[idept,M+iwayp].value))
            print("Depot "+str(idept+1)+" to CrossPts "+str(iwayp)+" is",G[idept,M+iwayp].value)

    f_log_G.write("Solution rates from depot to depot routes\n")
    print("\nSolution rates from depot to depot routes")
    #print initial flow connections
    for irow in range(N):
        for icol in range(M+P):
            iroute_id = G_constraint[irow][icol]
            #depot-customer direct connections
            if( 1 == iroute_id ):
                val = math.ceil(1/G[irow,icol].value)
                idelta = -1
                f_log_G.write("%d,%d,%d,%s\n" %(irow+1,icol,val,idelta))
                print("[Direct] Depot "+str(irow)+" to Depot "+str(icol)+" is",val)
            #depot-customer indirect conncetions (via crossing waypoints)
            elif( 1 < iroute_id ):
                for jrow in range(N,N+P):
                    for jcol in range(M):
                        jroute_id = G_constraint[jrow][jcol]
                        if( jroute_id == iroute_id ):
                            val = math.ceil(1/G[jrow,jcol].value)
                            idelta = -1
                            #find worse angle so worse delta to satisfy
                            for iway_r in range(N,N+P):
                                for iway_c in range(M,M+P):
                                    if( jroute_id == G_constraint[iway_r][iway_c] ):
                                        theta_t = theta[iway_r-N]
                                        theta_a = (math.pi-theta_t)/2
                                        if( theta_t>math.pi/2 ):
                                            delta_t = 2*(L/V) + max( W*(1-math.cos(theta_t))/(2*V*math.sin(theta_t)) , W/(2*V*math.tan(theta_a)) )
                                        else:
                                            delta_t = max( 2*(L/V)+W*math.sin(theta_t)/(2*V) , 2*(L/V)-W/(2*V*math.tan(theta_a)) )
                                        if( delta_t > idelta ):
                                            idelta = delta_t
                            #endfind
                            if( -1 == idelta ):
                                theta_t = theta[jrow-N]
                                theta_a = (math.pi-theta_t)/2
                                if( theta_t>math.pi/2 ):
                                    delta_t = 2*(L/V) + max( W*(1-math.cos(theta_t))/(2*V*math.sin(theta_t)) , W/(2*V*math.tan(theta_a)) )
                                else:
                                    delta_t = max( 2*(L/V)+W*math.sin(theta_t)/(2*V) , 2*(L/V)-W/(2*V*math.tan(theta_a)) )
                                idelta = delta_t
                            #endcheck
                            f_log_G.write("%d,%d,%d,%s\n" %(irow+1,jcol,val,idelta))
                            print("[Indirect] Depot "+str(irow)+" to Depot "+str(jcol)+" is",val)
    f_log_G.close()
    print("DONE with igraph[%d]\n" %igraph)

#------------------------------------------------

print("DONE")
