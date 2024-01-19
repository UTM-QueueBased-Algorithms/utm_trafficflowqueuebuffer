# graph_connection.py
#
#  Extracts adjaceny matrix for graph connections from 'main__path_generator.py'
import csv
import sys
import os
import math

#------------------------------------------------
dir_path_input   = "./path_input/"
dir_path_output  = "./path_output/"

# read conf_path.txt input file ----------------------
try:
    f_conf = open(dir_path_input+"conf_path.txt","r")
    reader = csv.reader(f_conf)
    #read data
    dept_loc_type    = int( reader.__next__()[1] )                     # 1 = random along square
    print("depot placement type (number symbol) = ",dept_loc_type)     # 2 = random along circle
    #                                                                  # 3 = read from file
    if( 3 != dept_loc_type ):
        pass
    #
except Exception as e:
    print("%s" %e )
    sys.exit(1)

# construct depot locations ---------------------
Dept_Loc = []   #item=(dept_id,x,y)
igraph = 0
#TODO: assume distance is in kilo-meters, convert to meters
#  - depot log data (1=dept_id, 2=x, 3=y)
with open(dir_path_output+"depot_loc.txt","r") as f_dept:
    reader = csv.reader(f_dept)
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


#run in loop for number of igraph
num_graphs = igraph
for igraph in range(num_graphs):
    subgraph_path_output = "G"+str(igraph)+"/"
    dir_path_output2 = dir_path_output + subgraph_path_output
    try:
        os.mkdir(dir_path_output2)
        print("Directory <"+dir_path_output2+"> created")
    except:
        print("Directory <"+dir_path_output2+"> exists")

    # find crossings then construct graph -----------
    P = 0
    theta = []
    visited_waypoints = []
    with open(dir_path_output2+"paths_s.txt","r") as f_path: #(ix,iy, idept,jdept,icust,jcust)
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
                ix = Dept_Loc[igraph][idept-1][1]
                iy = Dept_Loc[igraph][idept-1][2]
                ihead = math.atan2(ty-iy,tx-ix)
                jx = Dept_Loc[igraph][jdept-1][1]
                jy = Dept_Loc[igraph][jdept-1][2]
                jhead = math.atan2(ty-jy,tx-jx)
                #
                theta.append(abs(jhead-ihead))
                P += 1 #new crossing
            #end
    for ipt in visited_waypoints:
        print(ipt)

    # create graph connection matrix ----------------
    N = len(Dept_Loc[igraph]) #number of depots
    #P = ... #defined above
    f_graph = open(dir_path_output2+"graph_matrix.csv", "w") #graph output
    f_graph.write("Depots = %d, Waypoints = %d\n" %(N,P))
    f_graph.write("%d,%d,%d\n" %(N,N,P))
    print("Depots = %d, Waypoints = %d\n" %(N,P))

    G_constraint = []
    for irow in range(N+P):
        L = []
        for icol in range(N+P):
            #assume no connection
            L.append(0)
        G_constraint.append(L)
    for irow in range(N):
        for icol in range(N):
            #assume depot to depot connection
            G_constraint[irow][icol] = 1
    #---
    with open(dir_path_output2+"paths.txt","r") as f_path: 
        reader = csv.reader(f_path,delimiter=";")
        route_num = 1 # "1" is special depot to depot, "0" is no connection
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
            idept = path_list[0][2] #depot count starts 0
            icust = path_list[0][4] #customer count starts 0
            #                       #waypoint count starts 1
            
            #build graph connection matrix ---
            G_constraint[idept][icust] = 0 #waypoint in between, so no direct connection
            #going away depot
            G_constraint[N+w2-1][icust] = route_num
            G_constraint[idept][N+w1-1] = route_num
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
                    G_constraint[N+pw-1][N+cw-1] = route_num #previous waypoint to waypoint
                pw = cw
    #---
    for irow in range(N+P+1):
        for icol in range(N+P+1):
            if(irow==0 and icol==0):
                f_graph.write(' ,')
                print(' ',end=' ') #dummy space (formating)
            #start header
            elif(irow==0 and icol>0):
                if(icol<N+1):
                    f_graph.write('d,')
                    print("d",end=' ')
                else:
                    f_graph.write('w,')
                    print("w",end=' ')
            elif(irow>0 and icol==0):
                if(irow<N+1):
                    f_graph.write('d,')
                    print("d",end=' ')
                else:
                    f_graph.write('w,')
                    print("w",end=' ')
            #end header
            elif(irow>0 and icol>0):
                f_graph.write(str(G_constraint[irow-1][icol-1])+',')
                print(G_constraint[irow-1][icol-1],end=' ') #connections
        f_graph.write("\n")
        print()
    #exit(0)