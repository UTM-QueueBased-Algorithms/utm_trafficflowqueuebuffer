# path_generator.py
#
#  Closed path random generator, creates a new list as Google Earth:
#  ex: [(x,y), (altitude x,y), ...(altitude)]
#  First and last entry are looped so are special. In general [(x,y)], ignoring altitude
import sys
import os
import csv
import random
import math

#------------------------------------------------
dir_path_input   = "./path_input/"
dir_path_output  = "./path_output/"

# read conf_path.txt input file ----------------------
f_log = open(dir_path_output+"log.txt", "w+") #capture everything
try:
    f_conf = open(dir_path_input+"conf_path.txt","r")
    reader = csv.reader(f_conf)
    #read data
    dept_loc_type    = int( reader.__next__()[1] )
    f_log.write("depot placement type (number symbol), %d\n"%dept_loc_type)    # 1 = random along square
    print("depot placement type (number symbol) = ",dept_loc_type)             # 2 = random along circle
    #                                                                          # 3 = read from file
    if( 3 != dept_loc_type ):
        pass
    #
except Exception as e:
    print("%s" %e )
    sys.exit(1)
f_conf.close()

#create graph network with X num_paths ----------
r_dist=0.0001 #TODO: tolerance from path where location, i.e. customer or depot is located
# limit to 0 to 1 for x and y, can scale later
x_min= 0
x_max= 1
y_min= 0
y_max= 1
#TODO: modify depot location, currently located to left
#TODO: modify graph structure, currently equal grid

#create placemarkers ----------------------------
f_depot = open(dir_path_output+"depot_loc.txt", "w+")
#create depot locations
igraph = 0
Depot_Loc = []
#TODO: will need to modify this later
if( 1 == dept_loc_type ):
    for idepot in range(1,num_dept+1):
        #
        tmp_rand = random.random()
        if(   tmp_rand < 0.25 ):
            # left-side
            ix=x_min-r_dist
            iy=y_min+r_dist*idepot
        elif( tmp_rand < 0.5 ):
            # right-side
            ix=x_max+r_dist
            iy=y_min+r_dist*idepot
        elif( tmp_rand < 0.75 ):
            # top-side
            ix=x_min+r_dist*idepot
            iy=y_max+r_dist
        else:
            # bottom-side
            ix=x_min+r_dist*idepot
            iy=y_min-r_dist
        #end
        Depot_Loc.append( (idepot, ix,iy) )
        f_depot.write("Depot,%d,%f,%f\n" %(idepot,ix,iy))
elif( 2 == dept_loc_type ):
    pass
elif( 3 == dept_loc_type ):
    f_depot_data = open(dir_path_input+"depot_loc.txt", "r")
    reader = csv.reader(f_depot_data)
    for iline in reader:
        #new sub-graph?
        if( "Depot" == iline[0] ):
            idepot = int( iline[1] )
            ix     = float( iline[2] )
            iy     = float( iline[3] )
            Depot_Loc[igraph-1].append( (idepot, ix,iy) )
            f_depot.write("Depot,%d,%f,%f\n" %(idepot,ix,iy))
        else:
            #yes, new sub-graph...
            Depot_Loc.insert(igraph,[])
            f_depot.write("G%d\n" %igraph)
            igraph = igraph + 1
    f_depot_data.close()
f_depot.close()


####################
def L2dist(val):
    ix = val[0]
    iy = val[1]
    xd = val[-2]
    yd = val[-1]
    rval = math.sqrt( math.pow(ix-xd,2)+math.pow(iy-yd,2) )
    return rval

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

    f_path   = open(dir_path_output2+"paths.txt", "w+")
    f_path_s = open(dir_path_output2+"paths_s.txt", "w+")
    #crossing waypoints
    #straight line paths from depot to customer
    sPaths=[]
    for idept in Depot_Loc[igraph]:
        for icust in Depot_Loc[igraph]:
            #initial skip
            if( idept[0]==icust[0] ):
                continue #same depot
            #end init skip
            L = []
            # a x + b = c x + d           yc − yd = m (xc − xd) 
            # (a-c) x = (d-b)
            # x = (d-b)/(a-c)
            # y = a x + b
            #
            ixd = idept[1]
            iyd = idept[2]
            ixc = icust[1]
            iyc = icust[2]
            if( ixc == ixd ):
                ixc = ixc + r_dist
                ixd = ixd - r_dist
            a = (iyc-iyd)/(ixc-ixd)
            b = iyd - a*ixd
            for jdept in Depot_Loc[igraph]:
                for jcust in Depot_Loc[igraph]:
                    #initial skip
                    if( idept[0]==jdept[0] or icust[0]==jcust[0] ):
                        continue #same starting and/or end points
                    #end init skip
                    jxd = jdept[1]
                    jyd = jdept[2]
                    jxc = jcust[1]
                    jyc = jcust[2]
                    if( jxc == jxd ):
                        jxc = jxc + r_dist
                        jxd = jxd - r_dist
                    c = (jyc-jyd)/(jxc-jxd)
                    d = jyd - c*jxd
                    #post-process skip                    
                    if( a == c ):
                        continue #parallel lines (no intersect in grid)
                    #end post skip
                    ix = (d-b)/(a-c)
                    iy = a*ix + b
                    #skip pts outside of environment
                    #if( ix>(x_max+r_dist) or ix<(x_min-r_dist) or iy>(y_max+r_dist) or iy<(y_min-r_dist) ):
                    #    continue 
                    if( ix>=max([ixd,jxd,ixc,jxc])-r_dist or ix<=min([ixd,jxd,ixc,jxc])+r_dist or iy>=max([iyd,jyd,iyc,jyc])-r_dist or iy<=min([iyd,jyd,iyc,jyc])+r_dist ):
                        continue #outside of environment intersect (minus r_dist tolerance around depots)
                    if( (ixc>ixd and (ix>ixc or ix<ixd)) or (ixc<ixd and (ix<ixc or ix>ixd)) or (jxc>jxd and (ix>jxc or ix<jxd)) or (jxc<jxd and (ix<jxc or ix>jxd))):
                        continue #outside of line segments intersect
                    #end skip pts
                    L.append( (ix,iy, idept[0],jdept[0],icust[0],jcust[0], ixd,iyd) ) #add waypoint(x,y) and id
                    f_path_s.write("%f,%f,%d,%d,%d,%d\n" %(ix,iy,idept[0],jdept[0],icust[0],jcust[0]))
                #f_path_s.write("\n")
            #end
            L.sort(key=L2dist)
            sPaths.append(L)            #add straight path
            for ipt in L:
                for item in ipt:
                    f_path.write(str(item)+",")
                f_path.write(";")
            f_path.write("\n")
    #END crossing waypoint path generator
    f_path.close()
    f_path_s.close()


    num_dept = len(Depot_Loc[igraph])
    #create D(i,j) matrix based on paths ------------
    #waypoint_i to waypoint_j (depot or normal waypoint)
    f_d_matrix = open(dir_path_output2+"distance_matrix_path.csv", "w+")
    f_d_matrix.write("Number of Depots, %d\n" %num_dept)
    #write out header for matrix
    #TODO: header here...
    f_d_matrix.write("       , ")
    ispath_cnt = 0
    for idept in Depot_Loc[igraph]:
        f_d_matrix.write("Dept-%d: " %idept[0])
        for ispath in sPaths:
            f_d_matrix.write("sPath-%d, " %ispath_cnt)
            ispath_cnt += 1
            for iswaypoint in ispath:
                isx = iswaypoint[0]
                isy = iswaypoint[1]
                ixdept = idept[1]
                iydept = idept[2]
                distance = math.sqrt( math.pow(isx-ixdept,2.0)+math.pow(isy-iydept,2.0) ) #L2-norm
                f_d_matrix.write("%d, " %distance)
        f_d_matrix.write("\n")
    #END
    f_d_matrix.close()

    #create D(i,j) matrix based on L2-norm ----------
    #waypoint_i to waypoint_j (depot or normal waypoint)
    f_d_matrix = open(dir_path_output2+"distance_matrix_L2.csv", "w+")
    f_d_matrix.write("Number of Depots, %d\n" %num_dept)
    #write out header for matrix
    #TODO: header here...
    #Main Loop
    for idept in Depot_Loc[igraph]:
        f_d_matrix.write("Depot-%d ," %idept[0])
        for icust in Depot_Loc[igraph]:
            icx = icust[1]
            icy = icust[2]
            idx = idept[1]
            idy = idept[2]
            distance = math.sqrt( math.pow(icx-idx,2.0)+math.pow(icy-idy,2.0) ) #L2-norm
            f_d_matrix.write("%f," %distance)
        f_d_matrix.write("\n")
    #END
    f_d_matrix.close()

#END loop for creating sub-graph folders... 

print("DONE")
