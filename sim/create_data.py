# create_data.py
import sys
import os
import csv
import math
import customer_time

#------------------------------------------------
num_cust_requests = 1000 #divided among all the sub-graphs (weighted by number of depots per sub-graph)

seed = 12345
dir_sim_input   = "./sim_input/"
dir_sim_output  = "./sim_output/"

# read conf.txt input file ----------------------
try:
    f_conf = open(dir_sim_input+"conf.txt","r")
    reader = csv.reader(f_conf)
    #read data
    dir_graph_output       = reader.__next__()[1].replace(" ","") #remove whitespace
    print("graph output folder = ",dir_graph_output)
    reader.__next__()
    #------------------------
    #
    reader.__next__() #skip speed
    #
    reader.__next__() #skip setup time
    #
    poisson_client_arr_rate = float( reader.__next__()[1] )
    print("customer request parameter (orders/hr) = ",poisson_client_arr_rate)
    #convert to (orders/sec) for use in sim
    poisson_client_arr_rate = poisson_client_arr_rate / (60*60)
    #
    reader.__next__() #skip waypoint clearance
    reader.__next__()
    #------------------------
    #
except Exception as e:
    print("%s" %e )
    sys.exit(1)


f_depot_data = open(dir_graph_output+"depot_loc.txt", "r")
reader = csv.reader(f_depot_data)
num_graph=0
num_tot_dept=0
for iline in reader:
    #new sub-graph?
    if( "Depot" == iline[0] ):
        #no, increment number of total depots
        num_tot_dept = num_tot_dept + 1
    else:
        #yes, new sub-graph...
        num_graph = num_graph + 1
f_depot_data.close()

#run through all sub-graphs ---------------------
for igraph in range(num_graph):
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
    datafile = dir_sim_input2 + "customer_requests.txt"

    #distance matrix initialization
    f_distance_matrix = dir_subgraph + "distance_matrix_L2.csv"
    try:
        f_d = open(f_distance_matrix,"r")
        reader = csv.reader(f_d)
        num_dept = int( reader.__next__()[1] )
        #skip reading D(i,j)... only need above
    except Exception as e:
        print("%s" %e )
        sys.exit(1)
    f_d.close()

    # write data ------------------------------------
    num_cust_requests_igraph = math.floor(num_dept/num_tot_dept*num_cust_requests) #divide requests based on size of graph
    print("Number of requests[%d] for igraph[%d]" %(num_cust_requests_igraph,igraph))
    cust_write = customer_time.CustomerSimTime( (poisson_client_arr_rate) ,num_dept,num_dept, seed, datafile, False) 
    for icnt in range(num_cust_requests_igraph):
        cust_write.writeNextTime()
    cust_write.closeFile()

    # read data -------------------------------------
    print("\n\t ids \t time ")
    cust_read =  customer_time.CustomerSimTime( (poisson_client_arr_rate) ,num_dept,num_dept, seed, datafile, True) 
    for icnt in range(num_cust_requests_igraph):
        (icnt, ideptid,icustid,it) = cust_read.readNextTime()
        print("%d \t(%d %d \t %f)" %(icnt, ideptid,icustid,it))
    print("")
    cust_read.closeFile()

    print("DONE with igraph[%d] at <%s>" %(igraph,dir_subgraph))
#end run through all sub-graph ------------------

print("DONE")
