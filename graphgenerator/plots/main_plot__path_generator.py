# plot__path_generator.py
#
#  plots waypoint locations, i.e. depots, customers, and general waypoints.
#  Also displays other information such as distance and potential paths.
import matplotlib.pyplot as plt
import csv

#------------------------------------------------
dir_plot_input   = "../path_output/"
dir_plot_output  = dir_plot_input

# construct depot locations ---------------------
Dept_Loc = []   #item=(dept_id,x,y)
igraph = 0
#TODO: assume distance is in kilo-meters, convert to meters
#  - depot log data (1=dept_id, 2=x, 3=y)
with open(dir_plot_output+"depot_loc.txt","r") as f_dept:
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

Way_Loc_All = []
#run in loop for number of igraph
num_graphs = igraph
for igraph in range(num_graphs):
    subgraph_plot_output = "G"+str(igraph)+"/"
    dir_plot_output2 = dir_plot_output + subgraph_plot_output
    #init some data
    Way_Loc  = [] #item=(x,y)
    Distance_Radial_L2 = [] #item=(L2-norm between dept-cust)

    # expected data format
    #  - waypoint crossing log data (0=x, 1=y, others...)
    with open(dir_plot_output2+"paths_s.txt","r") as f_waypt:
        reader = csv.reader(f_waypt)
        for line in reader:
            ix = float(line[0])
            iy = float(line[1])
            #add only unique ones
            flag_same = False
            if( len(Way_Loc)>0 ):
                for iwaypt in Way_Loc:
                    if( abs(iwaypt[0]-ix)<0.00001 and abs(iwaypt[1]-iy)<0.00001 ):
                        #same pt, skip
                        flag_same = True
                        break
            if(flag_same):
                continue
            Way_Loc.append( (ix,iy) )
            Way_Loc_All.append( (ix,iy) )
    #  - distance_matrix_L2 data
    with open(dir_plot_output2+"distance_matrix_L2.csv","r") as f_matrix:
        reader = csv.reader(f_matrix)
        #ignore first line
        reader.__next__()
        #ignore first column
        for line in reader:
            Distance_Radial_L2.append( (line[1:]) )

    #------------------------------------------------
    #BUG in pyplot: make sure to plot numeric values only!
    num_bins = 20 #for histogram plots, can change

    # 2D top-view of simulation map
    fig = plt.figure(num_graphs+1)
    ax = fig.add_subplot(111) #dummy placeholder, used to get "ax"
    plt.cla()
    # Distance TBOVs (straight lines in 2D)
    for idept in Dept_Loc[igraph]:
        for icust in Dept_Loc[igraph]:
            plt.plot( (idept[1],icust[1]),(idept[2],icust[2]), 'k-',alpha=0.25 )

    # Depot locations data
    xdata_dept_list = []
    ydata_dept_list = []
    cnt=0
    for idata in Dept_Loc[igraph]:
        cnt += 1
        ix = idata[1]
        iy = idata[2]
        plt.text(ix,iy,"d"+str(cnt))
        xdata_dept_list.append( ix )
        ydata_dept_list.append( iy )
    plt.scatter(xdata_dept_list,ydata_dept_list, c='orange',s=80, edgecolors='none', label='depot')
    # Waypoint locations data
    xdata_way_list = []
    ydata_way_list = []
    cnt=0
    for idata in Way_Loc:
        cnt += 1
        ix = idata[0]
        iy = idata[1]
        plt.text(ix,iy,"w"+str(cnt))
        xdata_way_list.append( idata[0] )
        ydata_way_list.append( idata[1] )
    plt.scatter(xdata_way_list,ydata_way_list, c='yellow',s=5, edgecolors='black', label='crossing')
    #
    plt.legend()
    plt.grid(True, linewidth=0.25)
    ax.set_aspect('equal', adjustable='box')
    #plt.axis([-0.1,1.1, -0.1,1.1])
    #plt.xlabel("meters")
    #plt.ylabel("meters")
    plt.title("Simulation Map for igraph="+str(igraph))
    plt.savefig(dir_plot_output2+'Figure1.png')
    print("Saved Figure for igraph=%d" %igraph)

# create plot of all depots and waypoints
#------------------------------------------------
# 2D top-view of simulation map
fig = plt.figure(igraph)
ax = fig.add_subplot(111) #dummy placeholder, used to get "ax"
plt.cla()

cnt=0
for igraph in range(num_graphs):
    # Distance TBOVs (straight lines in 2D)
    for idept in Dept_Loc[igraph]:
        for icust in Dept_Loc[igraph]:
            plt.plot( (idept[1],icust[1]),(idept[2],icust[2]), 'k-',alpha=0.25 )

    # Depot locations data
    xdata_dept_list = []
    ydata_dept_list = []
    for idata in Dept_Loc[igraph]:
        cnt += 1
        ix = idata[1]
        iy = idata[2]
        plt.text(ix,iy,"d"+str(cnt))
        xdata_dept_list.append( ix )
        ydata_dept_list.append( iy )
    if( igraph==num_graphs-1 ):
        plt.scatter(xdata_dept_list,ydata_dept_list, c='orange',s=80, edgecolors='none', label='depot')
    else:
        plt.scatter(xdata_dept_list,ydata_dept_list, c='orange',s=80, edgecolors='none')
#END all igraphs

# All Waypoint locations data
xdata_way_list = []
ydata_way_list = []
cnt=0
for idata in Way_Loc_All:
    cnt += 1
    ix = idata[0]
    iy = idata[1]
    plt.text(ix,iy,"w"+str(cnt))
    xdata_way_list.append( idata[0] )
    ydata_way_list.append( idata[1] )
plt.scatter(xdata_way_list,ydata_way_list, c='yellow',s=5, edgecolors='black', label='crossing')
#
plt.legend()
plt.grid(True, linewidth=0.25)
ax.set_aspect('equal', adjustable='box')
#plt.axis([-0.1,1.1, -0.1,1.1])
#plt.xlabel("meters")
#plt.ylabel("meters")
plt.title("Full Simulation Map")
plt.savefig(dir_plot_output+'Figure_All.png')
print("Saved Figure for all graphs")

