# main_plot.py
#
#  Data plots from output main.py, i.e. reading log.txt
#  Uses mathplotlib. `python -m pip install -U matplotlib`
import matplotlib.pyplot as plt #main plotting
import statistics as stat
import csv

#------------------------------------------------
dir_sim_input   = "../sim_input/"
dir_sim_output  = "../sim_output/"

# READ log.txt ----------------------------------
# data initializations
cust_list = [] #item=(cust_id,         cust_service_time,    sim_time,cust_call_time)
dron_list = [] #item=(cust_id,dept_id, eta_dron_service_time,sim_time, drone_id)
dron_done_list = [] #item=(            sta_dron_service_time,          drone_id)

delay_list = [] #item=(cust_id,dron_service_time-cust_service_time)
queue_size_list = [] #item=(sim_time,          service_queue_size)
drone_size_list = [] #item=(sim_time, dept_id, free_drones)
drone_done_list = [] #item=(sim_time, dept_id,)

f_log_plot = open(dir_sim_output+"log_plot.txt", "w+") #capture everything
# expected data format 
#  - customer log data       (1=sim_time, 2=cust_service_time, 3=cust_id, 4=service_queue_size, 5=cust_service_call_time)
#  - drone Start log data    (1=sim_time, 2=x, 3=y, 4=heading, 5=cust_id, 6=dept_id, 7=eta, 8=free_drones, 9=id)
#  - drone End   log data    (1=sim_time, 2=dept_id, 3=sta, 4=drone_id)
#  - drone FLY   log data    (1=sim_time, 2=x, 3=y, 4=heading, 5=id)
#  - drone Conflict  data    (1=sim_time, 2=id)
with open(dir_sim_input+"log.txt","r") as f_log:
    reader = csv.reader(f_log)
    for line in reader:
        if(line[0] == "Customer"):
            cust_list.append( (int(line[3]), float(line[2]), float(line[1]), float(line[5]),int(line[6])) )
            queue_size_list.append( (float(line[1]), int(line[4])) )
        elif(line[0] == "Drone Start"):
            dron_list.append( (int(line[5]), int(line[6]), float(line[7]), float(line[1]), int(line[9]),int(line[10])) )
            drone_size_list.append( (float(line[1]), int(line[6]), int(line[8])) )
        elif(line[0] == "Drone End"):
            drone_done_list.append( (float(line[3]),int(line[4]),int(line[5])) )
        #elif(line[0] == "Drone FLY"):
        #    dron_fly_list.append( (float(line[1]), float(line[2]), float(line[3]), float(line[4]), int(line[5])) )
        #elif(line[0] == "Drone Conflict"):
        #    dron_conflict_list.append( ((float(line[1]), int(line[2]))) )
        else:
            f_log_plot.write(str(line)+"\n")
            #special values
            if(line[0] == "Number of Customers"):
                num_cust = int(line[1])
            elif(line[0] == "Number of Depots"):
                num_dept = int(line[1])
            elif(line[0] == "Total number of time steps"):
                tot_icnt = int(line[1])
            elif(line[0] == "Time step (sec)"):
                Ts = float(line[1])
            elif(line[0] == "ideal drone amount per depot"):
                max_drones_per_depot = int(line[1])
print("Num of customer requests =",len(cust_list))
print("Num of drone launches    =",len(dron_list))
print("Num of drone finishes    =",len(drone_done_list))

#populate delay data        #item=(cust_id,dron_service_time-cust_service_time)
# expected to be in chronological order, 
# i.e. first drone services first matched customer
cp_dron_list = dron_list.copy()
cp_cust_list = cust_list.copy()
for idrone in cp_dron_list:    #item=(cust_id,dept_id, dron_service_time, sim_time,drone_id)
    for icust in cp_cust_list: #item=(cust_id,         cust_service_time, sim_time,cust_call_time)
        #same customer id?
        if( idrone[0]==icust[0] and idrone[-1]==icust[-1]):
            delay_list.append( (idrone[0],idrone[3]-icust[2],idrone[4]) )
            #cp_dron_list.remove(idrone) #counted instance now remove
            cp_cust_list.remove(icust)
            break

#------------------------------------------------
num_bins = 20 #for histogram plots, can change

icnt_plot = 0 #don't change
icnt_plot += 1
# Customer requests histogram(removes time tick)/ timeplot(include time tick)
plt.figure()
data_list = []
for idata in cust_list: #item=(cust_id, cust_service_time, sim_time,cust_call_time)
    data_list.append( idata[1]-idata[3] )
plt.hist(data_list, num_bins)
plt.title("Customer request histogram")
plt.xlabel("Time (seconds)")
plt.ylabel("Count")
plt.savefig(dir_sim_output+'Figure'+str(icnt_plot)+'.png')
f_log_plot.write("\nSaved output to Figure"+str(icnt_plot)+" and "+str(icnt_plot+1)+": Customer request time\n")
f_log_plot.write("Max requesttime sec = %d\n" %(max(data_list)))
f_log_plot.write("Min requesttime sec = %d\n" %(min(data_list)))
f_log_plot.write("Avg requesttime sec = %.2f\n" %(stat.mean(data_list)))
plt.close()
#goes with previous one
icnt_plot += 1
plt.figure()
data_list = []
for idata in cust_list: #check: should be linear increasing
    data_list.append( idata[1] )
plt.plot(data_list)
plt.title("Customer request (check) timeplot")
plt.xlabel("Simulation time (seconds)")
plt.ylabel("Total Count")
plt.savefig(dir_sim_output+'Figure'+str(icnt_plot)+'.png')
plt.close()

icnt_plot += 1
# Drone delivery histogram
plt.figure()
data_list = []
data_list_eta = [] #save for later analysis
for idata in dron_list: #item=(cust_id,dept_id, eta_dron_service_time,sim_time,id)
    data_list.append( idata[2] )
    data_list_eta.append( (data_list[-1],idata[4]) )
plt.hist(data_list, num_bins)
plt.title("Drone expected (eta) servicetime histogram")
plt.xlabel("Flight time (seconds)")
plt.ylabel("Count")
plt.savefig(dir_sim_output+'Figure'+str(icnt_plot)+'.png')
f_log_plot.write("\nSaved output to Figure"+str(icnt_plot)+": Drone expected service time\n")
f_log_plot.write("Max servicetime sec = %d\n" %(max(data_list)))
f_log_plot.write("Min servicetime sec = %d\n" %(min(data_list)))
f_log_plot.write("Avg servicetime sec = %.2f\n" %(stat.mean(data_list)))
f_log_plot.write("Std servicetime sec = %.2f\n" %(stat.stdev(data_list)))
stat_list = stat.quantiles(data_list,n=4)
f_log_plot.write("Q1 servicetime sec = %.2f\n" %(stat_list[0]))
f_log_plot.write("Q2 servicetime sec = %.2f\n" %(stat_list[1]))
f_log_plot.write("Q3 servicetime sec = %.2f\n" %(stat_list[2]))
plt.close()

icnt_plot += 1
plt.figure()
data_list = []
data_list_sta = [] #save for later analysis
for idata in drone_done_list: #item=(actual dron_service_time,id)
    data_list.append( idata[0] )
    data_list_sta.append( (data_list[-1],idata[1]) )
plt.hist(data_list, num_bins)
plt.title("Drone actual (sta) servicetime histogram")
plt.xlabel("Flight time (seconds)")
plt.ylabel("Count")
plt.savefig(dir_sim_output+'Figure'+str(icnt_plot)+'.png')
f_log_plot.write("\nSaved output to Figure"+str(icnt_plot)+": Drone actual service time\n")
f_log_plot.write("Max servicetime sec = %d\n" %(max(data_list)))
f_log_plot.write("Min servicetime sec = %d\n" %(min(data_list)))
f_log_plot.write("Avg servicetime sec = %.2f\n" %(stat.mean(data_list)))
f_log_plot.write("Std servicetime sec = %.2f\n" %(stat.stdev(data_list)))
stat_list = stat.quantiles(data_list,n=4)
f_log_plot.write("Q1 servicetime sec = %.2f\n" %(stat_list[0]))
f_log_plot.write("Q2 servicetime sec = %.2f\n" %(stat_list[1]))
f_log_plot.write("Q3 servicetime sec = %.2f\n" %(stat_list[2]))
plt.close()

icnt_plot += 1
plt.figure()
data_list = []
for ista in data_list_sta:
    for ieta in data_list_eta:
        #check same drone id
        if( ista[1]==ieta[1] ):
            data_list.append( ista[0]-ieta[0] )
            break #found match for ista, go to next one
plt.hist(data_list, num_bins)
plt.title("Drone difference (sta-eta) servicetime histogram")
plt.xlabel("Difference in flight time (seconds)")
plt.ylabel("Count")
plt.savefig(dir_sim_output+'Figure'+str(icnt_plot)+'.png')
f_log_plot.write("\nSaved output to Figure"+str(icnt_plot)+": Drone diff (sta-eta) service time\n")
f_log_plot.write("Max diff servicetime sec = %d\n" %(max(data_list)))
f_log_plot.write("Min diff servicetime sec = %d\n" %(min(data_list)))
f_log_plot.write("Avg diff servicetime sec = %.2f\n" %(stat.mean(data_list)))
f_log_plot.write("Std diff servicetime sec = %.2f\n" %(stat.stdev(data_list)))
stat_list = stat.quantiles(data_list,n=4)
f_log_plot.write("Q1 diff servicetime sec = %.2f\n" %(stat_list[0]))
f_log_plot.write("Q2 diff servicetime sec = %.2f\n" %(stat_list[1]))
f_log_plot.write("Q3 diff servicetime sec = %.2f\n" %(stat_list[2]))
plt.close()

icnt_plot += 1
# Customer service delay histogram (drone_launch_time - customer_request_time) 
plt.figure()
data_list = []
for idata in delay_list:
    data_list.append( idata[1] )
plt.hist(data_list, num_bins) 
plt.title("Delay switchover time (drone-customer) histogram")
plt.xlabel("Switchover delay time (seconds)")
plt.ylabel("Count")
plt.savefig(dir_sim_output+'Figure'+str(icnt_plot)+'.png')
f_log_plot.write("\nSaved output to Figure"+str(icnt_plot)+": Switchover output\n")
f_log_plot.write("Max switchover sec = %d\n" %(max(data_list)))
f_log_plot.write("Min switchover sec = %d\n" %(min(data_list)))
f_log_plot.write("Avg switchover sec = %.2f\n" %(stat.mean(data_list)))
f_log_plot.write("Std switchover sec = %.2f\n" %(stat.stdev(data_list)))
stat_list = stat.quantiles(data_list,n=4)
f_log_plot.write("Q1 switchover sec = %.2f\n" %(stat_list[0]))
f_log_plot.write("Q2 switchover sec = %.2f\n" %(stat_list[1]))
f_log_plot.write("Q3 switchover sec = %.2f\n" %(stat_list[2]))
plt.close()

icnt_plot += 1
# Total delay = Customer service delay + Drone fly time 
plt.figure()
data_list = []
for ista in data_list_sta:
    for idata in delay_list:
        #check if same drone
        if(ista[1]==idata[2]):
            data_list.append( idata[1]+ista[0] )
plt.hist(data_list, num_bins) 
plt.title("Total delay (switchover+fly) time histogram")
plt.xlabel("Total delay time (seconds)")
plt.ylabel("Count")
plt.savefig(dir_sim_output+'Figure'+str(icnt_plot)+'.png')
f_log_plot.write("\nSaved output to Figure"+str(icnt_plot)+": Total delay\n")
f_log_plot.write("Max delay sec = %d\n" %(max(data_list)))
f_log_plot.write("Min delay sec = %d\n" %(min(data_list)))
f_log_plot.write("Avg delay sec = %.2f\n" %(stat.mean(data_list)))
f_log_plot.write("Std delay sec = %.2f\n" %(stat.stdev(data_list)))
stat_list = stat.quantiles(data_list,n=4)
f_log_plot.write("Q1 delay sec = %.2f\n" %(stat_list[0]))
f_log_plot.write("Q2 delay sec = %.2f\n" %(stat_list[1]))
f_log_plot.write("Q3 delay sec = %.2f\n" %(stat_list[2]))
plt.close()

icnt_plot += 1
# Queue statistics info
plt.figure()
data_list = []
for idata in queue_size_list: #item=(sim_time, service_queue_size)
    tmp_size = idata[1]
    for idrone in dron_list:
        if( idata[0]==idrone[3] and tmp_size>0 ):
            tmp_size -= 1
    data_list.append( tmp_size )
plt.hist(data_list, num_bins) 
plt.title("Service queue size histogram")
plt.xlabel("Queue size")
plt.ylabel("Count")
plt.savefig(dir_sim_output+'Figure'+str(icnt_plot)+'.png')
f_log_plot.write("\nSaved output to Figure"+str(icnt_plot)+": Service queue size stats\n")
f_log_plot.write("Max queue size = %d\n" %(max(data_list)))
f_log_plot.write("Min queue size = %d\n" %(min(data_list)))
f_log_plot.write("Avg queue size = %.2f\n" %(stat.mean(data_list)))
f_log_plot.write("Std queue size = %.2f\n" %(stat.stdev(data_list)))
stat_list = stat.quantiles(data_list,n=4)
f_log_plot.write("Q1 queue size = %.2f\n" %(stat_list[0]))
f_log_plot.write("Q2 queue size = %.2f\n" %(stat_list[1]))
f_log_plot.write("Q3 queue size = %.2f\n" %(stat_list[2]))
plt.close()

icnt_plot += 1
plt.figure()
f_log_plot.write("\nSaved output to Figure"+str(icnt_plot)+": Free drones per depot timeplot\n")
for idept in range(num_dept):
    xdata_list = []
    ydata_list = []
    for idata in drone_size_list: #item=(sim_time, dept_id, free_drones)
        if(idata[1] == idept):
            xdata_list.append( idata[0] )
            ydata_list.append( idata[2] )
    #end for
    if(len(xdata_list) == 0):
        #special case: depot not touched so drone capability full
        xdata_list = [0, (Ts*tot_icnt)]
        ydata_list = [max_drones_per_depot, max_drones_per_depot]
    if(len(xdata_list) == 1):
        #special case: only recorded once
        xdata_list = [xdata_list[-1], xdata_list[-1]]
        ydata_list = [max_drones_per_depot-1, max_drones_per_depot-1]
    plt.plot( xdata_list,ydata_list, c=('C'+str(idept)), label=str(idept) ) 
    #log data stats
    f_log_plot.write("Depot[%d] drones stats:\n" %idept)
    f_log_plot.write("Max free drones size = %d\n" %(max(ydata_list)))
    f_log_plot.write("Min free drones size = %d\n" %(min(ydata_list)))
    f_log_plot.write("Avg free drones size = %.2f\n" %(stat.mean(ydata_list)))
    f_log_plot.write("Std free drones size = %.2f\n" %(stat.stdev(ydata_list)))
    stat_list = stat.quantiles(ydata_list,n=4)
    f_log_plot.write("Q1 free drones size = %.2f\n" %(stat_list[0]))
    f_log_plot.write("Q2 free drones size = %.2f\n" %(stat_list[1]))
    f_log_plot.write("Q3 free drones size = %.2f\n" %(stat_list[2]))
#end
plt.title("Free drones per depot per timepoint")
plt.xlabel("Simulation time (seconds)")
plt.ylabel("Grounded drones count")
plt.legend()
plt.savefig(dir_sim_output+'Figure'+str(icnt_plot)+'.png')
plt.close()

#------------------------------------------------
f_log_plot.close()
print("DONE")
