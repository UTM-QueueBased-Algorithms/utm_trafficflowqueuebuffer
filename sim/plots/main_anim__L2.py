# main_anim__L2.py
#
#  Data (animation) plots from output main.py, i.e. reading log.txt
#  Uses mathplotlib. `python -m pip install -U matplotlib`
import matplotlib.pyplot as plt             #main plotting
from matplotlib.patches import Rectangle    #for plotting TBOVs
from matplotlib.patches import FancyArrow   #for plotting drones
import csv
import math

#  Uses local directory modules too
import sys
sys.path.insert(0,"..")
#---
import drone

#------------------------------------------------
dir_sim_input   = "../sim_input/"
dir_sim_output  = "../sim_output/"

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Times New Roman'

flag_plot        = True
flag_interactive = False #Enable prompts and requests for generating animations?
print_event_rate = 1   #Frequency (every X counts) of extraction from data to plot
tot_icnt = 10000         #Limit to X number of sim steps

# READ log.txt ----------------------------------
# data initializations
time_events_list = []   #item=(sim_time)
dron_conflict_list= []  #item=(sim_time, id)
dron_service_list = []  #item=(sim_time, cust_id,dept_id, dron_service_time, id)
dron_fly_list     = []  #item=(sim_time, x,y,heading, id)

Dept_Loc = []   #item=(dept_id,x,y)
Cust_Loc = []   #item=(cust_id,x,y)

#default settings, updated from reading log.txt
Ts = 0.5
print_rate = 100        
w = 5
h = 5
#
#speed = 1

f_log_anim = open(dir_sim_output+"log_anim.txt", "w+") #capture everything
# expected data format
#  - drone Start log data    (1=sim_time, 2=x, 3=y, 4=heading, 5=cust_id, 6=dept_id, 7=eta, 8=free_drones, 9=id)
#  - drone FLY   log data    (1=sim_time, 2=x, 3=y, 4=heading, 5=id)
#  - drone Conflict  data    (1=sim_time, 2=id)
with open(dir_sim_output+"log.txt","r") as f_log:
    reader = csv.reader(f_log)
    for line in reader:
        new_event = False
        #---
        if(  line[0] == "Drone Start"):
            dron_service_list.append( (float(line[1]), int(line[5]), int(line[6]), float(line[7]), int(line[9])) )
            new_event = True
        elif(line[0] == "Drone FLY"):
            dron_fly_list.append( (float(line[1]), float(line[2]), float(line[3]), float(line[4]), int(line[5])) )
            new_event = True
        elif(line[0] == "Drone Conflict"):
            dron_conflict_list.append( ((float(line[1]), int(line[2]))) )
            new_event = True
        else:
            f_log_anim.write(str(line)+"\n")
            #non-event parameters, used for plotting info
            if(line[0] == "time step (sec)"):
                Ts = float(line[1])
            elif(line[0] == "print output every (count)"):
                print_rate = int(line[1])
            elif(line[0] == "TBOV width from drone center (meter)"):
                w = int(line[1])
            elif(line[0] == "TBOV length from drone center (meter)"):
                h = int(line[1])
        #---
        if(new_event):
            tmp_time = float(line[1])
            #add when no old time to compare...
            if( time_events_list.__len__()==0 ):
                time_events_list.append( tmp_time )
            #otherwise, add only new times
            elif( time_events_list[-1]<tmp_time ):
                time_events_list.append( tmp_time )
f_log_anim.write("Number of Service events  = %d\n" %dron_service_list.__len__())
f_log_anim.write("Number of FLY events      = %d\n" %dron_fly_list.__len__())
f_log_anim.write("Number of Conflict events = %d\n" %dron_conflict_list.__len__())

#TODO: assume distance is in kilo-meters, convert to meters
#  - customer log data (1=cust_id, 2=x, 3=y)
with open(dir_sim_input+"customer_loc.txt","r") as f_cust:
    reader = csv.reader(f_cust)
    for line in reader:
        Cust_Loc.append( (line[1],float(line[2])*1000,float(line[3])*1000) )
#  - depot log data (1=dept_id, 2=x, 3=y)
with open(dir_sim_input+"depot_loc.txt","r") as f_dept:
    reader = csv.reader(f_dept)
    for line in reader:
        Dept_Loc.append( (line[1],float(line[2])*1000,float(line[3])*1000) )

# Animation -------------------------------------
if(flag_plot):
    fig, ax = plt.subplots()

#static objects initializations ---
# Depot locations data
xdata_dept_list = []
ydata_dept_list = []
for idata in Dept_Loc:
    xdata_dept_list.append( idata[1] )
    ydata_dept_list.append( idata[2] )
# Customer locations data
xdata_cust_list = []
ydata_cust_list = []
for idata in Cust_Loc:
    xdata_cust_list.append( idata[1] )
    ydata_cust_list.append( idata[2] )
#---

#other initializations
dron_active_list = []
dron_special_list= []
cust_active_list = []
#TODO: current assume chronological order in drone mission stats
icnt_fly = 0        #iterating through dron_active_list
icnt_special_fly = 0#iterating through dron_special_list
icnt_mission = 0    #iterating through cust_active_list

#OIs
factor = 0.75 #percent overlap, range(0,1)
OI_list = []
for idept in Dept_Loc:
    for icust in Cust_Loc:
        ixt = idept[1]
        iyt = idept[2]
        ixc = icust[1]
        iyc = icust[2]
        head = math.atan2(iyc-iyt,ixc-ixt)
        l_x = h*math.cos(head) #- w*math.sin(head)
        l_y = h*math.sin(head) #+ w*math.cos(head)
        L = []
        cval = abs(ixt-ixc)+abs(iyt-iyc)
        pval = cval + 1
        while( pval > cval ):
            L.append( (ixt,iyt,head) )
            pval = cval
            ixt += factor*l_x
            iyt += factor*l_y
            cval = abs(ixt-ixc)+abs(iyt-iyc)
            #print(cval,pval)
        OI_list.append(L)
#exit()

# MAIN ANIMATION ---
f_log_anim.write("---ANIMATION DATA---\n")
flag_start = True
icnt = -1   #is actually 0 because it is first incremented
try:
    for itime in time_events_list:
        icnt += 1
        f_log_anim.write("SIM TIME=%.2f\n" %itime)
        #UPDATING DATA-------
        #FLY data updated every Ts*print_rate
        if( itime % (Ts*print_rate) == 0 ):
            dron_active_list = []
            dron_special_list= []
            #skip past missed events
            while( icnt_fly<dron_fly_list.__len__() and dron_fly_list[icnt_fly][0]<itime ):
                icnt_fly += 1
            while( icnt_special_fly<dron_conflict_list.__len__() and dron_conflict_list[icnt_special_fly][0]<itime ):
                icnt_special_fly += 1
            #add current (flying) drones
            while( icnt_fly<dron_fly_list.__len__() and dron_fly_list[icnt_fly][0]==itime ):
                #check if drone is in special condition (i.e. conflict)
                drone_id = dron_fly_list[icnt_fly][4]
                status = ( dron_conflict_list.__len__()>0 and icnt_special_fly<dron_conflict_list.__len__() )
                if( status and dron_conflict_list[icnt_special_fly][0]==itime and dron_conflict_list[icnt_special_fly][1]==drone_id):
                    dron_special_list.append( dron_fly_list[icnt_fly][:] )
                    f_log_anim.write("Conflict drone[id=%d]\n" %drone_id)
                    icnt_special_fly += 1
                else:
                    dron_active_list.append( dron_fly_list[icnt_fly][:] )
                icnt_fly += 1 #check next one
            #end loop
            f_log_anim.write("Currently FLY=%d\n" %dron_active_list.__len__())
        #end FLY data

        #remove expired/serviced customer data
        for icustomer in cust_active_list:
            if( (icustomer[0]+icustomer[3])< itime ):
                f_log_anim.write("Depot[%d] drone[id=%d] returned from customer[%d]\n" %(cust_active_list[-1][2],cust_active_list[-1][4],cust_active_list[-1][1]) )
                cust_active_list.remove(icustomer)
        #end, now add new customer data
        while( icnt_mission<dron_service_list.__len__() and dron_service_list[icnt_mission][0]==itime ):
            cust_active_list.append( dron_service_list[icnt_mission][:] )
            f_log_anim.write("Depot[%d] sends drone[id=%d] to customer[%d]\n" %(cust_active_list[-1][2],cust_active_list[-1][4],cust_active_list[-1][1]) )
            icnt_mission += 1 #check next one
        #end loop

        #SPECIAL CHECKS------
        if(icnt > tot_icnt):
            f_log_anim.write("---stopped by user max cnt=%d---\n" %tot_icnt)
            break
        #skip X counts depending on print frequency
        if(icnt % print_event_rate != 0):
            continue #skip this time event
        #exit prompt if interactive 
        if(icnt % print_event_rate == 0 and flag_interactive):
            ans = input("Continue Animation [Y/N]: ")
            if(ans.lower() == "n"):
                f_log_anim.write("---stopped by user input---\n")
                break
        elif(flag_start):
            ans = input("Start Animation [Y/N]: ")
            if(ans.lower() == "n"):
                f_log_anim.write("---stopped by user input---\n")
                break
            else:
                flag_start = False
        #end

        #PLOTTING------------
        if(flag_plot):
            plt.cla()
            plt.axis([-h,1000+h,-h,1000+h])

            # 2D top-view of simulation map
            plt.scatter(xdata_dept_list,ydata_dept_list, c='orange',s=80, edgecolors='none', label='depot')
            plt.scatter(xdata_cust_list,ydata_cust_list, c='blue',s=20, edgecolors='none', label='customer')
            #end 2D top-view environment

            # OIs view map
            for L in OI_list:
                for iL in L:
                    x = iL[0]
                    y = iL[1]
                    heading = iL[2]*180.0/math.pi
                    ax.add_patch(Rectangle((x,y), h,w,heading, color='blue',fill=False,alpha=0.25))
            #end OIs view

            if( cust_active_list.__len__()>0 ):
                #go through all active mission (i.e. depot-customer) and plot TBOVs
                for icustomer in cust_active_list:
                    icust = icustomer[1]
                    idept = icustomer[2]
                    #plot active customers                    
                    plt.scatter(Cust_Loc[icust][1],Cust_Loc[icust][2], c='blue',s=30, edgecolors='yellow')

            tot_fly_cnt = dron_active_list.__len__() + dron_special_list.__len__()
            if( dron_active_list.__len__()>0 or dron_special_list.__len__()>0 ):
                firstflag_dronactivelist = True
                firstflag_dronspecialist = True
                #go through all active drones and plot their locations
                for idrone in dron_active_list:
                    x = idrone[1]
                    y = idrone[2]
                    heading = idrone[3]*180.0/math.pi
                    #dx = x+math.cos(heading)
                    #dy = y+math.sin(heading)
                    #plot drone location
                    if( firstflag_dronactivelist ):
                        plt.scatter(x,y, c='purple',s=20, edgecolors='none', label='normal UAV')
                        firstflag_dronactivelist = False
                    else:
                        plt.scatter(x,y, c='purple',s=20, edgecolors='none')
                    #ax.add_patch(FancyArrow(x,y, dx,dy, width=0.001,color='purple',capstyle='projecting'))
                    #find OIs, drone is in
                    oi_x=x
                    oi_y=y
                    oi_heading=heading
                    oi_cnt = 0
                    for L in OI_list:
                        for iL in L:
                            tmp_x = iL[0]
                            tmp_y = iL[1]
                            tmp_heading = iL[2]*180.0/math.pi
                            if( abs(tmp_heading-heading)<10 ):
                                tmp_xx = tmp_x + h*math.cos(tmp_heading*math.pi/180.0)
                                tmp_yy = tmp_y + h*math.sin(tmp_heading*math.pi/180.0)
                                if( (abs(tmp_xx-x)<h and abs(tmp_x-x)<h) and (abs(tmp_yy-y)<h and abs(tmp_y-y)<h) ):
                                    ax.add_patch(Rectangle((tmp_x,tmp_y), h,w,heading, color='blue',fill=True,alpha=0.35))
                                    oi_cnt += 1
                                    #plt.text(tmp_x,tmp_y,str(oi_cnt))

                #go through all special active drones and plot their locations
                for idrone in dron_special_list:
                    x = idrone[1]
                    y = idrone[2]
                    heading = idrone[3]*180.0/math.pi
                    #plot drone location
                    if( firstflag_dronspecialist ):
                        plt.scatter(x,y, c='red',s=20, edgecolors='none', label='hold UAV')
                        firstflag_dronspecialist = False
                    else:
                        plt.scatter(x,y, c='red',s=20, edgecolors='none')
                    ax.add_patch(Rectangle((x,y), h,w,heading, color='red',fill=False))

            plt.legend()
            plt.text(400,1200, "Time= "+str(itime)+"/"+str(time_events_list[-1])+" - Fly= "+str(tot_fly_cnt))
            plt.grid(True)
            plt.title("Simulation Animation")
            #plt.show()
            plt.pause(1)
        #END PLOTTING---

except KeyboardInterrupt:
    f_log_anim.write("---Stopping animation due to keyboard...\n")

#------------------------------------------------
f_log_anim.write("---DONE ANIMATION---")
f_log_anim.close()
print("DONE")
