# main_plot_full_v2.py
#
# version 2: plots from multiple Network algorithms into one plot
#
#  Data plots from output main_autoplot.sh, i.e. reading log_plot_full.txt
#  Uses mathplotlib. `python -m pip install -U matplotlib`
import matplotlib.pyplot as plt #main plotting
import sys
import csv

#------------------------------------------------
dir_sim_input  = "../sim_input/test0/"
dir_sim_output = "../sim_output/test0/"

plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = 'Times New Roman'

algorithm_list = ["FCFS","RR","Random"]
lambd=100
w=20
#
l_min=100
l_max=1000
l_step=100
#setup for plot---
x_ticks = []
i_ticks = []
x_ticks_len = int((l_max-l_min)/l_step) + 1
for i in range(x_ticks_len):
    x_ticks.append(l_min + i*l_step)
    i_ticks.append(i)
#---

icnt=0
for ialgo in algorithm_list:
    icnt += 1

    dir_sim_input_1   = dir_sim_input
    dir_sim_output_1  = dir_sim_output+str(ialgo)+"/lambda="+str(lambd)+"/"

    cmptext = "./sim_output/"+str(ialgo)+"/lambda"

    # READ log_plot_full.txt ------------------------
    try:
        f_plt_file = ""+dir_sim_output_1+"Width="+str(w)+"/log_plot_full.txt"
        print(f_plt_file)
        f_plt = open(f_plt_file,"r")
        reader = csv.reader(f_plt, delimiter='=')
        s = reader.__next__() #--- header ---
        print(s)
        data_list = []
        cust_list = []
        dron_list = []
        swit_list = []
        queu_list = []
        #
        for l in range(l_min,l_max+l_step,l_step):
            #continue till next sub-header
            cmpcheck = ""
            while( cmptext != cmpcheck ):
                s = reader.__next__()
                print(s,"len="+str(len(s)),"_1")
                if( 0 == len(s) ):
                    cmpcheck = ""
                else:
                    cmpcheck = s[0]
            #end
            #now find matchings
            while( "" != cmpcheck ):
                #---
                if(   "Total number of customer requests " == s[0] ):
                    c_resq = int(s[1])                  # customer requests
                    cust_list.append( c_resq )
                elif( "Total number of drones finished   " == s[0] ):
                    d_done = int(s[1])                  # drones finished
                    d_flew = int(reader.__next__()[1])  # drones flew 
                    dron_list.append( (d_done,d_flew) )
                elif( "Max switchover sec " == s[0] ):
                    s_max = float(s[1])                 # Max switchover
                    s_min = float(reader.__next__()[1]) # Min switchover
                    s_avg = float(reader.__next__()[1]) # Avg switchover
                    s_std = float(reader.__next__()[1]) # Std switchover 
                    s_Q1 = float(reader.__next__()[1])  # Q1 switchover
                    s_Q2 = float(reader.__next__()[1])  # Q2 switchover
                    s_Q3 = float(reader.__next__()[1])  # Q3 switchover
                    swit_list.append( (s_max,s_min,s_avg,s_std,s_Q1,s_Q2,s_Q3) )
                elif( "Max queue size " == s[0] ):
                    q_max = int(s[1])                   # Max queue
                    q_min = int(reader.__next__()[1])   # Min queue
                    q_avg = float(reader.__next__()[1]) # Avg queue
                    q_std = float(reader.__next__()[1]) # Std queue
                    q_Q1 = float(reader.__next__()[1])  # Q1 queue
                    q_Q2 = float(reader.__next__()[1])  # Q2 queue
                    q_Q3 = float(reader.__next__()[1])  # Q3 queue
                    queu_list.append( (q_max,q_min,q_avg,q_std,q_Q1,q_Q2,q_Q3) )
                #---
                s = reader.__next__() 
                print(s,"len="+str(len(s)),"_1")
                if( 0 == len(s) ):
                    cmpcheck = ""
                else:
                    cmpcheck = s[0]   
        #end L for
        data_list.append( (cust_list,dron_list,swit_list,queu_list) )
    except Exception as e:
        print("%s" %e )
        sys.exit(1)

    #------------------------------------------------
    # Queue stats ---
    fig_q = plt.figure(1)
    #ax = fig_q.add_subplot(111) #dummy placeholder, used to get "ax"
    q_avg_list = []
    q_std_1_list = [] # avg+std
    q_std_2_list = [] # avg-std
    q_min_list = []
    q_max_list = []
    q_Q1_list = []
    q_Q2_list = []
    q_Q3_list = []
    for idata in data_list[0][3]: 
        q_Q3_list.append( idata[6] )
        q_Q2_list.append( idata[5] )
        q_Q1_list.append( idata[4] )
        std = idata[3]
        q_avg_list.append( idata[2] )
        a_std = q_avg_list[-1]+std
        s_std = q_avg_list[-1]-std
        q_std_1_list.append(a_std)
        if( s_std < 0 ):
            q_std_2_list.append(0)
        else:
            q_std_2_list.append(s_std)
        q_min_list.append( idata[1] )
        q_max_list.append( idata[0] )
    #plt.plot(q_Q3_list,c='C'+str(icnt),linestyle='--')
    #plt.plot(q_Q2_list,c='C'+str(icnt),linestyle='-') #Median
    #plt.plot(q_Q1_list,c='C'+str(icnt),linestyle='--') 
    plt.plot(q_avg_list,c='C'+str(icnt),linestyle='-',marker='o',label=str(ialgo))#Mean
    #plt.plot(q_std_1_list,c='C'+str(icnt),linestyle='-')#Mean+std
    #plt.plot(q_std_2_list,c='C'+str(icnt),linestyle='-')#Mean-std
    for i in range(len(q_std_1_list)):
        plt.plot([(i),(i)],[q_std_1_list[i],q_std_2_list[i]],c='C'+str(icnt),linestyle='-',marker='_')
    #plt.plot(q_min_list,c='C'+str(icnt),linestyle='',marker='v')
    #plt.plot(q_max_list,c='C'+str(icnt),linestyle='',marker='^')
    plt.title("Service queue size with W="+str(w)+" meters")
    plt.xticks(i_ticks,x_ticks)
    plt.xlabel('Length, L (meters)')
    plt.ylabel('Average number of waiting customers')
    plt.grid(True, linewidth=0.25)
    #ax.set_aspect('equal', adjustable='box')
    plt.legend(title="Queueing Output Model")

    #---------------------------------
    # Switchover stats ---
    fig_s = plt.figure(2)
    #ax = fig_s.add_subplot(111) #dummy placeholder, used to get "ax"
    s_avg_list = []
    s_std_1_list = [] # avg+std
    s_std_2_list = [] # avg-std
    s_min_list = []
    s_max_list = []
    s_Q1_list = []
    s_Q2_list = []
    s_Q3_list = []
    for idata in data_list[0][2]: 
        s_Q3_list.append( idata[6] )
        s_Q2_list.append( idata[5] )
        s_Q1_list.append( idata[4] )
        std = idata[3]
        s_avg_list.append( idata[2] )
        a_std = s_avg_list[-1]+std
        s_std = s_avg_list[-1]-std
        s_std_1_list.append(a_std)
        if( s_std < 0 ):
            s_std_2_list.append(0)
        else:
            s_std_2_list.append(s_std)
        s_min_list.append( idata[1] )
        s_max_list.append( idata[0] )
    #plt.plot(s_Q3_list,c='C'+str(icnt),linestyle='--')
    #plt.plot(s_Q2_list,c='C'+str(icnt),linestyle='-') #Median
    #plt.plot(s_Q1_list,c='C'+str(icnt),linestyle='--')
    plt.plot(s_avg_list,c='C'+str(icnt),linestyle='-',marker='o',label=str(ialgo))#Mean
    for i in range(len(s_std_1_list)):
        plt.plot([(i),(i)],[s_std_1_list[i],s_std_2_list[i]],c='C'+str(icnt),linestyle='-',marker='_')
    #plt.plot(s_min_list,c='C'+str(icnt),linestyle='',marker='v')
    #plt.plot(s_max_list,c='C'+str(icnt),linestyle='',marker='^')
    plt.title("Drone ground delay with W="+str(w)+" meters")
    plt.xticks(i_ticks,x_ticks)
    plt.xlabel('Length, L (meters)')
    plt.ylabel('Average delay (seconds)')
    plt.grid(True, linewidth=0.25)
    #ax.set_aspect('equal', adjustable='box')
    plt.legend(title="Queueing Output Model")

    #------------------------------------------------
    # Throughput stats ---
    fig_t = plt.figure(3)
    #ax = fig_t.add_subplot(111) #dummy placeholder, used to get "ax"
    t_list = []
    c_reqt = data_list[0][0][0]
    for idata in data_list[0][1]: 
        t_list.append( 100 * idata[1]/c_reqt )
    #
    plt.plot(t_list,c='C'+str(icnt),linestyle='-',marker='o',label=str(ialgo))#Mean
    #
    plt.title("Throughput percentage with W="+str(w)+" meters")
    plt.xticks(i_ticks,x_ticks)
    plt.xlabel('Length, L (meters)')
    plt.ylabel('Customers querries percentage served (0-100%)')
    plt.grid(True, linewidth=0.25)
    #ax.set_aspect('equal', adjustable='box')
    plt.legend(title="Queueing Output Model")

#END ---------------------------------
plt.figure(1)
plt.savefig(dir_sim_output+'Figure_size_W='+str(w)+'.png')
plt.close(1)
plt.figure(2)
plt.savefig(dir_sim_output+'Figure_delay_W='+str(w)+'.png')
plt.close(2)
plt.figure(3)
plt.savefig(dir_sim_output+'Figure_throughput_W='+str(w)+'.png')
plt.close(3)

