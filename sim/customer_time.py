# customer_time.py
#
#  Module for obtaining customer service requests.
#  It draws next time depending on distribution or ...
import random
import math
import csv

#------------------------------------------------
#dir_sim_input   = "./sim_input/"

#####################################################################
class CustomerSimTime:
    '''Getting next sim customer service time Class'''

    '''Specific Customer Time Methods'''
    def __init__(self, scale, number_of_customers,number_of_depots, seed, datafile=None,readfile=False):
        self.rand = random.Random(seed)
        self.scale = scale
        self.numCust = number_of_customers
        self.numDept = number_of_depots
        #
        self.dataFile = datafile
        if(None!=self.dataFile):
            self.dataFile = datafile
            self.readFile = readfile
            if(self.readFile):
                self.file = open(self.dataFile,"r")
                self.reader = csv.reader(self.file)
            else:
                self.file = open(self.dataFile,"w")
                self.icnt = 0
                self.currTime = 0

    def getNextExpTime(self, curr_time):
        tmp = 0
        u = self.rand.uniform(0,1)
        x = float( -(1/self.scale) * math.log( (1-u) ))
        tmp = (x + curr_time)
        return tmp

    def getNextUniTime(self, curr_time):
        tmp = 0
        x = self.scale * self.rand.uniform(0,1)
        tmp = (x + curr_time)
        return tmp

    def getNextConstTime(self, curr_time):
        tmp = 0
        x = self.scale
        tmp = (x + curr_time)
        return tmp
    
    '''Data stored in file'''
    def readNextTime(self):
        if(None!=self.dataFile and self.readFile):
            (icnt, ideptid,icustid,it) = self.reader.__next__()
            icnt = int(icnt)
            ideptid = int(ideptid)
            icustid = int(icustid)
            it = float(it)
            #
            return (icnt, ideptid,icustid,it)

    def writeNextTime(self, type='exp'):
        if(None!=self.dataFile and not self.readFile):
            self.icnt = self.icnt + 1
            ideptid = self.getDepotId()
            icustid = self.getCustomerId()
            if('exp'==type):
                it = self.getNextExpTime(self.currTime)
            elif('uni'==type):
                it = self.getNextUniTime(self.currTime)
            else:
                it = self.getNextConstTime(self.currTime)
            #
            self.file.write("%d,%d,%d,%f\n" %(self.icnt,ideptid,icustid,it))
            self.currTime = it

    def closeFile(self):
        if(None!=self.dataFile):
            self.file.close()
    
    '''Dept/Customer ids'''
    def getCustomerId(self):
        return self.rand.randint(0,self.numCust-1)

    def getDepotId(self):
        return self.rand.randint(0,self.numDept-1)
