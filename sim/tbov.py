# tbov.py
#
#  Module for storing TBOV ops blocks. 
import math

#####################################################################
def rotateXY( x,y,angle_rad ):
    '''
    Returns rotation counterclockwise about (0,0) for (x,y)
    '''
    return (x*math.cos(angle_rad)-y*math.sin(angle_rad), x*math.sin(angle_rad)+y*math.cos(angle_rad))

#####################################################################
class TBOV:
    '''
    TBOV container per route:
        - Each route has its own TBOV, rectangluar volumes enclose trajectory
        - TODO
    TBOV status:
        all distance is (meter)
        all time is (sec)
    '''
    IGNORE_DEPOT_DISTANCE = 10      #distance from any depot to ignore conflicts

    '''
    Specific TBOV methods:
    '''
    def __init__(self, drone_route,drone_size, TBOV_length_scale=1,TBOV_width_scale=1):
        self.active = False
        #
        self.length = TBOV_length_scale * drone_size
        self.width = TBOV_width_scale * drone_size
        #
        self.Xr = drone_route[0]
        self.Yr = drone_route[1]
        self.Tr = drone_route[2]
        self.Hr = drone_route[3]
        #
        self.pts = []
        self.updatePts()
    
    def updatePts(self):
        #--- init pts
        ix = self.Xr[0]
        iy = self.Yr[0]
        ih = self.Hr[0]
        #
        (rxwid,rywid) = rotateXY(0,self.width,ih)
        ixup = ix + rxwid/2
        iyup = iy + rywid/2
        self.pts.append( (ix,iy , ixup,iyup) )
        ixdw = ix - rxwid/2
        iydw = iy - rywid/2
        self.pts.append( (ix,iy , ixdw,iydw) )
        #--- end init
        #create vertex pts around route
        for icnt in range(len(self.Xr)-1):
            px = self.Xr[icnt]
            py = self.Yr[icnt]
            ph = self.Hr[icnt]
            #
            cx = self.Xr[icnt+1]
            cy = self.Yr[icnt+1]
            #
            ir = math.sqrt( math.pow(cx-px,2)+math.pow(cy-py,2) )
            ircnt = round(ir/self.length)
            #length offset and cnt (len in direction of heading)
            (rxlen,rylen) = rotateXY(self.length,0,ph)
            for icntlen in range(ircnt):
                ix = self.pts[-1][0] + rxlen
                iy = self.pts[-1][1] + rylen
                #width offset
                (rxwid,rywid) = rotateXY(0,self.width,ph)
                ixup = ix + rxwid/2
                iyup = iy + rywid/2
                self.pts.append( (ix,iy , ixup,iyup) )
                ixdw = ix - rxwid/2
                iydw = iy - rywid/2
                self.pts.append( (ix,iy , ixdw,iydw) )
        #endfors

    '''Retrieve methods'''

    def getBoxes(self):
        status = False
        box_list = [] #[(x_bl,y_bl, x_br,y_br, x_tr,y_tr, x_tl,y_tl) , ...] bottomleft,bottomright,topright,topleft
        minpts = 4
        if( len(self.pts)>=minpts ):
            #pts = [center_x,center_y, edgeWidth_x,edgeWidth_y]
            bottom=False
            tmpbottom=[]
            tmptop=[]
            #
            firstflagCnt=0
            for tmppt in self.pts:
                if(firstflagCnt<2):
                    tmpbottom.append( (self.pts[1][2],self.pts[1][3] , self.pts[3][2],self.pts[3][3]) )
                    tmptop.append( (self.pts[0][2],self.pts[0][3] , self.pts[2][2],self.pts[2][3]) )
                    firstflagCnt += 1 #done one pt, special case
                else:
                    if(bottom):
                        tmpbottom.append( (tmpbottom[-1][2],tmpbottom[-1][3] , tmppt[2],tmppt[3]) )
                        bottom = False
                    else:
                        tmptop.append( (tmptop[-1][2],tmptop[-1][3] , tmppt[2],tmppt[3]) )
                        bottom = True
            #endfor, add last one
            tmpbottom.append( (tmpbottom[-1][2],tmpbottom[-1][3] , self.pts[-1][2],self.pts[-1][3]) )
            tmptop.append( (tmptop[-1][2],tmptop[-1][3] , self.pts[-2][2],self.pts[-2][3]) )
            #end
            lenT = len(tmpbottom)
            lenB = len(tmptop)
            #print("T=[%d] B=[%d]" %(lenT,lenB))
            size = min(lenT,lenB)
            for icnt in range(size-1):
                box_list.append( (tmpbottom[icnt][0],tmpbottom[icnt][1],tmpbottom[icnt+1][0],tmpbottom[icnt+1][1] , \
                                  tmptop[icnt+1][0], tmptop[icnt+1][1], tmptop[icnt][0],     tmptop[icnt][1]) )
            #endfor
            status = True
        #endif
        return (status,box_list)

#####################################################################
class TBOVbuffer:
    '''
    TBOV buffer for route crossings:
        - When two or more routes intersect, they have common buffers
        - buffer represented as square volume 
        - TODO
    TBOV buffer status:
        all distance is (meter)
        all time is (sec)
    '''

    '''
    Specific TBOV buffer methods:
    '''
    def __init__(self):
        self.active = False
        #
        self.buffers = []
        self.buffers_pts = []

    def placeBuffer(self, Loc, size):
        #Loc=(x,y)
        ix = Loc[0]
        iy = Loc[1]
        self.buffers.append( (ix,iy, size) )
        #place pts around
        ix2 = ix + size/2
        iy2 = iy + size/2
        self.buffers_pts.append( (ix,iy , ix2,iy2) )
        ix2 = ix + size/2
        iy2 = iy - size/2
        self.buffers_pts.append( (ix,iy , ix2,iy2) )
        ix2 = ix - size/2
        iy2 = iy - size/2
        self.buffers_pts.append( (ix,iy , ix2,iy2) )
        ix2 = ix - size/2
        iy2 = iy + size/2
        self.buffers_pts.append( (ix,iy , ix2,iy2) )

    '''Retrieve methods'''

    def getBuffers(self):
        status = False
        buffer_list = []
        tmp_list = []
        for i in range( len(self.buffers_pts) ):
            if( i%4 == 0 and i>0 ):
                buffer_list.append(tmp_list)
                tmp_list = [] #clear for next buffer
            #end
            tmp_list.append(self.buffers_pts[i])
        if(len(buffer_list)>0):
            buffer_list.append(tmp_list) #add last buffer
            status = True
        return (status,buffer_list)

#####################################################################
# TESTING MODULE
#################
if __name__ == "__main__":
    #  Uses mathplotlib. `python -m pip install -U matplotlib`
    import matplotlib.pyplot as plt             #main plotting
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = 'Times New Roman'
    fig, ax = plt.subplots()

    #--- load mission planner
    import drone
    graphFile = "sim_input/_test/graph_matrix.csv"
    testGraph = drone.DroneMissionPlanner()
    assert( testGraph.loadGraph(graphFile) )
    #
    pathsFile = "sim_input/_test/paths_s.txt"
    assert( testGraph.loadPaths(pathsFile) )
    #
    depotLocFile = "sim_input/_test/depot_loc.txt"
    assert( testGraph.loadDepots(depotLocFile) )
    custLocFile = "sim_input/_test/customer_loc.txt"
    assert( testGraph.loadCustomers(custLocFile) )
    #generate some routes
    numRoutes=(testGraph.num_dept)*(testGraph.num_cust)
    testroute_list=[]
    for idept in range(testGraph.num_dept):
        for icust in range(testGraph.num_cust):
            speed = 20  #m/s
            setup = 10 #sec
            testroute = testGraph.generateRoute(idept,icust,speed,setup)
            testroute_list.append(testroute)
            print("Route["+str(idept)+"->"+str(icust)+"] <"+str(testroute)+">" )
    #--- end load mission planner

    #pre-plot
    h = 150
    plt.cla()
    plt.axis([-h,1000+h,-h,1000+h])

    #--- test OI
    drone_size = 1
    for itest in range(numRoutes):
        TBOV_l = 50
        TBOV_w = 20
        testTBOV = TBOV(testroute_list[itest],drone_size,TBOV_l,TBOV_w)
        print("Pts["+str(itest)+"] total <"+str(len(testTBOV.pts))+">")

        xlist=[]
        ylist=[]
        for ipt in testTBOV.pts:
            xlist.append(ipt[2])
            ylist.append(ipt[3])
        plt.scatter(xlist,ylist, c='C'+str(itest),s=15, edgecolors='none', label=''+str(itest)+' OI')

        boxlist=[]
        try:
            (status,boxlist) = testTBOV.getBoxes()
            assert(status)
            print("Boxes["+str(itest)+"] total <"+str(len(boxlist))+">")
            for ibox in boxlist:
                plt.scatter(ibox[0],ibox[1], c='C'+str(itest),s=15, edgecolors='black')
                plt.plot( [ibox[0],ibox[2]],[ibox[1],ibox[3]], c='C'+str(itest) )
                plt.plot( [ibox[2],ibox[4]],[ibox[3],ibox[5]], c='C'+str(itest) )
                plt.plot( [ibox[4],ibox[6]],[ibox[5],ibox[7]], c='C'+str(itest) )
                plt.plot( [ibox[6],ibox[0]],[ibox[7],ibox[1]], c='C'+str(itest) )
            #end box plot
        except Exception as e:
            print("%s" %e)
    #--- end test OI
    #--- test buffers
    bufferptslist = testGraph.getCrossings()
    testTBOVbuffers = TBOVbuffer()
    testsize = 100
    print("Number of Crossings <"+str(len(bufferptslist))+"> :"+str(bufferptslist))
    for i in range(len(bufferptslist)):
        Loc = ( bufferptslist[i][0]*1000 , bufferptslist[i][1]*1000 )
        testTBOVbuffers.placeBuffer(Loc,testsize)
        print("Buffer <"+str(testTBOVbuffers.buffers[-1])+">")
    #endfor
    #plot the buffers
    (status,bufferlist) = testTBOVbuffers.getBuffers()
    assert(status)
    print("Number of Buffers <"+str(len(bufferlist))+">")
    for ibuffer in bufferlist:
        x0 = ibuffer[0][2]
        y0 = ibuffer[0][3]
        x1 = ibuffer[1][2]
        y1 = ibuffer[1][3]
        x2 = ibuffer[2][2]
        y2 = ibuffer[2][3]
        x3 = ibuffer[3][2]
        y3 = ibuffer[3][3]
        plt.scatter(ibuffer[0][0],ibuffer[0][1], c='green', edgecolors='black')
        plt.plot( [x0,x1],[y0,y1], c='green' )
        plt.plot( [x1,x2],[y1,y2], c='green' )
        plt.plot( [x2,x3],[y2,y3], c='green' )
        plt.plot( [x3,x0],[y3,y0], c='green' )
    #end plot

    #--- end test buffers

    print("DONE... close window")
    plt.legend()
    plt.grid(True)
    plt.show()
