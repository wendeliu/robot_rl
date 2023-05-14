
import time as t
import csv
from time import sleep


#import smbus

#from AiAgent import AiAgent

#import smbus as smb


class sysAgent():

    def __init__(self):
        #sensor init

        self.init=False
        #sensor self-check



        #sensor configure

        self.state=[]

#        return 0
    
    def getState(self):

        #sensor read


        #sensor value convert


        return self.state
    def shutdown():
        
        return 1


    def chgState(self,nextState):



        return 1








class mockAgent(sysAgent):

    def __init__(self,mess,dt,LogAgent=None):

        self.velo=20
        self.accel=0
        self.drag=0
        self.mess=10
        self.damp=0.01
        self.dt=0.001
        self.force=0
        self.state=[0,0,15,10]
        #loggind
        self.log=LogAgent
        self.localLOG=False
        if self.log:
            self.log.setVaris(["velo",'accel','drag','mass','force'])
    
    def initLocalLog(self):
        from datetime import datetime
        now = datetime.now()
        # dd/mm/YY H:M:S
        dtString = now.strftime("%d_%m_%Y_%H_%M_%S")
        self.logFile=open('log'+dtString+'.csv','w')
        self.logWriter=csv.writer(self.logFile)
        self.logWriter.writerow(["velo",'accel','drag','mass','force'])
        self.localLOG=True
        
    def getState(self):
        return self.state

    def impulse(self,input):#force

        #ckeck input format
        if input:
            if type(input)==list:# input from keyboard
                if (input[0]=='LEFT_KEY'):
                    input=-1000
                elif (input[0]=='RIGHT_KEY'):
                    input=1000
                elif (input[0]=='JOY'):
                    input=int(input[1])
                else:
                    input=0
        #prase input
        else:
            input = 0# no input

        self.force=input
        self.accel=(self.force-self.drag-self.velo*self.damp)/self.mess
        self.velo+=self.accel*self.dt
        self.state=[self.velo,self.accel,self.drag,self.mess]
        if self.log:
            self.log.logState(self.state+[self.force])
        if self.localLOG:
            self.logWriter.writerow(self.state+[self.force])

    #def logState(self):



if __name__ == '__main__':
    mck=mockAgent(10,0.01)
    print(mck.getState())
    mck.impulse(1)
    #print(mck.getState())
