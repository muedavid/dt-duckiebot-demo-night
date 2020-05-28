#!/usr/bin/env python
import time
import numpy as np
import rospy
from duckietown import DTROS
from duckietown_msgs.msg import SegmentList
from dt_duckiebot_led_controller.msg import Light_Adjustment

class Controller(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(Controller, self).__init__(node_name=node_name)
        
        #parameter
        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters['~number_average'] = None
        self.parameters['~segment_white_reference'] = None
        self.parameters['~segment_white_min'] = None
        self.parameters['~segment_yellow_reference'] = None
        self.parameters['~segment_yellow_min'] = None
        self.parameters['~k_p'] = None
        self.updateParameters()


        self.number_average=self.parameters['~number_average'] 
        self.segment_white_reference=self.parameters['~segment_white_reference']
        self.segment_white_min=self.parameters['~segment_white_min']
        self.segment_yellow_reference=self.parameters['~segment_yellow_reference']
        self.segment_yellow_min=self.parameters['~segment_yellow_min']
        self.k_p = self.parameters['~k_p']
        self.error = np.zeros([1,2])
        self.error_array = np.zeros([self.number_average,2])
        self.average = [0]*2
        self.i=0
        
        
        #neuer publisher
        self.msg = Light_Adjustment()
        self.pub=rospy.Publisher("Light_Adjustment",Light_Adjustment,queue_size=1)
               

        #Subscriber
        self.sub = self.subscriber("~segment_list",SegmentList, self.Controller,queue_size=1)

        
    def Controller(self,data):
        white = 0
        yellow = 0

        #take number of recognized segments
        for i in range(len(data.segments)):
            if data.segments[i].color == 0:
                white += 1
            if data.segments[i].color == 1:
                yellow += 1        

        #normalized error with reference and minimum value of segments recognized if street lighting system is off
        self.error[0,0] = 1.0/float(self.segment_white_reference-self.segment_white_min)*(self.segment_white_reference-white)
        self.error[0,1] = 1.0/float(self.segment_yellow_reference-self.segment_yellow_min)*(self.segment_yellow_reference-yellow)

        #update error array
        self.error_array = np.delete(self.error_array, 0, axis=0)
        self.error_array = np.append(self.error_array, self.error, axis=0)

        self.average [0] = sum(self.error_array[:,0])/float(self.number_average)
        self.average [1] = sum(self.error_array[:,1])/float(self.number_average)


        #saturation
        for i in range (2):
            if self.average[i] <=0.3:
                self.average[i] = 0.3
            elif self.average[i] >= 1:
                self.average[i] = 1
        
        #P controller
        self.msg.LEDscale_white = self.average[0]
        self.msg.LEDscale_yellow = self.average[1]


        #motor control if error for 1 color is very large drive slower:
        if self.average [0] >= 0.8:
            self.msg.motorscale = 0.7
        else:
            self.msg.motorscale = 1.0
        

        
        # for average 10 measurements need to bed done first of all
        if self.i >= self.number_average:
            self.pub.publish(self.msg)
            

            self.log("average is")
            self.log (self.average)
            self.log("message is for white %s" %self.msg.LEDscale_white)
            self.log("message is for yellow %s" %self.msg.LEDscale_yellow)
            self.log("Data ist for white %s " %white)
            self.log("Data ist for yellow %s " %yellow)
            
        self.i+=1
        
    
    """
    def Controller_PI(self,data):
        #We take the average of some segments together
        if self.i<self.number_together: 
            self.i +=1
            white = 0
            for i in range(len(data.segments)):#look only at the white color
                if data.segments[i].color == 0:
                    white += 1
            self.segment += white
            
        else:
            self.segment = self.segment/self.number_together

            self.error = 1.0/float(self.segment_day)*(self.segment_day-self.segment)

            self.integral += self.error - self.antiwindup

            self.msg.LEDscale = self.k_I*self.integral + self.k_p*self.error

            #saturation
            self.prev = self.msg.LEDscale
            if self.msg.LEDscale <= 0.3:
                self.msg.LEDscale = 0.3
            elif self.msg.LEDscale >= 1:
                self.msg.LEDscale = 1
            
            #antiwindup
            self.antiwindup = self.prev - self.msg.LEDscale


            #motor control:
            if self.error >= 1:
                self.msg.motorscale = 0.5
            else:
                self.msg.motorscale = 1
            
            self.pub.publish(self.msg)
            

            rospy.loginfo("error is %s" %self.error)
            rospy.loginfo ("message is %s" %self.msg.LEDscale)
            self.log("prev is %s" %self.prev)
            rospy.loginfo ("Data ist %s " %self.segment)

            #set value to 0
            self.i=0
            self.segment = 0
    """
    
if __name__ == '__main__':
    # create the node
    node = Controller(node_name='dt_duckiebot_led_controller_node')
    rospy.spin()
