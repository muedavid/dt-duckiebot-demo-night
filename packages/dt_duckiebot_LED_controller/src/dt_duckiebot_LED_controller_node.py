#!/usr/bin/env python
import time
import numpy as np
import rospy
from duckietown import DTROS
from duckietown_msgs.msg import SegmentList
from dt_duckiebot_LED_controller.msg import LEDscale

class Controller(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(Controller, self).__init__(node_name=node_name)
        
        #parameter
        # Add the node parameters to the parameters dictionary and load their default values
        self.parameters['~number_together'] = None
        self.parameters['~default_segment_day'] = None
        self.parameters['~k_I'] = None
        self.parameters['~k_p'] = None
        self.updateParameters()


        self.number_together=self.parameters['~number_together'] 
        self.segment_day=self.parameters['~default_segment_day'] 
        self.k_I= self.parameters['~k_I']
        self.k_p = self.parameters['~k_p']
        self.maximumscale = 0.8
        self.minimumscale = 0.4
        self.integrallist = [0]*10
        self.i=0
        self.j=0
        self.segment=0
        self.error = 0
        self.prev = 0
        self.integral = 0
        self.segmentarray = []
        self.calib_done = "false"
        
        
        #neuer publisher
        self.msg_LED_scale = LEDscale()
        self.pub=rospy.Publisher("LEDscale",LEDscale,queue_size=1)
        
        self.subcallibrator = self.subscriber("~segment_list",SegmentList, self.callibrator,queue_size=1)
        
        #wait until callibration is done.
        while self.calib_done == "false":
            rospy.sleep(0.5)

        #Subscriber
        self.sub = self.subscriber("~segment_list",SegmentList, self.Controller,queue_size=1)

    def Callibrator(self,data):
        if self.j < self.num_callibration*self.number_together:
            if self.i<self.number_together:
                self.i +=1
                self.segment += len(data.segments)
            elif self.i == self.number_together:
                self.i=0
                self.segmentarray.append(self.segment/self.number_together)
                self.j += 1
                rospy.loginfo(self.segment)
                self.segment = 0

        else: #now callibration is done and we can continue with our program
            self.segment_day = np.max(self.segmentarray)
            self.calib_done = "true"
            #stop this subscriber
            self.subcallibrator.unregister()
        
        
        
        
    def Controller(self,data):
        #We take the average of some segments together
        if self.i<self.number_together: 
            self.i +=1
            self.segment += len(data.segments)
            
        else:
            self.segment = self.segment/self.number_together

            #Controller 
            #normalized error
            self.error = 1.0/self.segment_day*(self.segment_day-self.segment)
            
            
            #we callibrate everything then the roomlight is off. If the roomlight is on self.error is strong negative.
            if self.error > 0:
                self.integrallist.append(self.error)
            else:
                self.integrallist.append(0)

            self.integral = np.sum(self.integrallist)

            #calculate the input and publish it to the LED
            self.msg_LED_scale = self.k_I*self.integral + self.k_p*self.error+self.prev
            
            if self.msg_LED_scale < self.minimumscale:
                self.msg_LED_scale = self.minimumscale
            
            elif self.msg_LED_scale > self.maximumscale:
                self.msg_LED_scale = self.maximumscale
            
            self.pub.publish(self.msg_LED_scale)
            

            #prev can get realy large number and to limit this: (like a saturation that we set)
            self.prev = self.msg_brightness
            if self.prev < 0:
                self.prev = 0


            rospy.loginfo("error is %d" %self.error)
            rospy.loginfo ("message is %d" %self.msg_brightness)
            rospy.loginfo ("Data ist %d " %self.segment)

            #set value to 0
            self.i=0
            self.segment = 0

    
if __name__ == '__main__':
    # create the node
    node = Controller(node_name='dt_duckiebot_LED_controller_node')
    rospy.spin()
