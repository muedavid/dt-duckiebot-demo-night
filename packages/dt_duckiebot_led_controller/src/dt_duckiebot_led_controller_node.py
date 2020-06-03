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
        self.parameters['~segment_white_reference_LED'] = None
        self.parameters['~segment_reference_motor'] = None
        self.parameters['~segment_white_min_LED'] = None
        self.parameters['~segment_yellow_reference_LED'] = None
        self.parameters['~segment_yellow_min_LED'] = None
        self.parameters['~k_p_LED'] = None
        self.parameters['~k_p_motor'] = None
        self.parameters['~k_I_LED'] = None
        self.parameters['~k_I_motor'] = None
        self.updateParameters()


        self.number_average=self.parameters['~number_average'] 
        self.segment_white_reference_LED=self.parameters['~segment_white_reference_LED']
        self.segment_reference_motor = self.parameters['~segment_reference_motor']
        self.segment_white_min_LED=self.parameters['~segment_white_min_LED']
        self.segment_yellow_reference_LED=self.parameters['~segment_yellow_reference_LED']
        self.segment_yellow_min_LED=self.parameters['~segment_yellow_min_LED']
        self.k_p_LED = self.parameters['~k_p_LED']
        self.k_p_motor = self.parameters['~k_p_motor']
        self.k_I_LED = self.parameters['~k_I_LED']
        self.k_I_motor = self.parameters['~k_I_motor']

        self.error_LED = np.zeros([1,2])
        self.error_motor = 0.0
        self.error_array_LED = np.zeros([self.number_average,2])
        self.error_array_motor = [0.0]*self.number_average
        self.average_LED = [0.0]*2
        self.average_motor = 0.0
        self.integral_LED = [0.0]*2
        self.integral_motor = 0.0
        self.antiwindup_LED = [0.0]*2
        self.antiwindup_motor = 0.0
        self.prev_LED = [0.0]*2
        self.prev_motor = 0.0
        self.i=0.0

        #mode is only white or both chanel: "white", "both"
        self.mode = "both"
        
        
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
        
        #Duckiebot lighting system controller

        #normalized error with reference and minimum value of segments recognized if street lighting system is off
        self.error_LED[0,0] = 1.0/float(self.segment_white_reference_LED-self.segment_white_min_LED)*(self.segment_white_reference_LED-white)
        self.error_LED[0,1] = 1.0/float(self.segment_yellow_reference_LED-self.segment_yellow_min_LED)*(self.segment_yellow_reference_LED-yellow)

        #update error array
        self.error_array_LED = np.delete(self.error_array_LED, 0, axis=0)
        self.error_array_LED = np.append(self.error_array_LED, self.error_LED, axis=0)

        self.average_LED [0] = np.mean(self.error_array_LED[:,0])
        self.average_LED [1] = np.mean(self.error_array_LED[:,1])

        # integral for LED with antiwindup
        self.integral_LED[0] = self.integral_LED[0] +  self.average_LED [0] - self.antiwindup_LED[0]
        self.integral_LED[1] = self.integral_LED[1] +  self.average_LED [1] - self.antiwindup_LED[1]

        # PI controller
        self.msg.LEDscale_white = self.k_I_LED * self.integral_LED[0] + self.k_p_LED * self.average_LED[0]
        self.msg.LEDscale_yellow = self.k_I_LED * self.integral_LED[1] + self.k_p_LED * self.average_LED[1]

        #saturation
        self.prev_LED[0] = self.msg.LEDscale_white
        self.prev_LED[1] = self.msg.LEDscale_yellow
        if self.msg.LEDscale_white <= 0.3:
            self.msg.LEDscale_white = 0.3
        if self.msg.LEDscale_white >= 1:
            self.msg.LEDscale_white = 1
        if self.msg.LEDscale_yellow <= 0.3:
            self.msg.LEDscale_yellow = 0.3
        if self.msg.LEDscale_yellow >= 1:
            self.msg.LEDscale_yellow = 1
        
        #antiwindup
        self.antiwindup_LED[0] = self.prev_LED[0] - self.msg.LEDscale_white
        self.antiwindup_LED[1] = self.prev_LED[1] - self.msg.LEDscale_yellow

        #if mode is white: set message for yellow to the value fo white
        if self.mode == "white":
            self.msg.LEDscale_yellow = self.msg.LEDscale_white
        

        #Controller for wheels driver node

        #normalized error
        self.error_motor=1.0/float(self.segment_reference_motor)*(self.segment_reference_motor-white-yellow)

        #update error array
        self.error_array_motor.pop(0)
        self.error_array_motor.append(self.error_motor)

        #average
        self.average_motor = np.mean (self.error_array_motor)

        # integral for LED with antiwindup
        self.integral_motor += self.average_motor - self.antiwindup_motor

        #PI controller
        self.msg.motorscale = self.k_I_motor * self.integral_motor + self.k_p_motor * self.average_motor

        #saturation
        self.prev_motor = self.msg.motorscale
        if self.msg.motorscale >=1:
            self.msg.motorscale = 1
        elif self.msg.motorscale <= 0:
            self.msg.motorscale = 0

        #antiwindup
        self.antiwindup_motor = self.prev_motor - self.msg.motorscale

        #if error is large: scale should be 0 and if error is 0 scale should be one:
        self.msg.motorscale = 1.0 - self.msg.motorscale
        
        self.pub.publish(self.msg)
            

        self.log("average is")
        self.log (self.average_LED)
        self.log ("error for motorscale %s" %self.average_motor)
        self.log("message is for white %s" %self.msg.LEDscale_white)
        self.log("message is for yellow %s" %self.msg.LEDscale_yellow)
        self.log("message for motorscale %s" %self.msg.motorscale)
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

            self.error_LED = 1.0/float(self.segment_day)*(self.segment_day-self.segment)

            self.integral += self.error_LED - self.antiwindup

            self.msg.LEDscale = self.k_I*self.integral + self.k_p_LED*self.error_LED

            #saturation
            self.prev = self.msg.LEDscale
            if self.msg.LEDscale <= 0.3:
                self.msg.LEDscale = 0.3
            elif self.msg.LEDscale >= 1:
                self.msg.LEDscale = 1
            
            #antiwindup
            self.antiwindup = self.prev - self.msg.LEDscale


            #motor control:
            if self.error_LED >= 1:
                self.msg.motorscale = 0.5
            else:
                self.msg.motorscale = 1
            
            self.pub.publish(self.msg)
            

            rospy.loginfo("error is %s" %self.error_LED)
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
