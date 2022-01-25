#!/usr/bin/env python

import time
import array
from math import pi,cos,sin
from hiddriver import hiddriver

import tf2_ros
import rospy
import geometry_msgs.msg
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped


FREQUENCY = 4
deltaT = 1.0/float(FREQUENCY)


class motordriver():
    def __init__(self,modulename="SinusoidalBLDC"):
        self.modulename = modulename
        rospy.init_node(modulename)
        self.motorhid = hiddriver(modulename)
        self.error_pub = rospy.Publisher('motordriver/error', String, queue_size=10)
        self.errorCode_pub = rospy.Publisher('motordriver/errorcode', UInt8, queue_size=10)
        self.control_command_counter = 0
        if not (self.motorhid.productFound):
            data = "%s not found1"%(modulename)
            rospy.loginfo(data)
            self.error_pub.publish(data)
            self.errorcode = 0
            self.old_errorcode = 0
            self.module_ready = False
            return
        rospy.loginfo("%s motor driver started"%(modulename))
        self.module_ready = True
        self.errorcode = 0
        self.old_errorcode = 0

        self.enc_x = 0
        self.enc_y = 0
        self.enc_th = 0

        self.enc_x_vel = 0
        self.enc_y_vel = 0
        self.enc_thetaVel = 0 
        
        self.stamp_old = rospy.get_time()
        self.desiredxvel = 0
        self.desiredthvel = 0
        self.desiredspeedright = 0
        self.desiredspeedleft = 0
        self.motorvoltage = 0
        self.motorcurrent1 = 0
        self.motorcurrent2 = 0
        self.control_command = "ER                            "

        rospy.Subscriber('/cmd_vel', Twist, self.on_cmd_vel)
        self.twist_pub = rospy.Publisher('cmdvel_echo', Twist, queue_size=10)
        self.voltage_pub = rospy.Publisher('motordriver/voltage', Float32, queue_size=10)
        self.current1_pub = rospy.Publisher('motordriver/current1', Float32, queue_size=10)
        self.current2_pub = rospy.Publisher('motordriver/current2', Float32, queue_size=10)
        self.temp1_pub = rospy.Publisher('motordriver/temp1', Float32, queue_size=10)
        self.temp2_pub = rospy.Publisher('motordriver/temp2', Float32, queue_size=10)

        self.publish_tf = rospy.get_param('~publish_tf',default=False)
        rospy.loginfo("%s is %s", rospy.resolve_name('~publish_tf'), self.publish_tf)

        self.publish_odom = rospy.get_param('~publish_odom',default=False)
        rospy.loginfo("%s is %s", rospy.resolve_name('~publish_odom'), self.publish_odom)

        odom_name = rospy.get_param('~odom_name',default='/odom')
        rospy.loginfo("%s is %s", rospy.resolve_name('~odom_name'), odom_name) 

        self.BASE_WIDTH = rospy.get_param('~BASE_WIDTH',default=0.50582)
        rospy.loginfo("%s is %s", rospy.resolve_name('~BASE_WIDTH'), self.BASE_WIDTH)

        self.WHEEL_DIAMETER = rospy.get_param('~WHEEL_DIAMETER',default=0.2)
        rospy.loginfo("%s is %s", rospy.resolve_name('~WHEEL_DIAMETER'), self.WHEEL_DIAMETER)

        self.GEAR_RATIO = rospy.get_param('~GEAR_RATIO',default=1.0)
        rospy.loginfo("%s is %s", rospy.resolve_name('~GEAR_RATIO'), self.GEAR_RATIO)

        self.CHAIN_RATIO = rospy.get_param('~CHAIN_RATIO',default=(1.0))
        rospy.loginfo("%s is %s", rospy.resolve_name('~CHAIN_RATIO'), self.CHAIN_RATIO)

        self.ENCODER_COUNT = rospy.get_param('~ENCODER_COUNT',default=90.0)
        rospy.loginfo("%s is %s", rospy.resolve_name('~ENCODER_COUNT'), self.ENCODER_COUNT)

        if(self.publish_odom):
            self.odometry_pub = rospy.Publisher(odom_name, Odometry, queue_size=10)

        if(self.publish_tf):
            self.odom_broadcaster = tf2_ros.TransformBroadcaster()



    def motor_control(self, control_msg):
        if(control_msg == "CLEAR_ENCODERS"):
            self.control_command = "EN "
        if(control_msg == "CLEAR_ERRORS"):
            self.errorcode = 0
            self.module_ready = True
            self.control_command = "ER "

    def on_cmd_vel(self,data):
        if(self.errorcode == 0):
            self.desiredxvel = data.linear.x
            self.desiredthvel = data.angular.z * float(self.BASE_WIDTH/2.0)
            self.desiredspeedright = float(self.desiredxvel+self.desiredthvel)
            self.desiredspeedleft = float(self.desiredxvel-self.desiredthvel)
        else:
            if(self.old_errorcode!=self.errorcode):
                self.old_errorcode = self.errorcode
                rospy.logerr("Motor Error %d"%(self.errorcode))
                data = String()
                data.data = "Motor Error %d"%(self.errorcode)
                self.error_pub.publish(data)
                self.desiredspeedright = 0
                self.desiredspeedleft = 0

                if(self.errorcode==1):#Inst Curr 1
                    data = UInt8()
                    data.data = 3
                    self.errorCode_pub.publish(data)
                elif(self.errorcode==2):#Inst Curr 2
                    data = UInt8()
                    data.data = 6
                    self.errorCode_pub.publish(data)
                elif(self.errorcode==4):#Battery Voltage
                    data = UInt8()
                    data.data = 1
                    self.errorCode_pub.publish(data)
                elif(self.errorcode==8):#Direnme Curr 1
                    data = UInt8()
                    data.data = 2
                    self.errorCode_pub.publish(data)
                elif(self.errorcode==16):#Direnme Curr 2
                    data = UInt8()
                    data.data = 5
                    self.errorCode_pub.publish(data)
                elif(self.errorcode==32):#Motor Temp 1
                    data = UInt8()
                    data.data = 4
                    self.errorCode_pub.publish(data)
                elif(self.errorcode==64):#Motor Temp 2
                    data = UInt8()
                    data.data = 7
                    self.errorCode_pub.publish(data)
                else:#Comm
                    data = UInt8()
                    data.data = 8
                    self.errorCode_pub.publish(data)
                    
        Twist_Echo_Data = Twist()
        Twist_Echo_Data.linear.x = self.desiredxvel
        Twist_Echo_Data.angular.z = self.desiredthvel
        self.twist_pub.publish(Twist_Echo_Data)

    
        

    def send_speed(self):
        if(self.control_command == ""):
            speed1 = -float(self.desiredspeedright) #cm/sn => (1 rpm / 60 ) * pi * WHEEL_DIAMETER * 100   negative for reversed motor  
            speed2 = float(self.desiredspeedleft) #cm/sn => (1 rpm / 60 ) * pi * WHEEL_DIAMETER * 100       
            rpmSpeed1 = int(((self.CHAIN_RATIO*self.GEAR_RATIO*speed1*60.0)/(pi*self.WHEEL_DIAMETER)))
            rpmSpeed2 = int(((self.CHAIN_RATIO*self.GEAR_RATIO*speed2*60.0)/(pi*self.WHEEL_DIAMETER)))
            #S=+0000,-0000
            command = ""
            if(rpmSpeed1 >= 0):
                command1 = "+%04d"%(rpmSpeed1)
            else:
                command1 = "%05d"%(rpmSpeed1)
            if(rpmSpeed2 >= 0):
                command2 = "+%04d"%(rpmSpeed2)
            else:
                command2 = "%05d"%(rpmSpeed2)
            command = "S="+command2+","+command1+"\r\n" 
        else:
            rospy.loginfo("%s motor control command"%(self.control_command))
            command = self.control_command
            self.control_command_counter = self.control_command_counter + 1
            if(self.control_command_counter>9):
                self.control_command_counter = 10
                self.control_command = ""
            self.old_errorcode = 0
            self.errorcode = 0
        self.motorhid.write_device(command)
        #print(command)

    def shutdown(self):
        self.desiredxvel = 0 
        self.desiredthvel = 0
        self.desiredspeedright = float(self.desiredxvel+self.desiredthvel)
        self.desiredspeedleft = float(self.desiredxvel-self.desiredthvel)
        if (self.module_ready):
            self.control_command = ""
            self.send_speed()

    def mod_pi(self,angle):
        while(angle > (2*pi)):
            angle = angle-2*pi
        while(angle < 0):
            angle = angle+2*pi
        return (angle)

    def module_info(self):
        if(self.module_ready==False):
            data = String()
            data.data = "%s module not found"%(self.modulename)
            self.error_pub.publish(data)
    
    def odom_publisher(self):
        if(self.publish_odom):
                
            quaternionX = quaternion_from_euler(0, 0, self.mod_pi(self.enc_th))
            quaternion = Quaternion()
            quaternion.x = quaternionX[0]
            quaternion.y = quaternionX[1]
            quaternion.z = quaternionX[2]
            quaternion.w = quaternionX[3] 


            self.odometry = Odometry()
            self.odometry.header.frame_id = "odom"
            self.odometry.header.stamp = rospy.Time.now()
            self.odometry.child_frame_id = "base_link"
            self.odometry.pose.pose.position.x = self.enc_x
            self.odometry.pose.pose.position.y = self.enc_y
            self.odometry.pose.pose.position.z = 0
            self.odometry.pose.pose.orientation = quaternion
            
            self.odometry.twist.twist.linear.x = self.enc_x_vel
            self.odometry.twist.twist.linear.y = 0
            self.odometry.twist.twist.linear.z = 0
            self.odometry.twist.twist.angular.x = 0
            self.odometry.twist.twist.angular.y = 0
            self.odometry.twist.twist.angular.z = self.enc_thetaVel
            self.odometry_pub.publish(self.odometry)

    def transform_publisher(self):
        if(self.publish_tf):
                
            quaternionX = quaternion_from_euler(0, 0, self.mod_pi(self.enc_th))
            quaternion = Quaternion()
            quaternion.x = quaternionX[0]
            quaternion.y = quaternionX[1]
            quaternion.z = quaternionX[2]
            quaternion.w = quaternionX[3] 
                    
            t = geometry_msgs.msg.TransformStamped()

            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.enc_x
            t.transform.translation.y = self.enc_y
            t.transform.translation.z = 0.0
            t.transform.rotation = quaternion
                    
            self.odom_broadcaster.sendTransform(t)


    def process_data(self):
        datastr = self.motorhid.read_device(64)
        if(datastr == ""):
            #self.errorcode = 128 
            self.module_ready = False 
        else:
            if("D" in datastr):
                #localHallCounter,globalHallCounter,packageNumber,Current,board_temperature,Voltage,globalError
                partialsplit = datastr.split("=")[1].split(",")
                #print(datastr)
                self.dataready=True
                self.stamp = rospy.get_time()-self.stamp_old
                self.stamp_old = rospy.get_time()
                self.rpm2 = int(partialsplit[0])#inverse
                self.rpm1 = -1*int(partialsplit[1])#negetive for reversed motor
                self.motorvoltage = float(partialsplit[2])
                self.motorcurrent1 = float(partialsplit[3])
                self.motorcurrent2 = float(partialsplit[4])
                self.errorcode = int(partialsplit[5])
                self.motortemp1 = float(partialsplit[6])
                self.motortemp2 = float(partialsplit[7].split("\r")[0])
                self.travel1 = (self.rpm1 / (self.ENCODER_COUNT * self.GEAR_RATIO * self.CHAIN_RATIO)) * pi * self.WHEEL_DIAMETER
                self.travel2 = (self.rpm2 / (self.ENCODER_COUNT * self.GEAR_RATIO * self.CHAIN_RATIO)) * pi * self.WHEEL_DIAMETER
                righttravel=self.travel1
                lefttravel=self.travel2
                deltatravel = (righttravel + lefttravel) / 2.0
                deltatheta = (righttravel - lefttravel) / (self.BASE_WIDTH)
                if righttravel == lefttravel:
                    deltax = lefttravel*cos(self.enc_th)
                    deltay = lefttravel*sin(self.enc_th)
                else:
                    radius = deltatravel / deltatheta
                    ##Find the instantaneous center of curvature (ICC).
                    iccX = self.enc_x - radius*sin(self.enc_th)
                    iccY = self.enc_y + radius*cos(self.enc_th) 

                    deltax = cos(deltatheta)*(self.enc_x - iccX) \
                        - sin(deltatheta)*(self.enc_y - iccY) \
                        + iccX - self.enc_x
                    deltay = sin(deltatheta)*(self.enc_x - iccX) \
                        + cos(deltatheta)*(self.enc_y - iccY) \
                        + iccY - self.enc_y  
                
                self.enc_x += deltax
                self.enc_y += deltay
                self.enc_th = self.mod_pi(self.enc_th + deltatheta)
                self.enc_x_vel = deltatravel / deltaT
                self.enc_y_vel = deltay / deltaT
                self.enc_thetaVel = deltatheta / deltaT
                motVoltage = Float32()
                motVoltage.data = self.motorvoltage
                motCurrent1 = Float32()
                motCurrent1.data = self.motorcurrent1
                motCurrent2 = Float32()
                motCurrent2.data = self.motorcurrent2
                motortemp1 = Float32()
                motortemp1.data = self.motortemp1
                motortemp2 = Float32()
                motortemp2.data = self.motortemp1
                self.voltage_pub.publish(motVoltage)
                self.current1_pub.publish(motCurrent1)
                self.current2_pub.publish(motCurrent2)
                self.temp1_pub.publish(motortemp1)
                self.temp2_pub.publish(motortemp2)
                self.odom_publisher()
                self.transform_publisher()
                self.send_speed()



if __name__ == '__main__':
    md = motordriver("SinusoidalBLDC")
    rospy.on_shutdown(md.shutdown)
    rate = rospy.Rate(8) # for 4 Hz dual check for sync
    while not rospy.is_shutdown():
        if(md.module_ready):
            md.process_data()
        else:
            time.sleep(5)
            md.module_info()
            time.sleep(5)
            del md
            md = motordriver("SinusoidalBLDC")
        rate.sleep()
