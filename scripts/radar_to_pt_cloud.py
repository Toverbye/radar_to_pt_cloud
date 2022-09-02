#!/usr/bin/env python3
from os import SCHED_BATCH
from turtle import st
import rospy
from sensor_msgs.msg import Image
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2
from vn300.msg import ins
import tf.transformations 
import collections

class radar_to_pt:

    def __init__(self):

        self.range_resolution = 0.175

        self.scan_duration = rospy.Duration.from_sec(0.250)

        self.threshold = 80.0

        self.offset = 19

        self.radar_frame = 'aft_mapped' #os_lidar

        self.pt_cloud_pub = rospy.Publisher("~radar_pointcloud", PointCloud2, queue_size = 1)

        self.rotation_data = collections.deque()

        rospy.Subscriber("/talker1/Navtech/Polar", Image, self.radar_callback,queue_size = 1)
        rospy.Subscriber("/vectornav/ins", ins, self.ins_callback)

        rospy.spin()


    def radar_callback(self, data):
        intensity = ros_numpy.image.image_to_numpy(data).astype(np.float32)

        intensity = intensity[:,self.offset:]

        r, theta = np.meshgrid(np.linspace(self.offset * self.range_resolution,self.range_resolution * (intensity.shape[1]+self.offset),intensity.shape[1],dtype=np.float32), np.linspace(np.pi*2.0 + np.pi,0.0 + np.pi,intensity.shape[0],endpoint=False,dtype=np.float32))

        x = np.cos(theta) * r
        y = np.sin(theta) * r
        z = np.zeros(intensity.shape)

        x[intensity < self.threshold] = None
        y[intensity < self.threshold] = None
        z[intensity < self.threshold] = None
        intensity[intensity < self.threshold] = None

        x1 = np.zeros(intensity.shape,dtype=np.float32)
        y1 = np.zeros(intensity.shape,dtype=np.float32)
        z1 = np.zeros(intensity.shape,dtype=np.float32)

        scan_start_time = data.header.stamp - self.scan_duration

        i = 0
        if self.rotation_data[0][0] < scan_start_time:   #check time of oldests element

            j = 0
            while(self.rotation_data[j][0]<data.header.stamp):
                j+=1

            t = self.inv_lerp(data.header.stamp,self.rotation_data[j-1][0],self.rotation_data[j][0])

            ending_orentation = tf.transformations.quaternion_slerp(self.rotation_data[j-1][1],self.rotation_data[j][1],t)
            #ending_orentation_inv = ending_orentation.copy()
            #ending_orentation_inv[3] = -ending_orentation_inv[3]

            last = None
            oldest = self.rotation_data.popleft()
            while(oldest[0] < scan_start_time):
                last = oldest
                oldest = self.rotation_data.popleft()

            times = np.linspace(scan_start_time.to_sec(),data.header.stamp.to_sec(),intensity.shape[0])
            i = intensity.shape[0] - 1
            

            for line_time in times:

                line_time = rospy.Time.from_sec(line_time)

                while(oldest[0] < line_time):
                    last = oldest
                    oldest = self.rotation_data.popleft()

                t = self.inv_lerp(line_time, last[0], oldest[0])
                
                #orentation = tf.transformations.quaternion_multiply (tf.transformations.quaternion_slerp(last[1],oldest[1],t) , ending_orentation)
                orentation = tf.transformations.quaternion_slerp(last[1],oldest[1],t)
                orentation[3] = - orentation[3]
                orentation = tf.transformations.quaternion_multiply(orentation, ending_orentation)
                r = tf.transformations.quaternion_matrix(orentation)
                
                x1[i,:] = r[0,0] * x[i,:] + r[0,1] * y[i,:] 
                y1[i,:] = r[1,0] * x[i,:] + r[1,1] * y[i,:] 
                z1[i,:] = r[2,0] * x[i,:] + r[2,1] * y[i,:] 



                #[x]   [r1 r2 r3]   [x]
                #[y] = [r4 r5 r6] * [y]
                #[z]   [r7 r8 r9]   [z]

                # x = r1*x + r2*y + r3*z

                i -= 1


            ## Publish pointcloud
            out_pc = np.core.records.fromarrays([x1,y1,z1,intensity],names='x,y,z,intensity')
        
            self.pt_cloud_pub.publish(ros_numpy.point_cloud2.array_to_pointcloud2(out_pc, data.header.stamp, self.radar_frame))


    def ins_callback(self, data):
        #print(data.header.stamp)
        

        #q = tf.transformations.quaternion_from_euler(data.RPY.x, data.RPY.y, data.RPY.z)
        
        q = tf.transformations.quaternion_from_euler(np.radians(data.RPY.x), np.radians(-data.RPY.y), np.radians(-data.RPY.z))
        self.rotation_data.append((data.header.stamp,q))
        #print(q)
        

    def inv_lerp(self, x, a, b):
        return (x - a) / (b - a) 


if __name__ == '__main__':
    try:
        rospy.init_node("radar_to_pt_cloud")
        node = radar_to_pt()

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass