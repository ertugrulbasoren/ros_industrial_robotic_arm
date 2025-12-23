#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
GRADUATION PROJECT - GROUP 1
Role: Vision System Lead (Ertuğrul Başören)
Task: Multi-Class Waste Sorting (Red/Plastic vs Blue/Glass)
"""

import rospy
import sys
import threading
import tf
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

class SortingRobot:
    def __init__(self):
        rospy.init_node('week6_waste_sorting')
        
        # Görsel hazırlık
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.camera_callback)
        
        
        self.detected_type = None 
        self.is_busy = False      
        
        
        self.robot_pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
        self.tf_listener = tf.TransformListener()
        
        
        rospy.wait_for_service('/gazebo/set_model_state')
        self.box_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.magnet_active = False
        self.target_box_name = '' 
        
        
        self.worker = threading.Thread(target=self.magnet_thread)
        self.worker.daemon = True
        self.worker.start()

        print(">>> ATIK AYRIŞTIRMA SİSTEMİ BAŞLATILIYOR <<<")
        self.wait_for_connection()
        self.go_home_pose()
        
        print("\n" + "="*50)
        print(" BEKLENİYOR: Kırmızı (Plastik) veya Mavi (Cam) Kutu")
        print(" Robot rengi algılayıp ilgili sepete atacak.")
        print("="*50 + "\n")
        
        
        rospy.spin()

    def wait_for_connection(self):
        while self.robot_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)
        print(">>> Robot Hazır.")

    def camera_callback(self, data):
        """
        ÇOKLU RENK TESPİTİ (SORTING ALGORITHM)
        """
        if self.is_busy: return 

        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # --- 1. KIRMIZI 
            lower_red1 = np.array([0, 70, 50])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 70, 50])
            upper_red2 = np.array([180, 255, 255])
            mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
            
            # --- 2. MAVİ 
            lower_blue = np.array([100, 150, 0])
            upper_blue = np.array([140, 255, 255])
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

            
            kernel = np.ones((5,5), np.uint8)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

            
            self.draw_box(frame, mask_red, (0, 0, 255), "PLASTIK (Kirmizi)")
            self.draw_box(frame, mask_blue, (255, 0, 0), "CAM (Mavi)")
            
            cv2.imshow("AYRISTIRMA KAMERASI", frame)
            cv2.waitKey(1)

            
            if cv2.countNonZero(mask_red) > 3000:
                print(">>> TESPİT: KIRMIZI (Plastik) -> SOLA Gidecek")
                self.target_box_name = 'kutu_kirmizi'
                self.detected_type = 'RED'
                self.start_sorting_thread()

            elif cv2.countNonZero(mask_blue) > 3000:
                print(">>> TESPİT: MAVİ (Cam) -> SAĞA Gidecek")
                self.target_box_name = 'kutu_mavi' 
                self.detected_type = 'BLUE'
                self.start_sorting_thread()

        except Exception as e:
            pass

    def draw_box(self, frame, mask, color, text):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 3000:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), color, 3)
                cv2.putText(frame, text, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    def start_sorting_thread(self):
        """Operasyonu ana döngüyü kitlemeden başlatır"""
        self.is_busy = True
        t = threading.Thread(target=self.run_mission)
        t.start()

    def magnet_thread(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.magnet_active:
                try:
                    (trans, rot) = self.tf_listener.lookupTransform('/base_link', '/wrist_3_link', rospy.Time(0))
                    state = ModelState()
                    state.model_name = self.target_box_name 
                    state.reference_frame = 'base_link'
                    state.pose.position.x = trans[0]
                    state.pose.position.y = trans[1]
                    state.pose.position.z = trans[2] - 0.22 
                    state.pose.orientation.w = 1.0
                    self.box_service(state)
                except:
                    pass
            rate.sleep()

    def run_mission(self):
        
        self.move([-0.2, -1.57, 1.57, -1.57, -1.57, 0.0], 4.0)
        
        
        self.move([-0.2, -1.8, 2.1, -1.57, -1.57, 0.0], 4.0)
        rospy.sleep(0.5)
        
        print(f">>> VAKUM AÇILDI: {self.target_box_name} yakalandı.")
        self.magnet_active = True
        rospy.sleep(0.5)
        
       
        self.move([-0.2, -1.57, 1.57, -1.57, -1.57, 0.0], 3.0)
        
        
        if self.detected_type == 'RED':
            
            print(">>> Hedef: PLASTİK KONTEYNERİ (Sol)")
            self.move([1.57, -1.57, 1.57, -1.57, -1.57, 0.0], 5.0)
            
        elif self.detected_type == 'BLUE':
           
            print(">>> Hedef: CAM KONTEYNERİ (Sağ)")
            self.move([-1.57, -1.57, 1.57, -1.57, -1.57, 0.0], 5.0)

        
        self.move_to_drop() 
        
        print(">>> VAKUM KAPATILDI")
        self.magnet_active = False
        rospy.sleep(1.0)
        
       
        self.go_home_pose()
        self.is_busy = False 
        print(">>> SİSTEM HAZIR: Yeni atık bekleniyor...")

    def move_to_drop(self):
      
        current_joints = rospy.wait_for_message("/joint_states", JointTrajectoryPoint, timeout=1.0)
       
        
        base_angle = 1.57 if self.detected_type == 'RED' else -1.57
        self.move([base_angle, -1.2, 1.5, -1.57, -1.57, 0.0], 3.0)

    def go_home_pose(self):
        self.move([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], 4.0)

    def move(self, joints, duration):
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        p = JointTrajectoryPoint()
        p.positions = joints
        p.time_from_start = rospy.Duration(duration)
        traj.points.append(p)
        self.robot_pub.publish(traj)
        rospy.sleep(duration + 0.5)

if __name__ == '__main__':
    try:
        SortingRobot()
    except rospy.ROSInterruptException:
        pass
