#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def ayaga_kaldir():
    rospy.init_node('robotu_kaldir_node')
    
    # Robotun dinlediği konu (Topic)
    # Eğer çalışmazsa terminale 'rostopic list' yazıp kontrol ederiz
    pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    
    # Bağlantının kurulması için azıcık bekle
    rospy.sleep(1)

    print("Robot kontrolcüsüne bağlanıldı. Emir gönderiliyor...")

    # Hareket Mesajı Oluştur
    traj = JointTrajectory()
    # UR5'in eklem isimleri (Sırası önemli)
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    point = JointTrajectoryPoint()
    
    # HEDEF POZİSYON (Radyan cinsinden)
    # [0.0, -1.57, 0.0, -1.57, 0.0, 0.0] -> Dimdik yukarı (Lamba direği gibi)
    # Biz biraz daha "Masa Başı Çalışma" pozisyonu verelim:
    point.positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0] 
    
    # Bu hareketi 2 saniyede tamamla
    point.time_from_start = rospy.Duration(5.0)
    
    traj.points.append(point)
    
    # Gönder!
    pub.publish(traj)
    print("KALK komutu gönderildi! Robotun hareket etmesi lazım.")

if __name__ == '__main__':
    try:
        ayaga_kaldir()
    except rospy.ROSInterruptException:
        pass
