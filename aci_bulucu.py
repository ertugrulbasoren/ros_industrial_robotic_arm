#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys

def robotu_oynat():
    rospy.init_node('aci_bulucu_node')
    pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    
    # Başlangıç Tahmini (L Şekli)
    # [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3]
    joints = [-1.57, -1.57, 1.57, -1.57, -1.57, 0.0]
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    print("--- MANUEL AÇI GİRME MODU ---")
    print("Mevcut Açılar:", joints)
    print("Robotu hareket ettirmek için Enter'a bas (İlk pozisyona gider).")
    input() # Bekle

    # İlk pozisyonu gönder
    traj = JointTrajectory()
    traj.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = joints
    point.time_from_start = rospy.Duration(2.0)
    traj.points.append(point)
    pub.publish(traj)
    
    while not rospy.is_shutdown():
        try:
            print("\n--------------------------------")
            print(f"ŞU ANKİ AÇILAR: {joints}")
            print("Hangi eklemi değiştirmek istersin?")
            print("0: Taban (Sağ/Sol)")
            print("1: Omuz (İleri/Geri)")
            print("2: Dirsek (Aşağı/Yukarı)")
            print("3: Bilek 1 (Kafa Eğimi)")
            print("9: Çıkış ve Açıları Yazdır")
            
            secim = input("Seçiminiz (0-3 veya 9): ")
            
            if secim == '9':
                break
                
            if secim not in ['0', '1', '2', '3', '4', '5']:
                continue
                
            idx = int(secim)
            yeni_deger = float(input(f"Yeni açı gir (Şu anki: {joints[idx]}): "))
            
            # Listeyi güncelle
            joints[idx] = yeni_deger
            
            # Robota gönder
            traj = JointTrajectory()
            traj.joint_names = joint_names
            point = JointTrajectoryPoint()
            point.positions = joints
            point.time_from_start = rospy.Duration(1.0) # 1 saniyede git
            traj.points.append(point)
            pub.publish(traj)
            print("Robot hareket ediyor...")
            
        except ValueError:
            print("Lütfen geçerli bir sayı girin!")
        except KeyboardInterrupt:
            break

    print("\n\n>>> KAYDEDİLECEK DEĞERLER (Bunu kopyala) <<<")
    print(f"{joints}")

if __name__ == '__main__':
    try:
        robotu_oynat()
    except rospy.ROSInterruptException:
        pass
