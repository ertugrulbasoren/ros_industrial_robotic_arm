#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import select
import termios
import tty

# --- KULLANIM KILAVUZU ---
# Q/A: Base (Sağ/Sol)
# W/S: Shoulder (Omuz Yukarı/Aşağı)
# E/D: Elbow (Dirsek)
# R/F: Wrist 1 (Bilek 1)
# T/G: Wrist 2
# Y/H: Wrist 3
# -------------------------

msg = """
ROBOT KALİBRASYON MODU
---------------------------
Robotu kutunun üzerine getirmek için şu tuşları kullan:

Q/A : Tabanı Döndür (Sağ/Sol)
W/S : Omuzu Hareket Ettir
E/D : Dirseği Hareket Ettir
R/F : Bilek 1
T/G : Bilek 2
Y/H : Bilek 3

Çıkış ve AÇILARI YAZDIRMAK için: CTRL+C
"""

moveBindings = {
    'q':(0, 0.1), 'a':(0, -0.1),
    'w':(1, 0.1), 's':(1, -0.1),
    'e':(2, 0.1), 'd':(2, -0.1),
    'r':(3, 0.1), 'f':(3, -0.1),
    't':(4, 0.1), 'g':(4, -0.1),
    'y':(5, 0.1), 'h':(5, -0.1),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('robot_kalibrasyon')
    pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)

    # Başlangıç pozisyonu (L şeklinde)
    joints = [-1.57, -1.57, 1.57, -1.57, -1.57, 0.0]
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    try:
        print(msg)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                index = moveBindings[key][0]
                change = moveBindings[key][1]
                joints[index] += change
                
                print(f"Güncel Açılar: {['{:.2f}'.format(x) for x in joints]}")

                traj = JointTrajectory()
                traj.joint_names = joint_names
                point = JointTrajectoryPoint()
                point.positions = joints
                point.time_from_start = rospy.Duration(0.5)
                traj.points.append(point)
                pub.publish(traj)
            
            elif key == '\x03': # CTRL+C
                break

    except Exception as e:
        print(e)

    finally:
        print("\n\n>>> BU AÇILARI KAYDET! <<<")
        print(f"[{', '.join(['{:.2f}'.format(x) for x in joints])}]")
        print(">>> Bu satırı kopyala ve robot_beyni.py içine yapıştır. <<<")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
