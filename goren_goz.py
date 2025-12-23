#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RenkTanima:
    def __init__(self):
        # Düğümü başlat
        rospy.init_node('goruntu_isleme_node', anonymous=True)
        
        self.bridge = CvBridge()
        
        # Kameraya abone ol (World dosyasında tanımladığımız isim)
        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.kamera_callback)
        print("Kamera açıldı! Görüntü bekleniyor...")

    def kamera_callback(self, data):
        try:
            # 1. ROS Resmini -> OpenCV Resmine Çevir
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            print("Hata:", e)
            return

        # 2. Rengi Algıla (BGR -> HSV formatına geçiş)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Kırmızı renk için alt ve üst sınırlar (HSV'de kırmızı iki uçtadır)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # Maskeleme (Sadece kırmızıyı beyaz yap, gerisini siyah)
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        kirmizi_maske = mask1 + mask2

        # 3. Şekli Bul (Kırmızının etrafına kutu çiz)
        contours, _ = cv2.findContours(kirmizi_maske, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500: # Çok küçük lekeleri görmezden gel
                x, y, w, h = cv2.boundingRect(cnt)
                # Orijinal resme dikdörtgen çiz
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(cv_image, "KIRMIZI KUTU", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # 4. Pencereleri Göster
        cv2.imshow("Robotun Gozu (Orjinal)", cv_image)
        # cv2.imshow("Filtre (Sadece Kirmizi)", kirmizi_maske) # İstersen bunu da açabilirsin
        
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        RenkTanima()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
