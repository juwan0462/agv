#!/usr/bin/env python3

import rospy

import cv2

from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge

​

def camera_publisher():

    rospy.init_node('camera_publisher', anonymous=True)

    pub = rospy.Publisher('/camera/image_raw/compressed', CompressedImage, queue_size=10)

    bridge = CvBridge()

​

    # 카메라 열기 (0: USB카메라)

    cap = cv2.VideoCapture(0)

​

    if not cap.isOpened():

        rospy.logerr("❌ 카메라 열기 실패")

        return

    rospy.loginfo("✅ 카메라 연결 성공 (Compressed Mode)")

​

    rate = rospy.Rate(30)  # 목표 30FPS

​

    while not rospy.is_shutdown():

        ret, frame = cap.read()

        if not ret:

            rospy.logwarn("⚠️ 프레임 읽기 실패")

            continue

​

        # JPEG 압축 전송

        msg = CompressedImage()

        msg.header.stamp = rospy.Time.now()

        msg.format = "jpeg"

        msg.data = cv2.imencode('.jpg', frame)[1].tobytes()

​

        pub.publish(msg)

        rate.sleep()

​

    cap.release()

    rospy.loginfo("✅ 카메라 종료")

​

if __name__ == '__main__':

    try:

        camera_publisher()

    except rospy.ROSInterruptException:

        pass