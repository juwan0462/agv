#!/usr/bin/env python3
import rospy
import cv2
import time
import numpy as np
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import String

# YOLO 모델 로드
model = YOLO("/home/vboxuser/Downloads/best.pt")
bridge = CvBridge()

prev_time = time.time()

# Skip-tracking 변수
skip_counter = 0
SKIP_MAX = 45  # YOLO 실행 후 유지 프레임 수

# 마지막 YOLO 결과(박스 유지용)
last_annotated = None

# 빠른 후보 감지용 HSV 범위
red_lower = np.array([0, 120, 70])
red_upper = np.array([10, 255, 255])
yellow_lower = np.array([20, 100, 100])
yellow_upper = np.array([30, 255, 255])

# /cmd_vel 퍼블리셔 (String 타입)
decision_pub = rospy.Publisher('/cmd_vel', String, queue_size=10)

def detect_colors(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_red = cv2.inRange(hsv, red_lower, red_upper)
    mask_yellow = cv2.inRange(hsv, yellow_lower, yellow_upper)

    red_count = cv2.countNonZero(mask_red)
    yellow_count = cv2.countNonZero(mask_yellow)

    if red_count > 300:
        return "RIGHT"
    elif yellow_count > 300:
        return "LEFT"
    return None

def callback(msg):
    global prev_time, skip_counter, last_annotated

    frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

    detected = detect_colors(frame)

    if detected:  # 색이 확실히 감지된 경우
        rospy.loginfo(f"Detected color: {detected}")
        decision_pub.publish(detected)  # 문자열 발행 ("RIGHT" or "LEFT")
        cv2.destroyAllWindows()
        rospy.signal_shutdown("Color detected and published")
        return

    # (이 아래는 YOLO 표시용 - 필요 없지만 구조 유지)
    annotated = frame.copy()
    now = time.time()
    fps = 1.0 / (now - prev_time)
    prev_time = now
    cv2.putText(annotated, f"FPS: {fps:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
    cv2.imshow("YOLO Detection Optimized (Stage 3)", annotated)
    cv2.waitKey(1)

def listener():
    rospy.init_node('yolo_camera_subscriber', anonymous=True)
    rospy.Subscriber('/camera/image_raw/compressed', CompressedImage, callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    listener()

