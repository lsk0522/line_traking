#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys, select, termios, tty

bridge = CvBridge()
latest_frame = None

speed = 0.05          # 초기 직진 속도
turn_gain = 0.005     # 비례 제어(P-control) 게인
turn_gain_d = 0.015   # 미분 제어(D-control) 게인 (카메라 딜레이 보상)
prev_err = 0.0        # D-control용 이전 오차
roi_top = 0.8         # 카메라 높이(20cm)와 광각(120도) 고려 상하 크롭
roi_bottom = 0.9
roi_left = 0.0        # 양옆 흰색 바닥/벽 노이즈 방지용 좌우 크롭
roi_right = 1.0
pub = None

def callback(msg):
    global latest_frame
    try:
        np_arr = np.frombuffer(msg.data, np.uint8)
        latest_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    except Exception as e:
        pass

def get_key(timeout=0.0):
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        return sys.stdin.read(1)
    else:
        return ''

def tprint(msg):
    # 일반 로그 출력용 (줄바꿈 포함)
    sys.stdout.write("\r" + msg + " " * 30 + "\r\n")
    sys.stdout.flush()

def tstatus(msg):
    # 현재 상태 덮어쓰기 출력용 (줄바꿈 없음)
    sys.stdout.write("\r" + msg + " " * 30)
    sys.stdout.flush()

def main():
    global latest_frame, speed, turn_gain, turn_gain_d, prev_err, pub, roi_top, roi_bottom, roi_left, roi_right

    rospy.init_node('line_tracer_smooth_node')
    settings = termios.tcgetattr(sys.stdin)
    
    speed = rospy.get_param('~speed', 0.05)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/camera/image/compressed', CompressedImage, callback, queue_size=1, buff_size=2**24)

    tty.setraw(sys.stdin.fileno())
    
    tprint("=========================================")
    tprint(" 🏎️ 부드러운 모드 라인 트레이서 (연속 P 제어)")
    tprint("=========================================")
    tprint(" [W/S]: 직진 속도 증가 / 감소")
    tprint(" [A/D]: 회전 감도(Gain) 증가 / 감소")
    tprint(" [I/K]: 스캔 영역 위아래로 이동 (상하 조절)")
    tprint(" [J/L]: 스캔 영역 좁게/넓게 (좌우 바닥 노이즈 제거)")
    tprint(" [Q]  : 프로그램 종료")
    tprint("=========================================")

    # 30Hz로 멈춤 없이 계속 Twist를 퍼블리시
    rate = rospy.Rate(30)
    twist = Twist()

    try:
        while not rospy.is_shutdown():
            key = get_key(timeout=0.0)
            if key == 'w':
                speed = min(1.0, speed + 0.05)
                tprint(f">>> 🔼 직진 속도: {speed:.2f} m/s")
            elif key == 's':
                speed = max(0.0, speed - 0.05)
                tprint(f">>> 🔽 직진 속도: {speed:.2f} m/s")
            elif key == 'a':
                turn_gain = min(0.05, turn_gain + 0.001)
                turn_gain_d = turn_gain * 3.0
                tprint(f">>> 🔄 회전 감도(P-Gain) 증가: {turn_gain:.4f}")
            elif key == 'd':
                turn_gain = max(0.001, turn_gain - 0.001)
                turn_gain_d = turn_gain * 3.0
                tprint(f">>> 🔄 회전 감도(P-Gain) 감소: {turn_gain:.4f}")
            elif key == 'i':
                roi_top = max(0.1, roi_top - 0.05)
                roi_bottom = max(0.2, roi_bottom - 0.05)
                tprint(f">>> 🔼 스캔 영역 위로: {roi_top:.2f}~{roi_bottom:.2f}")
            elif key == 'k':
                roi_top = min(0.8, roi_top + 0.05)
                roi_bottom = min(0.9, roi_bottom + 0.05)
                tprint(f">>> 🔽 스캔 영역 아래로: {roi_top:.2f}~{roi_bottom:.2f}")
            elif key == 'j':
                roi_left = min(0.45, roi_left + 0.02)
                roi_right = max(0.55, roi_right - 0.02)
                tprint(f">>> ➡️⬅️ 스캔 영역 좁게 (바닥 무시): {roi_left:.2f}~{roi_right:.2f}")
            elif key == 'l':
                roi_left = max(0.0, roi_left - 0.02)
                roi_right = min(1.0, roi_right + 0.02)
                tprint(f">>> ⬅️➡️ 스캔 영역 넓게: {roi_left:.2f}~{roi_right:.2f}")
            elif key == 'q' or key == '\x03':
                break

            if latest_frame is None:
                rate.sleep()
                continue

            frame = latest_frame.copy()
            h, w = frame.shape[:2]
            
            crop_start = int(h * roi_top)
            crop_end = int(h * roi_bottom)
            w_start = int(w * roi_left)
            w_end = int(w * roi_right)
            cropped_frame = frame[crop_start:crop_end, w_start:w_end]

            # 화면에 현재 설정된 값 표시
            cv2.rectangle(frame, (w_start, crop_start), (w_end, crop_end), (0, 255, 255), 2)
            cv2.putText(frame, f"Spd: {speed:.2f} / Gain: {turn_gain:.4f}", (w_start, crop_start - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            hsv_cropped = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)

            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([179, 255, 255])
            red_mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
            
            lower_green = np.array([40, 100, 100])
            upper_green = np.array([80, 255, 255])
            green_mask = cv2.inRange(hsv, lower_green, upper_green)
            
            lower_blue = np.array([100, 100, 100])
            upper_blue = np.array([140, 255, 255])
            blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

            red_count = cv2.countNonZero(red_mask)
            green_count = cv2.countNonZero(green_mask)
            blue_count = cv2.countNonZero(blue_mask)

            # 조금 더 여유로운 흰색 인식 (그림자로 인해 선이 끊기는 현상 방지)
            lower_white = np.array([0, 0, 150])
            upper_white = np.array([180, 60, 255])
            white_mask = cv2.inRange(hsv_cropped, lower_white, upper_white)

            threshold = 5000

            # --- 부드러운 연속 제어 (비례 제어: P-control) ---
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            if red_count > threshold:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                tstatus("🔴 정지 (빨강 감지)")
            elif green_count > threshold:
                twist.linear.x = speed
                twist.angular.z = 0.0
                tstatus("🟢 직진 (초록 감지)")
            elif blue_count > threshold:
                twist.linear.x = -speed
                twist.angular.z = 0.0
                tstatus("🔵 후진 (파랑 감지)")
            else:
                # ========================================================
                # 💡 [핵심 알고리즘]: 바닥 노이즈 무시하고 중앙 차선만 잡는 로직
                # ========================================================
                # 1. 흰색 덩어리(윤곽선) 모두 찾기
                contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                h, w = white_mask.shape
                mid = w // 2
                
                left_contours = []
                right_contours = []
                
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area < 50: # 너무 작은 노이즈 무시
                        continue
                        
                    M_cnt = cv2.moments(cnt)
                    if M_cnt['m00'] > 0:
                        cx_cnt = int(M_cnt['m10'] / M_cnt['m00'])
                        if cx_cnt < mid:
                            left_contours.append((cx_cnt, area, cnt))
                        else:
                            right_contours.append((cx_cnt, area, cnt))
                
                cx_left = None
                cx_right = None
                
                # 왼쪽에서 가장 중앙(화면 가운데)에 가까운 선 1개만 선택 (화면 왼쪽 끝의 바닥 무시)
                if left_contours:
                    left_contours.sort(key=lambda x: x[0], reverse=True) # cx가 큰 순서 (중앙에 가까운 순)
                    cx_left = left_contours[0][0]
                    cv2.drawContours(frame, [left_contours[0][2]], -1, (255, 0, 0), 3, offset=(int(frame.shape[1]*roi_left), int(frame.shape[0]*roi_top)))
                    
                # 오른쪽에서 가장 중앙에 가까운 선 1개만 선택 (화면 오른쪽 끝의 바닥 무시)
                if right_contours:
                    right_contours.sort(key=lambda x: x[0]) # cx가 작은 순서 (중앙에 가까운 순)
                    cx_right = right_contours[0][0]
                    cv2.drawContours(frame, [right_contours[0][2]], -1, (0, 0, 255), 3, offset=(int(frame.shape[1]*roi_left), int(frame.shape[0]*roi_top)))

                # 타겟 중앙 좌표 계산
                if cx_left is not None and cx_right is not None:
                    cx = (cx_left + cx_right) // 2
                    tstatus("🔵 양쪽 차선 감지 완료")
                elif cx_left is not None:
                    cx = cx_left + int(w * 0.25) # 오른쪽 차선이 안 보일 때 추정치
                    tstatus("◀️ 왼쪽 차선만 감지됨")
                elif cx_right is not None:
                    cx = cx_right - int(w * 0.25) # 왼쪽 차선이 안 보일 때 추정치
                    tstatus("▶️ 오른쪽 차선만 감지됨")
                else:
                    cx = None
                    
                if cx is not None:
                    # 에러 계산 (화면 중앙과 타겟 중앙의 차이)
                    err = cx - mid
                    
                    # 시각화 (타겟 지점)
                    target_x_full = cx + int(frame.shape[1] * roi_left)
                    target_y_full = int(frame.shape[0] * roi_top) + (h // 2)
                    cv2.circle(frame, (target_x_full, target_y_full), 10, (0, 255, 255), -1)

                    twist = Twist()
                    twist.linear.x = speed
                    twist.angular.z = -float(err) * turn_gain - (err - prev_err) * turn_gain_d
                    
                    pub.publish(twist)
                    prev_err = err
                else:
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    pub.publish(twist)
                    prev_err = 0.0
                    tstatus("⚪ 차선 미감지 -> 정지 (대기)")

            pub.publish(twist)

            cv2.imshow("Camera View", frame)
            cv2.imshow("White Mask (Line Only)", white_mask)
            cv2.waitKey(1)
            
            rate.sleep()

    except Exception as e:
        tprint(f"\r\n오류 발생: {e}")
    finally:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        if pub: 
            pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        cv2.destroyAllWindows()
        print("\r\n프로그램이 안전하게 종료되었습니다.\r\n")

if __name__ == '__main__':
    main()
