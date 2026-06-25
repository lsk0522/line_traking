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
speed = 0.2
turn_speed = 0.8  # 회전 스캔 기본 속도를 높임 (0.4 -> 0.8)
pub = None

# ROI 기본 위치 - 카메라 높이 20cm, 120도 광각 고려 상하좌우 크롭 (주변 흰색 노이즈 방지)
roi_top = 0.5
roi_bottom = 0.8
roi_left = 0.2
roi_right = 0.8

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
    sys.stdout.write(msg + "\r\n")
    sys.stdout.flush()

def sleep_and_check_keys(duration):
    global speed, turn_speed, roi_top, roi_bottom
    end_time = rospy.Time.now() + rospy.Duration(duration)
    while rospy.Time.now() < end_time and not rospy.is_shutdown():
        key = get_key(timeout=0.05)
        if key == 'w':
            speed = min(1.0, speed + 0.05)
            tprint(f">>> 직진 속도 증가: {speed:.2f} m/s")
        elif key == 's':
            speed = max(0.0, speed - 0.05)
            tprint(f">>> 직진 속도 감소: {speed:.2f} m/s")
        elif key == 'a':
            turn_speed = min(2.0, turn_speed + 0.1)
            tprint(f">>> 🔄 회전 스캔 량(각도) 증가: {turn_speed:.2f} rad/s")
        elif key == 'd':
            turn_speed = max(0.1, turn_speed - 0.1)
            tprint(f">>> 🔄 회전 스캔 량(각도) 감소: {turn_speed:.2f} rad/s")
        elif key == 'i':
            roi_top = max(0.1, roi_top - 0.05)
            roi_bottom = max(0.2, roi_bottom - 0.05)
            tprint(f">>> 🔼 스캔 영역 위로 이동 ({roi_top:.2f} ~ {roi_bottom:.2f})")
        elif key == 'k':
            roi_top = min(0.8, roi_top + 0.05)
            roi_bottom = min(0.9, roi_bottom + 0.05)
            tprint(f">>> 🔽 스캔 영역 아래로 이동 ({roi_top:.2f} ~ {roi_bottom:.2f})")
        elif key == 'q' or key == '\x03':  # Ctrl+C
            rospy.signal_shutdown("Quit")
            break
        cv2.waitKey(1)

def stop_robot():
    twist = Twist()
    pub.publish(twist)

def move_robot(linear_x, angular_z):
    twist = Twist()
    twist.linear.x = linear_x
    twist.angular.z = angular_z
    pub.publish(twist)

def main():
    global latest_frame, speed, turn_speed, pub, roi_top, roi_bottom, roi_left, roi_right

    rospy.init_node('line_tracer_turtle_node')
    settings = termios.tcgetattr(sys.stdin)
    
    speed = rospy.get_param('~speed', 0.2)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/camera/image/compressed', CompressedImage, callback, queue_size=1, buff_size=2**24)

    wait_time = 1.0
    move_time = 0.3

    tty.setraw(sys.stdin.fileno())
    
    tprint("=========================================")
    tprint(" 🐢 거북이 모드 라인 트레이서 (높은 카메라/광각 대응)")
    tprint("=========================================")
    tprint(" [W/S]: 직진 속도 증가 / 감소")
    tprint(" [A/D]: 회전 스캔 각도 증가 / 감소")
    tprint(" [I/K]: 스캔 영역(노란 박스) 위로 / 아래로 이동")
    tprint(" [Q]  : 프로그램 종료")
    tprint("=========================================")

    try:
        while not rospy.is_shutdown():
            stop_robot()
            sleep_and_check_keys(wait_time)
            
            if rospy.is_shutdown():
                break

            if latest_frame is None:
                continue

            frame = latest_frame.copy()
            h, w = frame.shape[:2]
            
            crop_start = int(h * roi_top)
            crop_end = int(h * roi_bottom)
            w_start = int(w * roi_left)
            w_end = int(w * roi_right)
            cropped_frame = frame[crop_start:crop_end, w_start:w_end]

            # 스캔 영역 시각화
            cv2.rectangle(frame, (w_start, crop_start), (w_end, crop_end), (0, 255, 255), 2)
            cv2.putText(frame, "Scan Area (I/K to move)", (w_start, crop_start - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

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

            # 조금 더 여유로운 흰색 임계값 (그림자로 인해 안 보이는 현상 방지)
            lower_white = np.array([0, 0, 150])
            upper_white = np.array([180, 60, 255])
            white_mask = cv2.inRange(hsv_cropped, lower_white, upper_white)

            threshold = 5000
            action_desc = ""

            if red_count > threshold:
                action_desc = "🔴 빨강 감지 -> 정지"
                stop_robot()
            elif green_count > threshold:
                action_desc = "🟢 초록 감지 -> 직진"
                move_robot(speed, 0.0)
                sleep_and_check_keys(move_time)
                stop_robot()
            elif blue_count > threshold:
                action_desc = "🔵 파랑 감지 -> 후진"
                move_robot(-speed, 0.0)
                sleep_and_check_keys(move_time)
                stop_robot()
            else:
                M = cv2.moments(white_mask)
                if M['m00'] > 0:
                    cx_cropped = int(M['m10'] / M['m00'])
                    cx = cx_cropped + w_start
                    err = cx - w / 2
                    
                    # 중심점 표기
                    abs_cx = cx
                    abs_cy = int(crop_start + M['m01']/M['m00'])
                    cv2.circle(frame, (abs_cx, abs_cy), 8, (0, 0, 255), -1)

                    if abs(err) > 40:
                        if err > 0:
                            action_desc = f"⚪ 우측 차선 (오차:{err:.1f}) -> 크게 우회전 스캔"
                            move_robot(0.0, -turn_speed)
                        else:
                            action_desc = f"⚪ 좌측 차선 (오차:{err:.1f}) -> 크게 좌회전 스캔"
                            move_robot(0.0, turn_speed)
                    else:
                        action_desc = f"⚪ 중앙 정렬 -> 직진 스텝"
                        move_robot(speed, 0.0)
                    
                    sleep_and_check_keys(move_time)
                    stop_robot()
                else:
                    action_desc = "⚪ 차선 미감지 -> 정지 (대기)"
                    stop_robot()

            tprint(f"[상태]: {action_desc} (직진:{speed:.2f}, 회전:{turn_speed:.2f})")

            cv2.imshow("Camera View", frame)
            cv2.imshow("White Mask (Line Only)", white_mask)
            cv2.waitKey(1)

    except Exception as e:
        tprint(f"오류 발생: {e}")
    finally:
        stop_robot()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        cv2.destroyAllWindows()
        print("\n프로그램이 안전하게 종료되었습니다.")

if __name__ == '__main__':
    main()
