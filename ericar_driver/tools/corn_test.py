#!/usr/bin/env python3
"""라바콘 단독 실차 시험: 차선·표지판·구간 없이 라이다 라바콘 코드(cones.py)만 실행

차를 첫 라바콘 쌍 앞 0.5~1.5 m에 통로를 보게 두고 실행.
문(좌·우 콘 쌍)이 보일 때까지 직진 → 라바콘 모드 → 콘이 끝나면 --after(m) 더 직진 후 정지·종료.
구간 관리자를 거치지 않으므로 params.py의 sections.off와 무관 (수정 불필요).

==================== 터미널에 칠 명령 (순서대로) ====================
노트북과 차를 같은 휴대폰 핫스팟에 연결한 뒤 시작. <차 IP>는 차 화면이나 핫스팟 연결 목록에서 확인.

[0] 노트북 → 차 접속 (터미널 창 2개에서 각각. 비밀번호는 직접 입력)
    ssh wego@<차 IP>

[1] 차, 터미널 A: 코드 받기 + 빌드 (코드가 바뀌었을 때만)
    cd ~/ericar_dev_ws/src/ericar && git fetch && git switch cone_fixed && git pull
    cd ~/ericar_dev_ws && colcon build --packages-select ericar_driver

[2] 차, 터미널 A·B 둘 다: ROS 환경 불러오기 (ssh 접속할 때마다 한 번)
    source /opt/ros/humble/setup.bash && source ~/wego_ws/install/setup.bash && source ~/ericar_dev_ws/install/setup.bash

[3] 차, 터미널 A: 배터리 확인 (battery_voltage 11 V 미만이면 충전 후 시험)
    ros2 topic echo /limo_status --once | grep battery_voltage

[4] 차, 터미널 A: 바퀴·라이다·카메라 + 녹화 켜기 (주행 노드 없이. 켜 둔 채로 둠)
    ros2 launch ericar_driver bringup.launch.py driver:=false record:=true bag:=$HOME/bags/cone_1
    - 반드시 driver:=false. 주행 노드가 같이 켜지면 명령이 겹쳐서 이 시험은 스스로 종료함
    - bag 이름(cone_1)은 주행마다 새 이름으로. 이미 있는 이름이면 녹화가 시작되지 않음
    - 녹화가 필요 없으면 record:=true bag:=... 부분을 뺌

[5] 차, 터미널 B: 인식만 확인 (명령을 보내지 않음. 차를 손으로 밀어 콘 쪽으로 가져가 봄)
    python3 ~/ericar_dev_ws/src/ericar/ericar_driver/tools/corn_test.py --dry
    - 화면에 "콘 N 문 N"이 실제 배치와 맞게 나오고 상태가 approach → cones로 바뀌면 정상
    - Ctrl+C로 끝냄

[6] 차, 터미널 B: 실제 주행 (차를 첫 콘 쌍 앞 0.5~1.5 m에 통로를 보게 두고, 조종기 SWB를 위(자동)로)
    python3 ~/ericar_dev_ws/src/ericar/ericar_driver/tools/corn_test.py
    - 끝나면 스스로 정지·종료하고 "종료: <이유>. 주행 ... 기록 <CSV 경로>"를 출력
    - 급할 때: 조종기 SWB를 아래로(즉시 정지) 또는 터미널 B에서 Ctrl+C
    - 다시 달릴 때는 차를 출발 위치로 옮기고 [6]만 반복 (녹화를 나누려면 [4]를 Ctrl+C 후 새 bag 이름으로 다시)

[7] 끝낼 때: 터미널 B가 끝난 뒤 터미널 A에서 Ctrl+C (녹화 저장)

[8] 기록 확인 (차) / 노트북으로 복사 (노트북에서 실행. CSV는 작아서 복사해도 됨, bag은 복사 금지)
    ls -t ~/bags/corn_test_*.csv | head -3
    scp "wego@<차 IP>:~/bags/corn_test_*.csv" .
======================================================================

옵션
  --dry            명령을 보내지 않고 판단만 출력·기록 (차를 손으로 밀거나 조종기로 몰며 인식 확인)
  --approach 0.15  문을 찾는 동안·통과 뒤 직진 속도 (m/s)
  --search 2.0     이 거리(m)를 직진해도 첫 문을 못 찾으면 종료
  --after 1.0      라바콘 모드가 꺼진 뒤 이 거리(m)를 더 가도 다음 문이 없으면 종료. 쌍 간격이 2 m보다 넓으면 늘릴 것
  --limit 6.0      총 주행 거리 제한 (m)
  --timeout 90     총 시간 제한 (s)
  --log 경로        기록 CSV (기본 ~/bags/corn_test_<월일_시분초>.csv)
정지: 정면 emergency_distance 안 물체(풀리면 다시 감), 라이다·오도메트리 끊김(풀리면 다시 감)
기록 CSV: 제어 주기(20 Hz)마다 1줄. 작은 파일이라 와이파이로 복사 가능 (bag은 차에서 분석)
  t, 상태, 속도 명령, 조향 명령, 실제 속도, 회전 속도, 정면 거리, 콘 수, 문 수, 목표 x, 목표 y, 진행 방향, 문 폭,
  안전 여부, 라바콘 모드 주행 거리, 콘 위치들(차체 좌표 "x:y x:y ...", 뒷바퀴 축 기준 m)
"""
import argparse
import math
import os
import signal
import sys
import time
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from ericar_driver.common import clip  # noqa: E402
from ericar_driver.cones import ConeDriving, find_posts, red_ratio  # noqa: E402
from ericar_driver.lidar import scan_sectors  # noqa: E402
from ericar_driver.loop import load_config  # noqa: E402

COLUMNS = ('t,state,speed,steering,velocity,yaw_rate,front,posts,gates,target_x,target_y,heading,width,'
           'safe,cone_travel,post_xy')


class ConeTest:
    """ROS 없는 계산부: 센서 값 → (속도, 조향, 상태). 맥·윈도우에서 합성 라이다로 시험 가능"""
    def __init__(self, config, approach=0.15, search=2.0, after=1.0, limit=6.0, timeout=90.0):
        self.c, self.robot = config['answer'], config['robot']
        self.cones = ConeDriving(self.c, self.robot)
        self.approach, self.search, self.after = approach, search, after
        self.limit, self.timeout = limit, timeout
        self.started = self.last = None
        self.travel = 0.0      # 총 주행 거리 (m)
        self.since = None      # 라바콘 모드가 꺼진 뒤 주행 거리 (m). 한 번도 안 켜졌으면 None
        self.entries = 0       # 라바콘 모드가 켜진 횟수
        self.finished = None   # 종료 이유. None이면 진행 중

    def step(self, now, scan, velocity, yaw_rate, red=0.0):
        """red: 카메라 붉은 화소 비율 (cones.red_ratio). 주행 노드와 같이 lane['cone_red']로 넘김"""
        if self.started is None:
            self.started = now
        dt = 0.05 if self.last is None else clip(now-self.last, 0.001, 0.2)
        self.last = now
        self.travel += abs(velocity)*dt
        if self.finished:
            return 0.0, 0.0, 'finished'
        was_active = self.cones.active
        command = self.cones.update({'cone_red': red}, scan, velocity, yaw_rate, dt, True)
        self.entries += self.cones.active and not was_active
        if command is not None:
            (speed, steering), state, self.since = command, 'cones', 0.0
        else:
            # 문이 아직 안 보이거나(approach) 콘 사이를 지난 뒤(after): 직진하며 다음 문을 기다림
            speed, steering = self.approach, 0.0
            state = 'approach' if self.since is None else 'after'
            if self.since is not None:
                self.since += abs(velocity)*dt
            if self.cones.done:
                self.finished = '라바콘 구간 거리 초과 (cone_max_distance)'
            elif self.since is None and self.travel >= self.search:
                self.finished = '%.1f m 직진해도 문을 못 찾음' % self.search
            elif self.since is not None and self.since >= self.after:
                self.finished = '라바콘 통과 뒤 %.1f m 동안 다음 문 없음' % self.after
        if self.travel >= self.limit:
            self.finished = '총 주행 거리 제한 %.1f m' % self.limit
        elif now-self.started >= self.timeout:
            self.finished = '시간 제한 %.0f s' % self.timeout
        if self.finished:
            return 0.0, 0.0, 'finished'
        # 주행 노드(driver.py)와 같은 최종 정지 조건: 라이다 유효 점 절반 미만 또는 정면 가까이 물체
        if scan['valid_fraction'] < .5 or scan['front'] < self.c['lidar']['emergency_distance']:
            return 0.0, steering, 'blocked'
        return speed, steering, state

    def row(self, now, scan, velocity, yaw_rate, speed, steering, state):
        """기록 CSV 한 줄"""
        d = self.cones.debug
        posts, _ = find_posts(scan, self.robot, self.c['lidar'])
        number = lambda key: '' if d.get(key) is None else '%.3f' % d[key]  # noqa: E731
        return ','.join([
            '%.2f' % (now-self.started), state, '%.3f' % speed, '%.3f' % steering, '%.3f' % velocity,
            '%.3f' % yaw_rate, '%.2f' % min(scan['front'], 9.99), str(len(posts)), str(d.get('gates', '')),
            number('target_x'), number('target_y'), '%.3f' % self.cones.heading, '%.2f' % self.cones.width,
            '' if 'safe' not in d else str(int(d['safe'])), '%.2f' % self.cones.travel,
            ' '.join('%.2f:%.2f' % (x, y) for x, y in posts)])


def main():
    parser = argparse.ArgumentParser(description='라바콘 단독 실차 시험')
    parser.add_argument('--dry', action='store_true', help='명령을 보내지 않고 판단만 출력')
    parser.add_argument('--approach', type=float, default=0.15, help='문을 찾는 동안 직진 속도 (m/s)')
    parser.add_argument('--search', type=float, default=2.0, help='첫 문을 찾으며 직진하는 최대 거리 (m)')
    parser.add_argument('--after', type=float, default=1.0, help='라바콘 모드가 꺼진 뒤 더 가 보는 거리 (m)')
    parser.add_argument('--limit', type=float, default=6.0, help='총 주행 거리 제한 (m)')
    parser.add_argument('--timeout', type=float, default=90.0, help='총 시간 제한 (s)')
    parser.add_argument('--log', default=time.strftime('~/bags/corn_test_%m%d_%H%M%S.csv'), help='기록 CSV 경로')
    parser.add_argument('--config', default=str(PACKAGE / 'config' / 'robot.yaml'), help='robot.yaml 경로')
    options = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from ackermann_msgs.msg import AckermannDrive
    from nav_msgs.msg import Odometry
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, Imu, LaserScan
    from std_msgs.msg import String

    config = load_config(options.config)
    c, robot, topics = config['answer'], config['robot'], config['topics']

    class ConeTestNode(Node):
        def __init__(self):
            super().__init__('corn_test')
            self.test = ConeTest(config, options.approach, options.search, options.after,
                                 options.limit, options.timeout)
            self.exit = False
            self.scan = None
            self.scan_seen = self.odom_seen = self.imu_seen = self.image_seen = -1e9
            self.velocity = self.angular_velocity = self.yaw_rate = self.red = 0.0
            self.bridge = CvBridge()
            self.state, self.printed, self.stop_until = '', -1e9, None
            path = Path(options.log).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.log = open(path, 'w', buffering=1, encoding='utf-8')   # 줄마다 바로 기록 (강제 종료돼도 남음)
            self.log.write(COLUMNS+'\n')
            self.publisher = self.create_publisher(AckermannDrive, topics['drive'], 10)
            self.status = self.create_publisher(String, topics.get('status', '/answer/status'), 10)
            self.create_subscription(LaserScan, topics['scan'], self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, topics['odometry'], self.on_odometry, qos_profile_sensor_data)
            self.create_subscription(Imu, topics.get('imu', '/imu'), self.on_imu, qos_profile_sensor_data)
            self.create_subscription(Image, topics['image'], self.on_image, qos_profile_sensor_data)
            self.create_timer(1.0/c['rate_hz'], self.control)
            lidar = c['lidar']
            self.get_logger().info(
                '라바콘 단독 시험 시작%s. 문 간격 %.2f~%.2f m, 진입 거리 %.1f m, 속도 %.2f m/s, 기록 %s' % (
                    ' (--dry: 명령 안 보냄)' if options.dry else '', lidar['cone_gate_min'], lidar['cone_gate_max'],
                    lidar['cone_start_range'], lidar['avoidance_speed'], path))

        def now(self):
            return self.get_clock().now().nanoseconds/1e9

        # 센서 나이는 수신 시각 기준 (헤더 시각이 비어 있는 드라이버가 있음)
        def on_scan(self, message):
            self.scan = scan_sectors(message.ranges, message.angle_min, message.angle_increment,
                                     message.range_min, message.range_max, c['lidar'])
            self.scan_seen = self.now()

        def on_odometry(self, message):
            self.velocity = message.twist.twist.linear.x
            self.angular_velocity = message.twist.twist.angular.z
            self.odom_seen = self.now()

        def on_imu(self, message):
            self.yaw_rate = message.angular_velocity.z
            self.imu_seen = self.now()

        def on_image(self, message):
            # 붉은색 비율만 사용 (차선 인식 없음). 실패해도 라이다만으로 계속
            try:
                self.red = red_ratio(self.bridge.imgmsg_to_cv2(message, 'bgr8'), c['lidar'])
                self.image_seen = self.now()
            except Exception as error:
                self.get_logger().error(f'image conversion failed: {error}')

        def publish(self, speed, steering):
            """node.py와 같은 최종 한계·조향 영점 보정. --dry면 보내지 않음"""
            if options.dry:
                return
            message = AckermannDrive()
            message.speed = float(clip(speed, -robot['max_speed'], robot['max_speed']))
            message.steering_angle = float(clip(steering+robot.get('steering_offset', 0.0),
                                                -robot['max_steering'], robot['max_steering']))
            self.publisher.publish(message)

        def control(self):
            now = self.now()
            if not options.dry and getattr(self, 'count_publishers', lambda topic: 1)(topics['drive']) > 1:
                self.test.finished = '%s에 다른 발행 노드가 있음 (주행 노드?). bringup을 driver:=false로 다시 켤 것' % topics['drive']
                self.get_logger().error(self.test.finished)
                self.exit = True
                return
            fresh = (self.scan is not None and now-self.scan_seen < c['sensor_timeout'] and
                     now-self.odom_seen < c['odometry_timeout'] and math.isfinite(self.velocity))
            if not fresh:
                speed, steering, state = 0.0, 0.0, 'waiting for scan / odom'
            else:
                # 회전 속도: IMU가 최신이면 자이로, 아니면 odom × 보정 (loop.py·driver.py와 같음)
                imu_ok = now-self.imu_seen < c['sections']['imu_timeout'] and math.isfinite(self.yaw_rate)
                yaw_rate = self.yaw_rate if imu_ok else self.angular_velocity*c['sections']['odom_yaw_scale']
                # 영상이 끊기면(0.5 s) 붉은색 없음으로 봄 → 라이다만으로 판단 (기존 동작)
                red = self.red if now-self.image_seen < 0.5 else 0.0
                speed, steering, state = self.test.step(now, self.scan, self.velocity, yaw_rate, red)
                self.log.write(self.test.row(now, self.scan, self.velocity, yaw_rate, speed, steering, state)+'\n')
            self.publish(speed, steering)
            d = self.test.cones.debug
            if state != self.state:
                self.state = state
                text = state if state != 'finished' else 'finished: %s' % self.test.finished
                self.get_logger().info('상태 → %s (주행 %.2f m, 진입 %d회)' % (text, self.test.travel, self.test.entries))
                self.status.publish(String(data='corn_test: '+text))
            elif now-self.printed >= 0.5 and fresh:
                self.printed = now
                self.get_logger().info('%s 속도 %.2f 조향 %+.1f° 콘 %s 문 %s 정면 %.2f m 진행 방향 %+.1f° 붉은색 %.4f%s' % (
                    state, speed, math.degrees(steering), d.get('posts', '-'), d.get('gates', '-'),
                    min(self.scan['front'], 9.99), math.degrees(self.test.cones.heading),
                    self.red, ' (콘 색)' if self.test.cones.red else ''))
            if state == 'finished':
                # 정지 명령을 1초 더 보낸 뒤 종료
                self.stop_until = self.stop_until or now+1.0
                self.exit = self.exit or now >= self.stop_until

    def terminate(*_):   # ssh 백그라운드 실행은 SIGINT를 무시 → SIGTERM도 같은 정지 절차로
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = ConeTestNode()
    try:
        while rclpy.ok() and not node.exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(5):   # 정지 명령이 빠지지 않게 여러 번
                node.publish(0.0, 0.0)
                time.sleep(0.02)
        node.log.close()
        test = node.test
        print('종료: %s. 주행 %.2f m, 라바콘 모드 진입 %d회, 기록 %s' % (
            test.finished or '사용자 중단', test.travel, test.entries, os.path.expanduser(options.log)))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
