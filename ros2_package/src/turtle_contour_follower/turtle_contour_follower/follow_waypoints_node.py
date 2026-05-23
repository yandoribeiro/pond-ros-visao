import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class FollowWaypointsNode(Node):
    def __init__(self):
        super().__init__('follow_waypoints_node')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('linear_speed', 4.0)
        self.declare_parameter('angular_speed', 12.0)
        self.declare_parameter('goal_tolerance', 0.20)
        self.declare_parameter('angular_gain', 8.0)
        self.declare_parameter('heading_gate', 1.0)
        self.declare_parameter('min_point_spacing', 0.04)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.heading_gate = float(self.get_parameter('heading_gate').value)
        self.min_point_spacing = float(self.get_parameter('min_point_spacing').value)

        waypoints_file = str(self.get_parameter('waypoints_file').value).strip()
        if not waypoints_file:
            raise RuntimeError('Parametro waypoints_file nao informado.')

        self.contours = self._load_waypoints(Path(waypoints_file))
        self.contour_idx = 0
        self.point_idx = 0

        self.pose = None
        self._started = False
        self._transition = None
        self._pending_future = None

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self._pose_cb, 10)
        self.pen_cli = self.create_client(SetPen, '/turtle1/set_pen')
        self.tp_cli = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.timer = self.create_timer(0.05, self._tick)

        total_points = sum(len(c) for c in self.contours)
        self.get_logger().info(f'Contornos: {len(self.contours)} | pontos: {total_points}')

    def _load_waypoints(self, path: Path):
        if not path.exists():
            raise FileNotFoundError(f'Arquivo de waypoints nao encontrado: {path}')

        data = json.loads(path.read_text(encoding='utf-8'))
        contours = []

        for contour in data.get('contours', []):
            pts = []
            last = None
            for p in contour:
                cur = (float(p['x']), float(p['y']))
                if last is None or math.hypot(cur[0] - last[0], cur[1] - last[1]) >= self.min_point_spacing:
                    pts.append(cur)
                    last = cur
            if len(pts) >= 2:
                contours.append(pts)

        if not contours:
            raise RuntimeError('Nenhum contorno valido no arquivo de waypoints.')

        return contours

    def _pose_cb(self, msg: Pose):
        self.pose = msg

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _send_set_pen(self, off: bool):
        req = SetPen.Request()
        req.r = 255
        req.g = 255
        req.b = 255
        req.width = 2
        req.off = int(off)
        self._pending_future = self.pen_cli.call_async(req)

    def _send_teleport_to_current_start(self):
        contour = self.contours[self.contour_idx]
        x, y = contour[0]
        nx, ny = contour[1] if len(contour) > 1 else contour[0]
        theta = math.atan2(ny - y, nx - x)

        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        self._pending_future = self.tp_cli.call_async(req)

    def _start_transition(self):
        self._transition = 'pen_up'
        self._pending_future = None

    def _run_transition(self):
        if self._transition is None:
            return True

        if self._transition == 'pen_up':
            if self._pending_future is None:
                if not self.pen_cli.service_is_ready():
                    return False
                self._send_set_pen(off=True)
                return False
            if not self._pending_future.done():
                return False
            self._pending_future = None
            self._transition = 'teleport'
            return False

        if self._transition == 'teleport':
            if self._pending_future is None:
                if not self.tp_cli.service_is_ready():
                    return False
                self._send_teleport_to_current_start()
                return False
            if not self._pending_future.done():
                return False
            self._pending_future = None
            self._transition = 'pen_down'
            return False

        if self._transition == 'pen_down':
            if self._pending_future is None:
                if not self.pen_cli.service_is_ready():
                    return False
                self._send_set_pen(off=False)
                return False
            if not self._pending_future.done():
                return False
            self._pending_future = None
            self._transition = None
            return True

        return False

    def _advance_reached_points(self):
        # Consome varios pontos de uma vez para reduzir "stop-and-go".
        while self.contour_idx < len(self.contours):
            contour = self.contours[self.contour_idx]
            if self.point_idx >= len(contour):
                self.contour_idx += 1
                self.point_idx = 0
                return 'next_contour'

            tx, ty = contour[self.point_idx]
            dx = tx - self.pose.x
            dy = ty - self.pose.y
            dist = math.hypot(dx, dy)

            if dist >= self.goal_tolerance:
                return 'keep'

            self.point_idx += 1

        return 'done'

    def _tick(self):
        if self.pose is None:
            return

        if not self._started:
            self._start_transition()
            self._started = True

        if not self._run_transition():
            self._publish_stop()
            return

        status = self._advance_reached_points()
        if status == 'done':
            self._publish_stop()
            return
        if status == 'next_contour':
            if self.contour_idx >= len(self.contours):
                self._publish_stop()
                return
            self._start_transition()
            self._publish_stop()
            return

        contour = self.contours[self.contour_idx]
        tx, ty = contour[self.point_idx]

        dx = tx - self.pose.x
        dy = ty - self.pose.y
        dist = math.hypot(dx, dy)

        desired_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_heading - self.pose.theta)

        cmd = Twist()
        cmd.angular.z = clamp(self.angular_gain * heading_error, -self.angular_speed, self.angular_speed)

        if abs(heading_error) < self.heading_gate:
            cmd.linear.x = min(self.linear_speed, dist * 1.5)
        else:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FollowWaypointsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
