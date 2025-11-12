#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion


class DWA(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.viz_pub = self.create_publisher(Marker, '/dwa_marker', 1)

        self.odom = None
        self.scan = None
        self.goal = None
        self.v = 0.0
        self.w = 0.0

        self.vmax = 0.22
        self.vmin = 0.0
        self.wmax = 2.0
        self.wmin = -2.0
        self.acc = 1.0
        self.acc_w = 4.0
        self.r = 0.14
        self.margin = 0.02

        self.dt = 0.1
        self.pred_t = 1.5
        self.dv = 10
        self.dw = 15

        self.wh = 0.7
        self.wv = 0.25
        self.wc = 0.05

        self.goal_tol = 0.12
        self.marker_skip = 3
        self.mctr = 0
        self.goal_done = False

        self.create_timer(self.dt, self.loop)
        self.get_logger().info('DWA node started')

    def odom_cb(self, msg):
        self.odom = msg
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

    def scan_cb(self, msg):
        self.scan = msg

    def goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'goal: {self.goal}')

    def scan_pts(self):
        if not self.scan:
            return []
        pts = []
        a = self.scan.angle_min
        for r in self.scan.ranges[::3]:
            if math.isfinite(r) and 0.02 < r < self.scan.range_max:
                pts.append((r * math.cos(a), r * math.sin(a)))
            a += self.scan.angle_increment * 3
        return pts

    def dyn_window(self):
        v1 = max(self.vmin, self.v - self.acc * self.dt)
        v2 = min(self.vmax, self.v + self.acc * self.dt)
        w1 = max(self.wmin, self.w - self.acc_w * self.dt)
        w2 = min(self.wmax, self.w + self.acc_w * self.dt)
        return v1, v2, w1, w2

    def sim(self, v, w):
        x = y = th = 0.0
        path = []
        t = 0.0
        while t < self.pred_t:
            x += v * math.cos(th) * self.dt
            y += v * math.sin(th) * self.dt
            th += w * self.dt
            path.append((x, y))
            t += self.dt
        return path, (x, y, th)

    def clearance(self, path, obs):
        if not obs:
            return 10.0
        mind = 999
        for px, py in path:
            for ox, oy in obs:
                d = math.hypot(px - ox, py - oy)
                if d < mind:
                    mind = d
        return mind

    def score(self, endp, v, clear, robot):
        rx, ry, rth = robot
        gx, gy = self.goal
        ex, ey, _ = endp
        wx = rx + math.cos(rth) * ex - math.sin(rth) * ey
        wy = ry + math.sin(rth) * ex + math.cos(rth) * ey
        dist = math.hypot(gx - wx, gy - wy)
        head = max(0.0, 1.0 - dist / 5.0)
        vel = v / self.vmax if self.vmax else 0
        c = min(clear, 2.0) / 2.0
        return self.wh * head + self.wv * vel + self.wc * c

    def loop(self):
        if not self.odom or not self.scan or not self.goal:
            return

        pose = self.odom.pose.pose
        q = pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        rx, ry = pose.position.x, pose.position.y
        gx, gy = self.goal
        d_goal = math.hypot(gx - rx, gy - ry)

        if d_goal < self.goal_tol:
            if not self.goal_done:
                self.get_logger().info(f'reached goal ({d_goal:.2f})')
                self.goal_done = True
            stop = Twist()
            self.cmd_pub.publish(stop)
            if self.mctr % self.marker_skip == 0:
                self.show_goal()
            self.mctr += 1
            return
        self.goal_done = False

        robot = (rx, ry, yaw)
        obs = self.scan_pts()
        v1, v2, w1, w2 = self.dyn_window()
        vs = [v1 + (v2 - v1) * i / (self.dv - 1) for i in range(self.dv)]
        ws = [w1 + (w2 - w1) * i / (self.dw - 1) for i in range(self.dw)]

        best, bv, bw, best_traj = -1, 0.0, 0.0, None
        for v in vs:
            for w in ws:
                path, endp = self.sim(v, w)
                c = self.clearance(path, obs)
                if c < self.r + self.margin:
                    continue
                s = self.score(endp, v, c, robot)
                if s > best:
                    best, bv, bw, best_traj = s, v, w, path

        cmd = Twist()
        if best_traj:
            cmd.linear.x = bv
            cmd.angular.z = bw
            if self.mctr % self.marker_skip == 0:
                self.show_traj(best_traj)
                self.show_goal()
            self.mctr += 1
        else:
            cmd.angular.z = 0.5
            self.get_logger().warn('no path, rotating')

        self.cmd_pub.publish(cmd)

    def show_traj(self, path):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'dwa_traj'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color.a = 1.0
        m.color.g = 1.0
        m.color.r = 0.0
        m.points = []
        for x, y in path:
            p = Point()
            p.x, p.y, p.z = x, y, 0.0
            m.points.append(p)
        m.lifetime.sec = 2
        self.viz_pub.publish(m)

    def show_goal(self):
        if not self.goal:
            return
        gx, gy = self.goal
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'dwa_goal'
        m.id = 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = gx
        m.pose.position.y = gy
        m.pose.position.z = 0.05
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color.a = 1.0
        m.color.r = 1.0
        self.viz_pub.publish(m)


def main():
    rclpy.init()
    node = DWA()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
