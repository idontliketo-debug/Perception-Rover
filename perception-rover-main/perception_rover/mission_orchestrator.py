import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json
import math
import subprocess


WASTE_POSITIONS = {
    'waste_0': (1.5, 3.5),
    'waste_1': (-4.0, 0.5),
    'waste_2': (4.5, -2.0),
    'waste_3': (-1.0, -4.5),
    'waste_4': (2.5, -1.0),
    'waste_5': (-2.5, 3.0),
    'waste_6': (4.0, 3.5),
    'waste_7': (-4.5, -4.0),
}

APPROACH_DISTANCE = 0.8
COLLECT_PROXIMITY = 1.0
COLLECT_DURATION = 4.0
WAYPOINT_TOLERANCE = 0.5
NAV_GOAL_TIMEOUT = 30.0


class MissionOrchestrator(Node):
    STATE_SWEEP = 'SWEEP'
    STATE_INTERCEPT = 'INTERCEPT'
    STATE_COLLECT = 'COLLECT'
    STATE_RESUME = 'RESUME'

    def __init__(self):
        super().__init__('mission_orchestrator')

        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self._target_sub = self.create_subscription(
            String, '/target', self._target_callback, 10)
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)

        self._state = self.STATE_SWEEP
        self._navigating = False
        self._goal_id = 0
        self._current_goal_handle = None
        self._nav_goal_sent_time = None

        self._waypoints = self._generate_lawnmower()
        self._waypoint_index = 0
        self._last_waypoint_index = 0

        self._waste_model = None
        self._waste_world_pos = None
        self._collect_start = None

        self._remaining_waste = set(WASTE_POSITIONS.keys())

        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0

        self._timer = self.create_timer(0.5, self._control_loop)
        self.get_logger().info(
            f'Mission Orchestrator started — '
            f'{len(self._waypoints)} waypoints, '
            f'{len(self._remaining_waste)} waste items, '
            f'approach={APPROACH_DISTANCE}m, '
            f'collect_proximity={COLLECT_PROXIMITY}m')

    def _odom_callback(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _target_callback(self, msg: String):
        if self._state != self.STATE_SWEEP:
            return
        if not self._remaining_waste:
            return

        closest_model = self._find_nearest_waste()
        if closest_model is None:
            return

        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()

        self._waste_model = closest_model
        self._waste_world_pos = WASTE_POSITIONS[closest_model]
        self._last_waypoint_index = self._waypoint_index
        self._navigating = False
        self._current_goal_handle = None
        self._nav_goal_sent_time = None

        self.get_logger().info(
            f'Green detected → intercepting {closest_model} '
            f'at {self._waste_world_pos}')
        self._state = self.STATE_INTERCEPT

    def _find_nearest_waste(self):
        best = None
        best_dist = float('inf')
        for model_name in self._remaining_waste:
            mx, my = WASTE_POSITIONS[model_name]
            d = math.hypot(self._robot_x - mx, self._robot_y - my)
            if d < best_dist:
                best_dist = d
                best = model_name
        return best

    def _generate_lawnmower(self):
        x_min, x_max = -4.5, 4.5
        y_min, y_max = -4.5, 4.5
        row_spacing = 2.0
        wp_spacing = 1.5

        waypoints = []
        y = y_min
        row_idx = 0
        while y <= y_max + 0.01:
            if row_idx % 2 == 0:
                x = x_min
                while x <= x_max + 0.01:
                    waypoints.append((round(x, 2), round(y, 2), 0.0))
                    x += wp_spacing
            else:
                x = x_max
                while x >= x_min - 0.01:
                    waypoints.append((round(x, 2), round(y, 2), 0.0))
                    x -= wp_spacing
            y += row_spacing
            row_idx += 1
        return waypoints

    def _compute_approach_pose(self, waste_x, waste_y):
        dx = waste_x - self._robot_x
        dy = waste_y - self._robot_y
        dist = math.hypot(dx, dy)
        if dist < 0.01:
            return (self._robot_x + APPROACH_DISTANCE, self._robot_y, 0.0)
        ux = dx / dist
        uy = dy / dist
        ax = waste_x - ux * APPROACH_DISTANCE
        ay = waste_y - uy * APPROACH_DISTANCE
        yaw = math.atan2(waste_y - ay, waste_x - ax)
        return (ax, ay, yaw)

    def _make_goal(self, x, y, yaw=0.0):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        half = yaw / 2.0
        goal.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        return nav_goal

    def _send_nav_goal(self, x, y, yaw=0.0):
        if self._navigating:
            return
        self._action_client.wait_for_server()
        self._navigating = True
        self._goal_id += 1
        self._nav_goal_sent_time = self.get_clock().now()
        nav_goal = self._make_goal(x, y, yaw)
        future = self._action_client.send_goal_async(nav_goal)
        future.add_done_callback(self._nav_response_callback)

    def _nav_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self._navigating = False
            self._current_goal_handle = None
            return
        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        self._navigating = False
        self._current_goal_handle = None
        self.get_logger().info('Nav goal completed')

    def _delete_waste(self, model_name):
        req = f'entity: {{name: "{model_name}"}}, position: {{z: -10}}'
        cmd = [
            'gz', 'service', '-s', '/world/perception_world/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3',
            '--req', req,
        ]
        try:
            result = subprocess.run(cmd, capture_output=True, timeout=5)
            if result.returncode == 0:
                self.get_logger().info(f'Deleted {model_name}')
            else:
                self.get_logger().error(
                    f'Failed to delete {model_name}: {result.stderr.decode()}')
        except subprocess.TimeoutExpired:
            self.get_logger().error(f'Timeout deleting {model_name}')
        except Exception as e:
            self.get_logger().error(f'Exception deleting {model_name}: {e}')

    def _control_loop(self):
        if self._state == self.STATE_SWEEP:
            if self._waypoint_index >= len(self._waypoints):
                self.get_logger().info('All waypoints visited — looping')
                self._waypoint_index = 0
                return

            wx, wy, _ = self._waypoints[self._waypoint_index]
            dist = math.hypot(
                self._robot_x - wx, self._robot_y - wy)

            if dist < WAYPOINT_TOLERANCE:
                self._waypoint_index += 1
                self._navigating = False
                self._current_goal_handle = None
                return

            if self._nav_goal_sent_time is not None:
                elapsed = (
                    self.get_clock().now() - self._nav_goal_sent_time
                ).nanoseconds / 1e9
                if elapsed > NAV_GOAL_TIMEOUT:
                    self.get_logger().warn(
                        f'Nav goal timeout ({NAV_GOAL_TIMEOUT}s), advancing')
                    self._waypoint_index += 1
                    self._navigating = False
                    self._current_goal_handle = None
                    self._nav_goal_sent_time = None
                    return

            if not self._navigating:
                self.get_logger().info(
                    f'SWEEP → wp {self._waypoint_index}/{len(self._waypoints)} '
                    f'({wx}, {wy}) dist={dist:.1f}m')
                self._send_nav_goal(wx, wy)

        elif self._state == self.STATE_INTERCEPT:
            if self._waste_world_pos is None:
                self._state = self.STATE_RESUME
                return

            waste_dist = math.hypot(
                self._robot_x - self._waste_world_pos[0],
                self._robot_y - self._waste_world_pos[1])

            if waste_dist < COLLECT_PROXIMITY:
                self.get_logger().info(
                    f'Within {COLLECT_PROXIMITY}m of {self._waste_model} '
                    f'(dist={waste_dist:.2f}m) → COLLECT')
                self._state = self.STATE_COLLECT
                self._collect_start = self.get_clock().now()
                if self._navigating:
                    self._navigating = False
                    self._current_goal_handle = None
                return

            if self._nav_goal_sent_time is not None:
                elapsed = (
                    self.get_clock().now() - self._nav_goal_sent_time
                ).nanoseconds / 1e9
                if elapsed > NAV_GOAL_TIMEOUT:
                    self.get_logger().warn(
                        f'Intercept timeout, giving up on {self._waste_model}')
                    self._waste_model = None
                    self._waste_world_pos = None
                    self._state = self.STATE_RESUME
                    return

            if not self._navigating:
                ax, ay, ayaw = self._compute_approach_pose(
                    self._waste_world_pos[0], self._waste_world_pos[1])
                self.get_logger().info(
                    f'Approach ({ax:.2f}, {ay:.2f}) for '
                    f'{self._waste_model} (dist={waste_dist:.1f}m)')
                self._send_nav_goal(ax, ay, ayaw)

        elif self._state == self.STATE_COLLECT:
            elapsed = (
                self.get_clock().now() - self._collect_start
            ).nanoseconds / 1e9
            if elapsed >= COLLECT_DURATION:
                if self._waste_model:
                    self._delete_waste(self._waste_model)
                    self._remaining_waste.discard(self._waste_model)
                    self.get_logger().info(
                        f'Remaining waste: {len(self._remaining_waste)}')
                    self._waste_model = None
                    self._waste_world_pos = None
                self._state = self.STATE_RESUME
                self.get_logger().info(
                    f'Collection complete → RESUME at wp '
                    f'{self._last_waypoint_index}')

        elif self._state == self.STATE_RESUME:
            self._waypoint_index = self._last_waypoint_index
            self._navigating = False
            self._current_goal_handle = None
            self._nav_goal_sent_time = None
            self._state = self.STATE_SWEEP


def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
