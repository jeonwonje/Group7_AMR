import json

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


class ScoreAndPostNode(Node):
    def __init__(self):
        super().__init__('score_and_post')
        self.get_logger().info('score_and_post node has started')
        self.bfs_data = {}
        self.frontiers = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_resolution = 1.0
        self.navigation_in_progress = False
        self.frontiers_subscription()
        self.bfs_subscription()
        self.map_data_subscription()
        self.nav2_server_subscription()
        self.nav2_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def frontiers_subscription(self):
        self.frontiers_subscriber = self.create_subscription(
            String,
            'frontiers',
            self.frontiers_callback,
            10,
        )

    def bfs_subscription(self):
        self.bfs_subscriber = self.create_subscription(
            String,
            'bfs_distance_transform',
            self.bfs_callback,
            10,
        )

    def map_data_subscription(self):
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10,
        )

    def nav2_server_subscription(self):
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_client.wait_for_server()
        self.get_logger().info('Connected to navigation action server')
        self.navigation_in_progress = False

    def map_callback(self, msg):
        self.robot_x = msg.info.origin.position.x
        self.robot_y = msg.info.origin.position.y
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_resolution = msg.info.resolution

    def bfs_callback(self, msg):
        self.get_logger().info('Received BFS distance transform data')
        try:
            self.bfs_data = json.loads(msg.data)
        except json.JSONDecodeError as error:
            self.get_logger().error(f'Failed to decode BFS data: {error}')

    def frontiers_callback(self, msg):
        self.get_logger().info('Received frontiers data')
        try:
            self.frontiers = json.loads(msg.data)
            self.filter_frontiers()
        except json.JSONDecodeError as error:
            self.get_logger().error(f'Failed to decode frontiers data: {error}')

    def calculate_distance_score(self, frontier, robot_position):
        frontier_x, frontier_y = frontier
        robot_x, robot_y = robot_position
        distance = ((frontier_x - robot_x) ** 2 + (frontier_y - robot_y) ** 2) ** 0.5
        return distance

    def filter_frontiers(self):
        if self.navigation_in_progress:
            self.get_logger().debug(
                'Navigation in progress - skipping frontier scoring'
            )
            return
        if not hasattr(self, 'frontiers') or not isinstance(self.frontiers, list):
            self.get_logger().warning(
                'Frontiers data not yet available or invalid; skipping scoring.'
            )
            return

        if not hasattr(self, 'bfs_data') or not isinstance(self.bfs_data, dict):
            self.get_logger().warning(
                'BFS data not yet available; skipping scoring.'
            )
            return

        if not self.frontiers:
            self.get_logger().info('Frontiers data is empty; no frontiers found!')
            return

        if not self.bfs_data:
            self.get_logger().info(
                'BFS data is empty; no distance transform available.'
            )
            return

        self.scored_frontiers = {}
        for frontier in self.frontiers:
            frontier_key = str(tuple(frontier))
            if frontier_key in self.bfs_data:
                bfs_score = self.bfs_data[frontier_key]
                if bfs_score == float('inf'):
                    self.get_logger().debug(
                        f'Frontier {frontier_key} has infinite BFS score; skipping.'
                    )
                    continue
                distance_score = self.calculate_distance_score(
                    frontier,
                    (self.robot_x, self.robot_y),
                )
                self.scored_frontiers[frontier_key] = (bfs_score, -distance_score)
            else:
                self.get_logger().warning(
                    f'Frontier {frontier_key} not found in BFS data; skipping.'
                )

        self.choose_best_frontier()

    def choose_best_frontier(self):
        if not hasattr(self, 'scored_frontiers') or not isinstance(
            self.scored_frontiers,
            dict,
        ):
            self.get_logger().warning(
                'Scored frontiers data not yet available or invalid.'
            )
            return

        if not self.scored_frontiers:
            self.get_logger().info(
                'No scored frontiers available; cannot select best frontier.'
            )
            return

        if self.navigation_in_progress:
            self.get_logger().debug('Navigation in progress - skipping goal posting')
            return

        best_frontier = max(self.scored_frontiers, key=self.scored_frontiers.get)
        self.get_logger().info(
            f'Best frontier selected: {best_frontier} '
            f'with score {self.scored_frontiers[best_frontier]}'
        )
        self.nav2_goal_publisher(best_frontier)

    def nav2_goal_publisher(self, best_frontier):
        if self.navigation_in_progress:
            self.get_logger().debug(
                'Navigation in progress - skipping goal publishing'
            )
            return
        try:
            frontier_coords = eval(best_frontier)
            frontier_x, frontier_y = frontier_coords

            world_x = self.map_origin_x + frontier_x * self.map_resolution
            world_y = self.map_origin_y + frontier_y * self.map_resolution

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = world_x
            goal.pose.position.y = world_y
            goal.pose.position.z = 0.0
            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = 0.0
            goal.pose.orientation.w = 1.0

            self.navigation_in_progress = True
            future = self.nav_client.send_goal_async(
                NavigateToPose.Goal(pose=goal)
            )
            future.add_done_callback(self.goal_response_callback)
            self.get_logger().info(
                'Published navigation goal to frontier at world coordinates: '
                f'({world_x:.2f}, {world_y:.2f})'
            )
        except Exception as error:
            self.get_logger().error(f'Failed to publish navigation goal: {error}')

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Navigation goal rejected')
                self.navigation_in_progress = False
                return

            self.get_logger().info('Navigation goal accepted')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.nav_result_callback)
        except Exception as error:
            self.get_logger().error(
                f'Error in goal response callback: {error}'
            )
            self.navigation_in_progress = False

    def nav_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f'Navigation result received: {result}')
        except Exception as error:
            self.get_logger().error(
                f'Error in navigation result callback: {error}'
            )
        finally:
            self.navigation_in_progress = False


def main(args=None):
    rclpy.init(args=args)
    node = ScoreAndPostNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutdown requested (KeyboardInterrupt).')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
