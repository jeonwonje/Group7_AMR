import json
from collections import deque

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import String


class AutoExploreNode(Node):
    def __init__(self):
        super().__init__('auto_explore')
        self.get_logger().info('auto_explore node has started')
        self.frontiers_pub = self.create_publisher(String, 'frontiers', 10)
        self.bfs_pub = self.create_publisher(
            String,
            'bfs_distance_transform',
            10,
        )
        self.navigation_in_progress = False
        self.map_data_subscription()
        self.nav_status_subscription()

    def map_data_subscription(self):
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10,
        )

    def nav_status_subscription(self):
        self.nav_status_subscriber = self.create_subscription(
            String,
            '/nav_status',
            self.nav_status_callback,
            10,
        )

    def nav_status_callback(self, msg):
        self.navigation_in_progress = msg.data == 'navigating'

    def map_callback(self, msg):
        self.get_logger().info('Received map data')
        self.map_formatted_data = {}
        self.occupancy_grid = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = msg.info.origin
        for i in range(self.map_height):
            for j in range(self.map_width):
                index = i * self.map_width + j
                self.map_formatted_data[(j, i)] = self.occupancy_grid[index]
        self.find_frontiers()
        self.bfs_distance_transform()

    def find_frontiers(self):
        if self.navigation_in_progress:
            self.get_logger().debug(
                'Navigation in progress - skipping frontier search'
            )
            return

        if not hasattr(self, 'map_formatted_data'):
            self.get_logger().warning(
                'Map data not yet available; skipping frontier search.'
            )
            return

        if not self.map_formatted_data:
            self.get_logger().info('Map data is empty; no frontiers to find.')
            return

        self.frontiers = []
        for (x, y), value in self.map_formatted_data.items():
            if 0 <= value <= 50:
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if (dx, dy) == (0, 0):
                            continue
                        neighbor = (x + dx, y + dy)
                        if (
                            neighbor in self.map_formatted_data
                            and self.map_formatted_data[neighbor] == -1
                        ):
                            self.frontiers.append((x, y))
                            break
                    else:
                        continue
                    break

        self.get_logger().info(
            f'Found {len(self.frontiers)} frontiers in the map'
        )

    def bfs_distance_transform(self):
        if self.navigation_in_progress:
            self.get_logger().debug(
                'Navigation in progress - skipping BFS distance transform'
            )
            return

        self.distance_transform = {}
        queue = deque()
        for (x, y), value in self.map_formatted_data.items():
            if value >= 50:
                self.distance_transform[(x, y)] = 0
                queue.append((x, y))
            else:
                self.distance_transform[(x, y)] = float('inf')

        while queue:
            x, y = queue.popleft()
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                neighbor = (x + dx, y + dy)
                next_distance = self.distance_transform[(x, y)] + 1
                if (
                    neighbor in self.distance_transform
                    and self.distance_transform[neighbor] > next_distance
                ):
                    self.distance_transform[neighbor] = next_distance
                    queue.append(neighbor)

        self.publish_frontiers()
        self.publish_bfs()

    def publish_frontiers(self):
        frontiers_serializable = [list(frontier) for frontier in self.frontiers]
        self.frontiers_pub.publish(
            String(data=json.dumps(frontiers_serializable))
        )

    def publish_bfs(self):
        distance_serializable = {
            str(key): value for key, value in self.distance_transform.items()
        }
        self.bfs_pub.publish(String(data=json.dumps(distance_serializable)))
        self.get_logger().info(
            'Published frontiers and BFS distance transform data'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AutoExploreNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutdown requested (KeyboardInterrupt).')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
