#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import random
import time

class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner_node')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Gazebo services...')

        #configuration
        self.x_range = [-1.5, 1.5] 
        self.LANE_HEIGHTS = [1.0, 0.5, 0.0, -0.5, -1.0, -1.5]
        self.obj_names = ["my_cylinder", "my_box"]

        #clear previous objects to avoid duplicates
        self.clear_garden()
        time.sleep(1.0)

        #select 2 different lanes randomly
        selected_lanes = random.sample(self.LANE_HEIGHTS, 2)

        self.spawn_object("cylinder", self.obj_names[0], selected_lanes[0])
        self.spawn_object("box", self.obj_names[1], selected_lanes[1])

    def clear_garden(self):
        for name in self.obj_names:
            req = DeleteEntity.Request()
            req.name = name
            self.get_logger().info(f'Attempting to delete {name}...')
            self.delete_client.call_async(req)

    def spawn_object(self, shape, name, lane_y):
        request = SpawnEntity.Request()
        request.name = name
        
        if shape == "cylinder":
            xml = f"""
            <sdf version='1.6'>
              <model name='{name}'>
                <link name='link'>
                  <collision name='collision'>
                    <geometry><cylinder><radius>0.1</radius><length>0.3</length></cylinder></geometry>
                  </collision>
                  <visual name='visual'>
                    <geometry><cylinder><radius>0.1</radius><length>0.3</length></cylinder></geometry>
                  </visual>
                </link>
              </model>
            </sdf>
            """
        else:
            xml = f"""
            <sdf version='1.6'>
              <model name='{name}'>
                <link name='link'>
                  <collision name='collision'>
                    <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
                  </collision>
                  <visual name='visual'>
                    <geometry><box><size>0.2 0.2 0.2</size></box></geometry>
                  </visual>
                </link>
              </model>
            </sdf>
            """

        request.xml = xml
        pos_x = random.uniform(self.x_range[0], self.x_range[1])
        
        # Basic avoidance for the Home starting area
        if lane_y == 1.0 and pos_x < -0.8:
            pos_x = 0.5

        request.initial_pose.position.x = pos_x
        request.initial_pose.position.y = float(lane_y)
        request.initial_pose.position.z = 0.2 
        
        self.get_logger().info(f'Spawning {name} at x: {pos_x:.2f}, y: {lane_y}')
        self.spawn_client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectSpawner()
    time.sleep(2.0) 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()