import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Trigger
import json
from os import path
from pathlib import Path


class CheckpointsService(Node):

    def __init__(self):
        super().__init__('CheckpointsService')
        self.goal_handle = None
        # self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.p_x,self.p_y,self.q_x,self.q_y,self.q_z,self.q_w = None,None,None,None,None,None
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.srv_start = self.create_service(Trigger, '/saveCheckpoint', self.start_callback)
        # self.srv_cancel = self.create_service(Trigger, '/cancel', self.cancel_callback)
        self.declare_parameter('checkpoints_file', '')


    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.pose.pose.position)
        mcl_p = msg.pose.pose.position
        mcl_q = msg.pose.pose.orientation

        self.p_x = mcl_p.x
        self.p_y = mcl_p.y

        self.q_x = mcl_q.x        
        self.q_y = mcl_q.y
        self.q_z = mcl_q.z
        self.q_w = mcl_q.w


        self.get_logger().info(f'I heard posX: "{self.p_x}", posY: {self.p_y}')


    def start_callback(self, request, response):

        try:
            # Goes up two folders
            base_dir = Path(__file__).resolve().parent.parent 
            filename = base_dir / 'config' / 'map' / 'presentation_checkpoints.json'            
            dictObj = []
        
            # Check if file exists
            if path.isfile(filename) is False:
                raise Exception("File not found")
        
            # Read JSON file
            with open(filename) as fp:
                dictObj = json.load(fp)

            # Append new pose data
            new_pose = {
                "position": {"x": round(self.p_x,2), "y": round(self.p_y,2), "z": 0.0},
                "orientation": {"x": round(self.q_x,2), "y": round(self.q_y,2), "z": round(self.q_z,2), "w": round(self.q_w,2)}
            }        

            dictObj["poses"].append(new_pose)
            
            self.get_logger().info(str(dictObj))


            with open(filename, 'w') as json_file:
                json.dump(dictObj, json_file, indent=4)

            # goal_msg = FollowWaypoints.Goal()
            # goal_msg.poses = poses
            # self._action_client.wait_for_server()
            # self.goal_handle = self._action_client.send_goal_async(goal_msg)

            response.success = True
            response.message = "Success"
            return response
        except FileNotFoundError as e:
            self.get_logger().error (f"{e}")
            response.success = False
            response.message = f"{e}"
            return response


    # def cancel_callback(self, request, response):
    #     self.goal_handle.result().cancel_goal_async()

    #     response.success = True
    #     response.message = "Success"
    #     return response

def main(args=None):
    rclpy.init(args=args)

    service_client = CheckpointsService()
    rclpy.spin(service_client)



if __name__ == '__main__':
    main()