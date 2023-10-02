import rclpy
from rclpy.node import Node

from typing import Any, List
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
import copy
from mineros_inter.srv import BlockInfo, FindBlocks, Craft, FurnaceInfo
from mineros_inter.msg import Item, Furnace
from time import sleep
import pprint

class CrafterNode(Node):
    """
    This class encompases the skeleton of a ROS2 node. It inherits from the class Node, which is the base class
    for all nodes in ROS2. The class Node is defined in the rclpy package. The class Node has a constructor that
    takes the name of the node as an argument. The name of the node is the name that will be used to identify the
    node in the ROS2 ecosystem. The name of the node must be unique in the ROS2 ecosystem. If two nodes have the
    same name, then the second node will not be able to start.
    
    The aim of this node will be to create a small node that will allow the bot to move around and navigate its
    space
    """
    def __init__(self) -> None:
        """
        The constructor of the class. It initializes the node with the name 'my_first_node'. In this method
        we also initialize the publishers and subscribers that we want to use in the node. As well as all the possible
        parameters that we want to be able to change from the launch file.
        """
        super().__init__('crafter_node')
        self.get_logger().info('Hewwo Wowwd!')
        
        # Clients
        self.find_block_client = self.create_client(
            FindBlocks,
            '/mineros/mining/find_blocks'
        )
        self.crafting_client = self.create_client(
            Craft,
            '/mineros/interaction/craft'
        )
        self.furnace_client = self.create_client(
            FurnaceInfo,
            '/mineros/interaction/furnace_info'  
        )
        # Publishers
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/mineros/set_position',
            10
        )
############################################################################################################################################################################################################
        #Furnace code
        self.test_crafting()
        self.get_logger().info("TRY")
        furnace = self.get_furnace()
        self.get_logger().info("Funnet furnace")
        furnace_info = self.check_furnace(furnace)
        self.get_logger().info(f"Progress: {furnace_info.progress}")

        


############################################################################################################################################################################################################

    def get_furnace(self, max_distance = 16) -> Pose: # Gir exception om det ikke er noen furnacer i nærheten
        furnace_array = self.find_blocks(185,1, max_distance)

        self.get_logger().info(f"Funnet {len(furnace_array)} ovn{'er'*(len(furnace_array)!=1)}!")
        if len(furnace_array) != 0:
            return furnace_array[0]
        else: # 
            return Pose()
    def check_furnace(self, furnace: Pose) -> Furnace:
        furnace_request = FurnaceInfo.Request()
        furnace_request.block_pose = furnace
        while not self.furnace_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service (FurnaceInfo)')
        self.get_logger().info("Funnet service")
        future = self.furnace_client.call_async(furnace_request)
        rclpy.spin_until_future_complete(self,future)
        self.get_logger().info("Er det en suksess?")
        if future.result().success:
            self.get_logger().info("Suksess!")
            return future.result().furnace
        else:
            self.get_logger().info("Usuksess")
            return Furnace()

    def move_to_pose(self, msg):
        self.pose_publisher.publish(msg) 

    def find_blocks(self, blockid: int, count: int, max_distance = 16) -> List[Pose]:
        self.get_logger().info('Find blocks')
        block_search = FindBlocks.Request()
        block_search.blockid = blockid
        block_search.count = count
        block_search.max_distance = max_distance

        while not self.find_block_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service (FindBlocks)')
        future = self.find_block_client.call_async(block_search)
        rclpy.spin_until_future_complete(self, future)
        blocks = future.result()
        return blocks.blocks.poses   
    
    def craft_item(self,itemID,count,use_crafting_table,crafting_table_pose):
        self.get_logger().info(f"Crafting item {itemID} {'with a crafting table' * use_crafting_table}")
        crafting_request = Craft.Request()
        crafting_request.item = Item(); crafting_request.item.id = itemID; crafting_request.item.count = count
        crafting_request.crafting_table = use_crafting_table
        crafting_request.crafting_table_location = crafting_table_pose

        while not self.crafting_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service (Craft)')
        future = self.crafting_client.call_async(crafting_request)
        rclpy.spin_until_future_complete(self,future)
        out = future.result().success
        self.get_logger().info(f"{'Crafting success' * out}{'Crafting failed' * (not out)}")
        return out
    
    def test_crafting(self):
        self.crafting_table_pose = self.find_blocks(182,1)
        if len(self.crafting_table_pose) != 0:
            self.crafting_table_pose = self.crafting_table_pose[0]
        else:
            self.get_logger().info("No table ò_ó")
            return
        self.get_logger().info(f"{self.crafting_table_pose.position}")
    
        item = 807 #bør være stick
        count = 1
        false = False
        true = True
        crafting_table = false
        self.craft_item(item,count,crafting_table,self.crafting_table_pose)

        item = 799 #bør være diamond pickaxe
        count = 1
        false = False
        true = True
        crafting_table = true
        self.craft_item(item,count,crafting_table,self.crafting_table_pose)


           
            
def main(args=None):
    rclpy.init(args=args)
    node = CrafterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()    