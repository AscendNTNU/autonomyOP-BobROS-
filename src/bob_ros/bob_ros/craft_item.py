import rclpy
from rclpy.node import Node

from typing import Any, List, Dict
import math
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
import copy
from mineros_inter.srv import BlockInfo, FindBlocks, Craft, FurnaceInfo, Inventory, PlaceBlock, BotPos, FurnaceUpdate
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
    fuel_hierarchy = {}
    for i in [762, 763]:
        fuel_hierarchy[i] = 8
    for i in range(23, 34):
        fuel_hierarchy[i] = 1.5
    logs = [i for i in range(110, 117)]
    planks = [i for i in range(23, 30)]
    plank_to_log = {}
    for i in range(7):
        plank_to_log[planks[i]] = logs[i]
    CT_BLOCKID = 182
    CT_ITEMID = 278
    IRON_ITEMID = 333
    RIRON_ITEMID = 769

    FURNACE_SPEED = 10

    def __init__(self) -> None:
        """
        The constructor of the class. It initializes the node with the name 'my_first_node'. In this method
        we also initialize the publishers and subscribers that we want to use in the node. As well as all the possible
        parameters that we want to be able to change from the launch file.
        """
        super().__init__('crafter_node')
        # self.get_logger().info(f"{self.fuel_hierarchy}")
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
        self.furnace_info_client = self.create_client(
            FurnaceInfo,
            '/mineros/interaction/furnace_info'
        )
        self.inventory_contents_service = self.create_client(
            Inventory,
            '/mineros/inventory/contents'
        )
        self.place_block_client = self.create_client(
            PlaceBlock,
            '/mineros/interaction/place_block'
        )
        self.get_position_client = self.create_client(
            BotPos,
            '/mineros/local_position/pose'
        )
        self.furnace_usage_client = self.create_client(
            FurnaceUpdate,
            '/mineros/interaction/furnace_update'
        )
        # Publishers
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/mineros/set_position',
            10
        )
        self.look_at_block_publisher = self.create_publisher(
            PoseStamped,
            '/mineros/set_look_at_block',
            10
        )
############################################################################################################################################################################################################
        # Furnace code
        item_count = 3
        self.get_logger().info(f"{self.optimize_fuel(3)}")
        suksess = self.use_furnace(self.RIRON_ITEMID, item_count)
        self.get_logger().error(f"{suksess}")


############################################################################################################################################################################################################

    # Gir exception om det ikke er noen furnacer i nærheten

    def get_furnace(self, max_distance=16) -> Pose:
        furnace_array = self.find_blocks(185, 1, max_distance)

        self.get_logger().info(
            f"Funnet {len(furnace_array)} ovn{'er'*(len(furnace_array)!=1)}!")
        if len(furnace_array) != 0:
            return furnace_array[0]
        else:
            return Pose()

    def check_furnace(self, furnace: Pose) -> Furnace:
        furnace_PoseStaped = PoseStamped()
        furnace_PoseStaped.pose = furnace
        self.look_at_block(furnace_PoseStaped)
        furnace_request = FurnaceInfo.Request()
        furnace_request.block_pose = furnace
        while not self.furnace_info_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service (FurnaceInfo)')
        future = self.furnace_info_client.call_async(furnace_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            return future.result().furnace
        else:
            return Furnace()

    def use_furnace(self, item_id: int, count: int) -> bool:
        furnace_pose = self.get_furnace()
        if self.get_item_stock([item_id])[item_id] < count and furnace_pose == Pose():
            return (false := False)
        
        fuel_plan = list(self.optimize_fuel(count).items())
        fuel_plan = [fuel for fuel in fuel_plan if fuel[1] > 0]
        get_fuel_wait_time = lambda fuel,count: self.fuel_hierarchy[fuel] * count * self.FURNACE_SPEED 

        self.refill_furnace(furnace_pose, item_id, count)
        sleep_buffer = 0
        for fuel in fuel_plan:
            self.refuel_furnace(furnace_pose, fuel[0], fuel[1])
            sleep(sleep_buffer)
            sleep_count = max(1, fuel[1]-1)
            sleep_buffer = get_fuel_wait_time(fuel[0],min(1, fuel[1]-1))
            sleep(get_fuel_wait_time(fuel[0],sleep_count)) 
        return self.get_foutput(furnace_pose)             



    def get_fuel(self) -> float:
        total_fuel = 0
        fuel_inventory = self.get_item_stock(
            self.fuel_hierarchy.keys())
        for curfuel in self.fuel_hierarchy.keys():
            total_fuel += fuel_inventory[curfuel] * \
                self.fuel_hierarchy[curfuel]
        return total_fuel

    def optimize_fuel(self, count: int):
        remaining_fuel = count
        count_dictionary = dict((item, 0)
                                for item in self.fuel_hierarchy.keys())
        fuel_inventory = self.get_item_stock(
            list(self.fuel_hierarchy.keys()))
        log_inventory = self.get_item_stock(self.logs)
        log_count = sum(list(log_inventory.values()))
        #self.get_logger().info(f"{log_count}")
        
        # Legg til brennstoff uten å gå over
        for fuel in self.fuel_hierarchy.keys():
            fuel_strength = self.fuel_hierarchy[fuel]
            if fuel_strength <= remaining_fuel:
                fuel_to_add = min(
                    (remaining_fuel//fuel_strength), fuel_inventory[fuel])
                count_dictionary[fuel] += fuel_to_add
                remaining_fuel -= fuel_to_add * fuel_strength
        if remaining_fuel <= 0:
            return count_dictionary
        
        if self.get_fuel() >= count:
            # Legg til brennstoff selv om det er for sterkt
                for fuel in list(self.fuel_hierarchy.keys())[::-1]:
                    fuel_strength = self.fuel_hierarchy[fuel]
                    fuel_to_add = min(
                        math.ceil((remaining_fuel/fuel_strength)), fuel_inventory[fuel])
                    count_dictionary[fuel] += fuel_to_add
                    remaining_fuel -= fuel_to_add * fuel_strength
                    if remaining_fuel <= 0:
                        break
        else:
            if log_count * 6 > remaining_fuel:
                for plank in self.planks:
                    plank_amount = min((int) (4*(max(6,remaining_fuel)//6))
                                       , log_inventory[self.plank_to_log[plank]])
                    while plank_amount > 0:
                        if not self.primal_craft_item(plank, plank_amount, False, Pose()):
                            plank_amount -= 4
                        else:
                            planks_to_add = max(0, min(4*plank_amount, math.ceil(remaining_fuel/1.5)))
                            remaining_fuel -= 1.5*planks_to_add
                            count_dictionary[plank] += planks_to_add
                            break
            else:
                return {list(self.fuel_hierarchy.keys())[0]: -math.ceil(remaining_fuel/8)}
        return count_dictionary
    
    def update_furnace(self, furnace: Furnace, furnace_pos: Pose, ignore_fuel: bool):
        request = FurnaceUpdate.Request()
        request.furnace = furnace
        request.block_pose = furnace_pos
        request.ignore_fuel = ignore_fuel
        while not self.furnace_usage_client.wait_for_service(timeout_sec=1.0):
            self.get_logger.info("Waiting for service (FurnaceUpdate)")
        future = self.furnace_usage_client.call_async(request)
        rclpy.spin_until_future_complete(self,future)
        return future.result().success
    
    def refill_furnace(self, furnace_pos: Pose, itemID : int, count :int) -> bool:
        input = Item(); input.id = itemID; input.count = count
        furnace = self.check_furnace(furnace_pos)
        output_furnace = Furnace()
        output_furnace.input_item = input
        output_furnace.output_item = furnace.output_item
        if input.id != furnace.input_item.id:
            output_furnace.output_item.count = 0
        return self.update_furnace(output_furnace, furnace_pos,True)

    def get_foutput(self, furnace_pos: Pose) -> bool:
        furnace = self.check_furnace(furnace_pos)
        furnace.output_item.count = 0
        return self.update_furnace(furnace, furnace_pos,True)
    
    def refuel_furnace(self, furnace_pos: Pose, fuelID : int, count: int) -> bool:
        furnace = self.check_furnace(furnace_pos)
        fuel = Item(); fuel.id = fuelID; fuel.count = count
        furnace.fuel_item = fuel
        return self.update_furnace(furnace, furnace_pos, False)

    def move_to_pose(self, pose: PoseStamped):
        self.pose_publisher.publish(pose)

    def look_at_block(self, block: PoseStamped):
        self.look_at_block_publisher.publish(block)

    def place_block(self,itemID: int, position: Pose, face_vector = (0,1,0)) -> Pose:
        item = Item(); item.id = itemID; item.count = 1
        request = PlaceBlock.Request()
        request.block_pose.position = position
        request.block.block_pose.position = position
        request.block.block = item
        # FULLFØR DENNE FUNKSJONEN


    def get_bot_pos(self) -> PoseStamped:
        request = BotPos.Request()
        while not self.get_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for service (BotPos)')
        future = self.get_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    def find_blocks(self, blockid: int, count: int, max_distance=16) -> List[Pose]:
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

    def ensure_block(self, blockID: int,itemID: int, crafting_requires_table: bool = False) -> Pose:   
        pose_list = self.find_blocks(blockID,1)
        if len(pose_list) > 0:
            return pose_list[0]
        if self.get_item_stock([itemID])[itemID] > 0:
            # plasser block og returner plassering
            return Pose()
        assert self.craft_item(itemID, 1, crafting_requires_table)
        self.ensure_block(blockID,itemID,crafting_requires_table)
            


    def craft_item(self,itemID: int, count: int, use_crafting_table: bool = True) -> bool:
        crafting_table_pose = self.find_blocks(self.CT_BLOCKID,1)
        if len(crafting_table_pose) == 0:
            # Lag crafting table
            if self.get_item_stock([self.CT_ITEMID]) > 0:
               self.place_block(self.CT_BLOCKID) 
            crafting_table_pose.append(Pose())
        return self.primal_craft_item(itemID, count, use_crafting_table, crafting_table_pose[0])
    
    def primal_craft_item(self, itemID: int, count: int, use_crafting_table: bool, crafting_table_pose: Pose = Pose()) -> bool:
        ct_PoseStamped = PoseStamped()
        ct_PoseStamped.pose = crafting_table_pose
        if use_crafting_table:
            self.look_at_block(ct_PoseStamped)
        self.get_logger().info(
            f"Crafting {count} item {itemID} {'with a crafting table' * use_crafting_table}")
        crafting_request = Craft.Request()
        crafting_request.item = Item()
        crafting_request.item.id = itemID
        crafting_request.item.count = count
        crafting_request.crafting_table = use_crafting_table
        crafting_request.crafting_table_location = crafting_table_pose

        while not self.crafting_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service (Craft)')
        future = self.crafting_client.call_async(crafting_request)
        rclpy.spin_until_future_complete(self, future)
        out = future.result().success
        self.get_logger().info(
            f"{'Crafting success' * out}{'Crafting failed' * (not out)}")
        return out

    def test_crafting(self):
        self.crafting_table_pose = self.find_blocks(182, 1)
        if len(self.crafting_table_pose) != 0:
            self.crafting_table_pose = self.crafting_table_pose[0]
        else:
            self.get_logger().info("No table ò_ó")
            return
        self.get_logger().info(f"{self.crafting_table_pose.position}")

        item = 807  # bør være stick
        count = 1
        false = False
        true = True
        crafting_table = false
        self.primal_craft_item(item, count, crafting_table, self.crafting_table_pose)

        item = 799  # bør være diamond pickaxe
        count = 1
        false = False
        true = True
        crafting_table = true
        self.primal_craft_item(item, count, crafting_table, self.crafting_table_pose)

    def test_check_furnace(self):
        furnace = self.get_furnace()
        furnace_info = self.check_furnace(furnace)

    def test_fuel_optimization(self):
        logitems = self.get_item_stock(self.logs)
        self.get_logger().error(f"{logitems}")
        

        fuel_goal = 64
        
        while True:
            self.get_logger().info("Printing dictionary:")
            self.get_logger().info(f"{self.optimize_fuel(fuel_goal)}")
            sleep(5)

    def get_item_stock(self, item_id_list: List[int]) -> Dict[int, int]:
        inv_req = Inventory.Request()
        #self.get_logger().info("In getinvmul")
        fut = self.inventory_contents_service.call_async(inv_req)
        #self.get_logger().warn("Called service")
        rclpy.spin_until_future_complete(self, fut)
        #self.get_logger().error("Future completed")
        inventory = fut.result().inventory
        count_dictionary = dict((item, 0) for item in item_id_list)
        for item in inventory:
            if item.id in count_dictionary.keys():
                count_dictionary[item.id] += item.count
        #self.get_logger().info(f"get inv mul: {count_dictionary}")
        return count_dictionary


def main(args=None):
    rclpy.init(args=args)
    node = CrafterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
