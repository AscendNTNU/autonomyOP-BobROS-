import rclpy, time, math
from typing import List
from rclpy.node import Node

from typing import Any
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from mineros_inter.msg import Item
from mineros_inter.srv import FindBlocks, MineBlock, Inventory, Craft

class BobBotNode(Node):
    def __init__(self) -> None:
        super().__init__('bob_bot')
        self.position: PoseStamped = None
        self.stage = 0
        self.pos_reached = False
        self.get_logger().info('Hello World!')

        self.BLOCKID_STONE = 1
        self.BLOCKID_DIRT = 9
        self.BLOCKID_COAL = 43
        self.BLOCKID_WOOD_OAK_LOG = 46
        self.BLOCKID_CRAFTING_TABLE = 182
        self.BLOCKID_DIAMOND = 179

        self.ITEMID_COBBLESTONE = 22
        self.ITEMID_OAK_PLANK = 23
        self.ITEMID_WOOD_OAK_LOG = 110
        self.ITEMID_CRAFTING_TABLE = 278
        self.ITEMID_COAL = 762
        self.ITEMID_WOOD_PICKAXE = 779
        self.ITEMID_STONE_PICKAXE = 784
        self.ITEMID_DIAMOND_PICKAXE = 799
        self.ITEMID_STICK = 807

        self.inventory_contents_service = self.create_client(
            Inventory,
            '/mineros/inventory/contents'
        )

        self.local_position_sub = self.create_subscription(
            PoseStamped,
            '/mineros/local_position/pose',
            self.position_cb,
            10
        )

        self.target_position_reached_sub = self.create_subscription(
            Empty,
            '/mineros/set_position/reached',
            self.target_position_reached_cb,
            10
        )

        self.set_position_pub = self.create_publisher(
            PoseStamped,
            '/mineros/set_position',
            10
        )
        
        self.set_position_composite_pub = self.create_publisher(
            PoseArray,
            '/mineros/set_position/composite',
            10
        )

        self.find_block_client = self.create_client(
            FindBlocks,
            '/mineros/mining/find_blocks'
        )
        
        self.mine_block_client = self.create_client(
            MineBlock,
            '/mineros/mining/mine_block'
        )
        
        self.craft_client = self.create_client(
            Craft,
            '/mineros/interaction/craft'
        )

        self.fsm_test()

    def find_blocks(self, blockid: int, count: int, max_distance) -> List[Pose]:
        self.get_logger().info('Find blocks')
        block_search = FindBlocks.Request()
        block_search.blockid = blockid
        block_search.count = count
        block_search.max_distance = max_distance

        while not self.find_block_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service')
        future = self.find_block_client.call_async(block_search)
        rclpy.spin_until_future_complete(self, future)
        blocks = future.result()
        return blocks.blocks.poses

    def mine_block(self, block: Pose) -> bool:
        req = MineBlock.Request()
        req.block = block

        future = self.mine_block_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        success = future.result()

        if success.success: print('Mine block success.')
        else: print('Mine block failed.')
        return success.success

    def mine_block_list(self, block_pose_list) -> bool:
        for block_pose in block_pose_list:
            result = self.mine_block(block_pose)
            if not result: 
                return False
            time.sleep(.5)
        return True

    def get_inventory_item_count(self, item_id):
        inv_req = Inventory.Request()
        fut = self.inventory_contents_service.call_async(inv_req)
        rclpy.spin_until_future_complete(self, fut)
        inventory = fut.result().inventory

        count = 0
        for item in inventory:
            if item.id == item_id:
                count += item.count

        self.get_logger().info(f'Looking for item_id {item_id} in inventory, {count} found.')
        return count
    
    def print_inventory(self):
        inv_req = Inventory.Request()
        fut = self.inventory_contents_service.call_async(inv_req)
        rclpy.spin_until_future_complete(self, fut)
        inventory = fut.result().inventory

        print('----------------- CURRENT INVENTORY -----------------')
        if len(inventory) < 1:
            print('inventory empty')
        else:
            string = '\n'
            for item in inventory:
                string += f'Item: {item.display_name}, Amount: {item.count}, Id: {item.id}\n'
            print(string)
        print('-----------------------------------------------------')

    def aquire_blocks(self, block_id, item_id, count, max_distance):
        current_item_count = self.get_inventory_item_count(item_id)
        while (current_item_count < count):
            print('Block count less than requested block count.')
            blocks = self.find_blocks(block_id, max(1, count - current_item_count), max_distance)
            if len(blocks) > 0:
                self.mine_block_list(blocks)
            else:
                print('No blocks of desired type found.')
                return False
            time.sleep(.5)
            current_item_count = self.get_inventory_item_count(item_id)
        return True

    def craft_item(self, item_id, count, requires_crafting_table):
        crafting_req = Craft.Request()
        item = Item()
        item.id = item_id
        item.count = count
        crafting_req.item = item
        crafting_req.crafting_table = requires_crafting_table
        if requires_crafting_table: 
            blocks = self.find_blocks(self.BLOCKID_CRAFTING_TABLE, 1, 16)
            if len(blocks) < 1:
                print('Required crafting table but none was found.')
                return False
            block: Pose = blocks[0]
            crafting_req.crafting_table_location = block
        
        future = self.craft_client.call_async(crafting_req)
        rclpy.spin_until_future_complete(self, future)
        return True

    def position_cb(self, msg: PoseStamped):
        self.position = msg
    
    def target_position_reached_cb(self, response):
        self.pos_reached = True
        self.get_logger().info('walk to pose successful')

    def fsm_test(self):
        print('------------------------ fsm start ------------------------')
        self.print_inventory()
        self.aquire_blocks(self.BLOCKID_WOOD_OAK_LOG, self.ITEMID_WOOD_OAK_LOG, 3, 16)
        self.craft_item(self.ITEMID_OAK_PLANK, 3, False)
        self.craft_item(self.ITEMID_STICK, 3, False)
        self.craft_item(self.ITEMID_CRAFTING_TABLE, 1, False)
        self.craft_item(self.ITEMID_WOOD_PICKAXE, 1, True)
        self.aquire_blocks(self.BLOCKID_STONE, self.ITEMID_COBBLESTONE, 3, 16)
        self.craft_item(self.ITEMID_STONE_PICKAXE, 1, True)
        self.print_inventory()
        print('------------------------- fsm end  ------------------------')

def main(args=None):
    rclpy.init(args=args)
    node = BobBotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()