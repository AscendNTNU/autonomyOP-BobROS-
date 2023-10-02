import os
import sys
import time
import unittest
import uuid

import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import launch_testing_ros

import pytest

import rclpy

from std_msgs.msg import String
from time import sleep

@pytest.mark.launch_test
def generate_test_description():

        path_to_test = os.path.dirname(__file__)
        path_to_src = os.path.dirname(os.path.dirname(os.path.realpath(__file__))
 + "/../rclpy_example/")

        publisher_node = launch_ros.actions.Node(
            executable=sys.executable,
            arguments=[os.path.join(path_to_src, 'minimal_publisher.py')],
            additional_env={'PYTHONUNBUFFERED': '1'},
        )

        subscriber_node = launch_ros.actions.Node(
            executable=sys.executable,
            arguments=[os.path.join(path_to_src, 'minimal_subscriber.py')],
            additional_env={'PYTHONUNBUFFERED': '1'},
        )

        return (
        launch.LaunchDescription([
            publisher_node,
            subscriber_node,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'publisher': publisher_node,
            'subscriber': subscriber_node,
        }
    )

class PublisherSubscriberTest(unittest.TestCase):

    publisher_ok = False

    @classmethod
    def setUpClass(cls):
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.test_node = rclpy.create_node("test_node")

    def tearDown(self):
        self.test_node.destroy_node()

    def callback(self, data):
        self.publisher_ok = True

    def test_if_publisher_publishes(self, launch_service, proc_output):
        sub = self.test_node.create_subscription(String, "topic", self.callback, 1)
        print("Created Subscription")

        counter = 0
        while rclpy.ok() and counter < 5 and (not self.publisher_ok):
            rclpy.spin_once(self.test_node, timeout_sec=1)
            sleep(1)
            counter += 1

        self.assertTrue(self.publisher_ok)
        self.test_node.destroy_subscription(sub)

    def test_if_subscriber_receives(self, launch_service, subscriber, proc_output):
        pub = self.test_node.create_publisher(String, "topic", 1)
        try:
            # Publish a unique message on /topic and verify that the listener
            # gets it and prints it
            msg = String()
            msg.data = str(uuid.uuid4())
            for _ in range(10):
                pub.publish(msg)
                success = proc_output.waitFor(
                    expected_output=msg.data,
                    process=subscriber,
                    timeout=1.0,
                )
                if success:
                    break
            assert success, 'Waiting for output timed out'
        finally:
            self.test_node.destroy_publisher(pub)

