#!/usr/bin/env python3
from threading import Thread, Lock
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import subprocess
import math
import time

from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda

GRASP_Z_OFFSET  = 0.02   # metres above object centre at grasp
PREGRASP_HEIGHT = 0.15   # metres above object for approach
LIFT_HEIGHT     = 0.25   # metres above object after grasping
GRASP_QUAT      = [0.0, 1.0, 0.0, 0.0]


class PickAndPlace(Node):

    def __init__(self):
        super().__init__("pick_and_place")

        self.callback_group = ReentrantCallbackGroup()
        self.queue_lock     = Lock()
        self.detection_queue = []
        self.busy            = False
        self.carried_model   = None
        self.carrying        = False

        # MoveIt2
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )
        self.moveit2.max_velocity     = 0.1
        self.moveit2.max_acceleration = 0.1

        # Gripper
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        # TF2 for end effector pose
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber
        self.sub = self.create_subscription(
            String, "/color_coordinates",
            self.coords_callback, 10,
            callback_group=self.callback_group
        )

        # Joint configurations
        self.start_joints = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.radians(-125.0)
        ]
        self.home_joints = [
            0.0, 0.0, 0.0,
            math.radians(-90.0), 0.0,
            math.radians(92.0),
            math.radians(50.0)
        ]
        self.drop_joints = {
            "Clip": [
                math.radians(-155.0), math.radians(30.0),
                math.radians(-20.0),  math.radians(-124.0),
                math.radians(44.0),   math.radians(163.0),
                math.radians(7.0)
            ],
            "Rivet": [
                math.radians(-130.0), math.radians(30.0),
                math.radians(-20.0),  math.radians(-124.0),
                math.radians(44.0),   math.radians(163.0),
                math.radians(7.0)
            ],
            "Screw": [
                math.radians(-105.0), math.radians(30.0),
                math.radians(-20.0),  math.radians(-124.0),
                math.radians(44.0),   math.radians(163.0),
                math.radians(7.0)
            ],
        }

        # Move to start
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()
        self.get_logger().info("Ready. Listening for detections...")

    def coords_callback(self, msg):
        try:
            parts       = msg.data.split(",")
            fastener_id = parts[0].strip()
            x           = float(parts[1])
            y           = float(parts[2])
            z           = float(parts[3])
            model_name  = parts[4].strip() if len(parts) > 4 else fastener_id

            if fastener_id not in self.drop_joints:
                return

            with self.queue_lock:
                already = any(
                    abs(d[1]-x) < 0.02 and abs(d[2]-y) < 0.02
                    for d in self.detection_queue
                )
                if not already:
                    self.detection_queue.append(
                        (fastener_id, x, y, z, model_name))
                    self.get_logger().info(
                        f"Queued: {fastener_id} at [{x:.3f},{y:.3f},{z:.3f}]")
        except Exception as e:
            self.get_logger().error(f"Bad message: {e}")

    def set_model_pose(self, model_name, x, y, z):
        """Move a Gazebo model to world position (x,y,z) using ign service."""
        cmd = (
            f'ign service -s /world/empty_world/set_pose '
            f'--reqtype ignition.msgs.Pose '
            f'--reptype ignition.msgs.Boolean '
            f'--timeout 1000 '
            f'--req \'name: "{model_name}" '
            f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}\''
        )
        subprocess.run(cmd, shell=True, capture_output=True)

    def get_ee_pose(self):
        """Get current end effector world position."""
        try:
            t = self.tf_buffer.lookup_transform(
                'world', 'panda_hand',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            return x, y, z
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def carry_object(self, model_name):
        """Continuously teleport object to EE position while carrying."""
        while self.carrying:
            pose = self.get_ee_pose()
            if pose:
                x, y, z = pose
                self.set_model_pose(model_name, x, y, z - 0.05)
            time.sleep(0.05)

    def pick_and_place(self, fastener_id, x, y, z, model_name):
        pregrasp_pos = [x, y, z + PREGRASP_HEIGHT]
        grasp_pos    = [x, y, z + GRASP_Z_OFFSET]
        lift_pos     = [x, y, z + LIFT_HEIGHT]
        drop         = self.drop_joints[fastener_id]

        self.get_logger().info(
            f"Picking {fastener_id} at [{x:.3f},{y:.3f},{z:.3f}]")

        try:
            # 1. Home
            self.moveit2.move_to_configuration(self.home_joints)
            self.moveit2.wait_until_executed()

            # 2. Open gripper
            self.gripper.open()
            self.gripper.wait_until_executed()

            # 3. Pre-grasp
            self.moveit2.move_to_pose(
                position=pregrasp_pos, quat_xyzw=GRASP_QUAT)
            self.moveit2.wait_until_executed()

            # 4. Move down to grasp
            self.moveit2.move_to_pose(
                position=grasp_pos, quat_xyzw=GRASP_QUAT, cartesian=True)
            self.moveit2.wait_until_executed()

            # 5. Close gripper
            self.gripper.close()
            self.gripper.wait_until_executed()

            # 6. Start carrying object (teleport thread)
            self.carrying = True
            carry_thread = Thread(
                target=self.carry_object, args=(model_name,), daemon=True)
            carry_thread.start()

            # 7. Lift
            self.moveit2.move_to_pose(
                position=lift_pos, quat_xyzw=GRASP_QUAT, cartesian=True)
            self.moveit2.wait_until_executed()

            # 8. Move to bin
            self.moveit2.move_to_configuration(drop)
            self.moveit2.wait_until_executed()

            # 9. Stop carrying, open gripper
            self.carrying = False
            carry_thread.join(timeout=1.0)

            # Get current EE pose for drop position
            drop_pose = self.get_ee_pose()
            if drop_pose:
                self.set_model_pose(model_name,
                    drop_pose[0], drop_pose[1], drop_pose[2] - 0.1)

            self.gripper.open()
            self.gripper.wait_until_executed()

            # 10. Return home
            self.moveit2.move_to_configuration(self.home_joints)
            self.moveit2.wait_until_executed()

            self.gripper.close()
            self.gripper.wait_until_executed()

            self.moveit2.move_to_configuration(self.start_joints)
            self.moveit2.wait_until_executed()

            self.get_logger().info(
                f"✓ {fastener_id} placed in {fastener_id} bin")
            return True

        except Exception as e:
            self.get_logger().error(f"Pick failed: {e}")
            self.carrying = False
            try:
                self.gripper.open()
                self.gripper.wait_until_executed()
                self.moveit2.move_to_configuration(self.start_joints)
                self.moveit2.wait_until_executed()
            except Exception:
                pass
            return False

    def process_loop(self):
        while rclpy.ok():
            item = None
            with self.queue_lock:
                if self.detection_queue and not self.busy:
                    item = self.detection_queue.pop(0)
                    self.busy = True
            if item:
                self.pick_and_place(*item)
                self.busy = False
            else:
                time.sleep(0.1)


def main():
    rclpy.init()
    node = PickAndPlace()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        node.process_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
