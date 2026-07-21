#!/usr/bin/env python3
"""
go2_lowstate_to_joint_states_bridge.py

Bridge node that lets RViz show the Unitree Go2 moving with its REAL joints.

unitree_ros2 publishes the raw robot state on `/lowstate` (unitree_go/msg/LowState),
which RViz cannot render on its own. This node translates that into the two things
RViz + robot_state_publisher need:

  1. /joint_states (sensor_msgs/msg/JointState)
       The 12 leg-joint angles, read from LowState.motor_state[i].q and mapped to the
       joint NAMES used by the go2_description URDF. robot_state_publisher consumes
       this and emits the TF tree for the whole leg kinematics, so the mesh articulates.

  2. odom -> base transform (tf2)
       So the whole robot model is placed at the robot's estimated pose in the world.
       Taken from the lidar odometry topic (/utlidar/robot_odom, nav_msgs/Odometry),
       which is expressed in the same `odom` frame as the lidar point clouds — so the
       model and the clouds line up. Can be disabled with a parameter.

IMPORTANT — joint index mapping:
  The URDF lists joints in the order FL, FR, RL, RR, but the Unitree LowState
  `motor_state` array is in the order FR, FL, RR, RL (hip, thigh, calf each). We map
  each URDF joint NAME to its explicit motor-array INDEX below, so order can never
  silently drift.

QoS:
  The robot publishes several topics BEST_EFFORT. A BEST_EFFORT subscriber is
  compatible with BOTH best-effort and reliable publishers, so we subscribe with the
  sensor-data QoS profile to reliably receive /lowstate and /utlidar/robot_odom.

Run (inside the devcontainer, after `source /workspace/setup.sh`):
    python3 go2_lowstate_to_joint_states_bridge.py
Usually you launch it via go2_full_visualization.launch.py instead.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from unitree_go.msg import LowState


# go2_description URDF joint NAME -> index into LowState.motor_state (Unitree Go2
# convention: FR, FL, RR, RL; each leg is hip, thigh, calf).
URDF_JOINT_NAME_TO_LOWSTATE_MOTOR_INDEX = {
    "FR_hip_joint": 0, "FR_thigh_joint": 1, "FR_calf_joint": 2,
    "FL_hip_joint": 3, "FL_thigh_joint": 4, "FL_calf_joint": 5,
    "RR_hip_joint": 6, "RR_thigh_joint": 7, "RR_calf_joint": 8,
    "RL_hip_joint": 9, "RL_thigh_joint": 10, "RL_calf_joint": 11,
}


class GoTwoLowStateToJointStatesBridge(Node):
    def __init__(self):
        super().__init__("go2_lowstate_to_joint_states_bridge")

        # --- Parameters (all overridable from the launch file) ----------------
        self.declare_parameter("low_state_topic", "lowstate")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("publish_base_transform_from_odometry", True)
        self.declare_parameter("odometry_topic", "utlidar/robot_odom")
        self.declare_parameter("odometry_parent_frame", "odom")
        self.declare_parameter("robot_base_frame", "base")

        low_state_topic = self.get_parameter("low_state_topic").value
        joint_states_topic = self.get_parameter("joint_states_topic").value
        self._publish_base_tf = self.get_parameter(
            "publish_base_transform_from_odometry").value
        odometry_topic = self.get_parameter("odometry_topic").value
        self._odom_parent_frame = self.get_parameter("odometry_parent_frame").value
        self._robot_base_frame = self.get_parameter("robot_base_frame").value

        # Fixed, explicit joint-name order (keys iterate in insertion order).
        self._joint_names = list(URDF_JOINT_NAME_TO_LOWSTATE_MOTOR_INDEX.keys())
        self._joint_motor_indices = list(
            URDF_JOINT_NAME_TO_LOWSTATE_MOTOR_INDEX.values())

        # --- Publishers / subscribers ----------------------------------------
        # robot_state_publisher subscribes /joint_states with default (reliable) QoS.
        self._joint_states_publisher = self.create_publisher(
            JointState, joint_states_topic, 10)
        self.create_subscription(
            LowState, low_state_topic, self._on_low_state, qos_profile_sensor_data)

        self._missing_motor_warned = False

        if self._publish_base_tf:
            self._tf_broadcaster = TransformBroadcaster(self)
            self.create_subscription(
                Odometry, odometry_topic, self._on_odometry, qos_profile_sensor_data)
            self.get_logger().info(
                f"Publishing TF {self._odom_parent_frame} -> {self._robot_base_frame} "
                f"from '{odometry_topic}'.")

        self.get_logger().info(
            f"Bridging '{low_state_topic}' (LowState) -> '{joint_states_topic}' "
            f"(JointState) for {len(self._joint_names)} Go2 leg joints.")

    def _on_low_state(self, low_state: LowState) -> None:
        """Turn one LowState into a JointState with the 12 leg-joint angles."""
        motor_states = low_state.motor_state
        if len(motor_states) <= max(self._joint_motor_indices):
            if not self._missing_motor_warned:
                self.get_logger().warn(
                    f"LowState.motor_state has only {len(motor_states)} entries; "
                    "expected at least 12. Skipping until it grows.")
                self._missing_motor_warned = True
            return

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self._joint_names
        joint_state.position = [
            float(motor_states[i].q) for i in self._joint_motor_indices]
        self._joint_states_publisher.publish(joint_state)

    def _on_odometry(self, odometry: Odometry) -> None:
        """Broadcast odom -> base so the model rides at the robot's estimated pose."""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._odom_parent_frame
        transform.child_frame_id = self._robot_base_frame
        transform.transform.translation.x = odometry.pose.pose.position.x
        transform.transform.translation.y = odometry.pose.pose.position.y
        transform.transform.translation.z = odometry.pose.pose.position.z
        transform.transform.rotation = odometry.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = GoTwoLowStateToJointStatesBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
