from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions  import ParameterValue
from launch.substitutions import Command
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
     #路径设置
    planning_path = get_package_share_directory("planning")

     #主车模型设置
    car_path = os.path.join(planning_path, "urdf/main_car", "car.xacro")

     #xacro 命令行指令，封装参数
    car_para = ParameterValue(
        Command(["xacro ", car_path]),
    )

     #启动robot_state_publisher 节点，以参数的方式加载文件内容
    car_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="car_state_pub",
        output="screen",
        parameters=[{"robot_description": car_para}],
    )

     #启动joint_state_publisher 节点
    car_joint_state_pub_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="car_joint_state_pub",
    )

    #启动rviz2 节点
    rviz2 = Node(
    package="rviz2",
    executable="rviz2",
    )    

    #启动绘图节点

    #启动全局规划

    #启动地图服务器

    #启动planning 节点
    planning_node = Node(
        package="planning",
        executable="planning",
        name="planning_node",
    )

    #节点分组
    car_main = GroupAction(
        actions=[
            PushRosNamespace("car"),
            car_state_pub,
            car_joint_state_pub_gui,
        ]
    )

    planning = GroupAction(
        actions=[
            PushRosNamespace("planning"),
            planning_node,
        ]
    )

    return LaunchDescription([
        car_main,
        planning,
    ])