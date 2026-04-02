from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    supervisor = Node(
        package="controllers",
        executable="supervisor",
        name="supervisor",
        parameters=[{
            "use_replan": LaunchConfiguration("use_replan")
        }],
        output="screen",
        emulate_tty=True
    )

    lqr = Node(
        package="controllers",
        executable="lqr_final",
        name="lqr_final",
        parameters=[{
            "use_noise": LaunchConfiguration("use_noise")
        }],
        output="screen",
        emulate_tty=True
    )
    
    path_draw = Node(
        package="path_makers",
        executable="path_draw",
        name="path_drawer",
        parameters=[{
            "use_replan": LaunchConfiguration("use_replan")
        }],
        output="screen",
        emulate_tty=True
    )
    
    lidar_detect = Node(
        package="perception",
        executable="lidar_detect",
        name="colab_lidar",
        output="screen",
        emulate_tty=True
    )
    
    path_pub = Node(
        package="path_makers",
        executable="path_publisher",
        name="path_publisher",
        output="screen",
        emulate_tty=True
    )
    
    delayed_path_pub = TimerAction(
        period=4.0,   # segundos de espera
        actions=[path_pub]
    )
    
    shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=path_draw,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[EmitEvent(event=Shutdown())]
                 )
            ]
        )
    )
    
    return LaunchDescription([
        supervisor,
        lqr,
        path_draw,
        lidar_detect,
        delayed_path_pub,
        shutdown_handler
    ])
