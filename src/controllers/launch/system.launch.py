from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    
    experiment_id = LaunchConfiguration("experiment_id")
    use_replan = LaunchConfiguration("use_replan")
    use_noise = LaunchConfiguration("use_noise")
    
    declare_experiment = DeclareLaunchArgument(
    "experiment_id",
    default_value="0"
    )
    
    declare_use_replan = DeclareLaunchArgument(
        "use_replan",
        default_value="true",
        description="Activar o desactivar replanificación"
    )
    
    declare_use_noise = DeclareLaunchArgument(
        "use_noise",
        default_value="false",
        description="Activar o desactivar ruido en actuadores"
    )
    
    # ==================== Stage launch =======================
    stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("controllers"),
                "launch",
                "reactive_rrt.launch.py"
            )
        ),
        launch_arguments={
            "world": "cave_mod",
            "use_stamped_velocity": "false"
        }.items()
    )


    # ============ Controllers launch ================
    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("controllers"),
                "launch",
                "master.launch.py"
            )
        ),
        launch_arguments={
            "experiment_id": experiment_id,
            "use_replan": use_replan,
            "use_noise": use_noise
        }.items()
    )

    return LaunchDescription([
        declare_experiment,
        declare_use_replan,
        declare_use_noise,
        stage_launch,
        controllers_launch
    ])
