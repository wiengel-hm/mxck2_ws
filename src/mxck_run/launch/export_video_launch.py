import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix


# Usage examples:
# ros2 launch mxck_run export_video_launch.py bag:=traffic_sign_seq_vol_I.mcap fps:=15.0
# ros2 launch mxck_run export_video_launch.py bag:=/full/path/to/bag.mcap topic:=/camera/depth/image_raw fps:=15.0


def setup_output_path(context, *args, **kwargs):
    """
    Creates output directory structure and generates appropriate filename.
    Structure: <package>/export/<bagfile_name>/<topic_name>.avi
    """
    bag_input = LaunchConfiguration("bag").perform(context)
    topic = LaunchConfiguration("topic").perform(context)
    fps = LaunchConfiguration("fps").perform(context)
    custom_out = LaunchConfiguration("out").perform(context)
    
    # Get package root using ament_index
    package_root = Path(get_package_prefix('mxck_run').replace('install', 'src'))
    
    # Resolve bag path - check if it's a full path or just a filename
    bag_path = Path(bag_input)
    if not bag_path.exists():
        # Try looking in package_root/bagfiles/
        potential_path = package_root / "bagfiles" / bag_input
        if potential_path.exists():
            bag_path = potential_path
            print(f"  Found bag in package bagfiles: {bag_path}")
        else:
            raise FileNotFoundError(
                f"Bag file not found!\n"
                f"  Tried: {bag_input}\n"
                f"  Also tried: {potential_path}"
            )
    
    # Get bag filename without extension
    bag_name = bag_path.stem
    
    # Sanitize topic name for filesystem (remove leading /, replace / with _)
    topic_safe = topic.lstrip('/').replace('/', '_')
    
    # Create export directory structure
    export_dir = package_root / "export" / bag_name
    export_dir.mkdir(parents=True, exist_ok=True)
    
    # Generate output filename
    if custom_out and custom_out != "":
        output_file = export_dir / custom_out
    else:
        output_file = export_dir / f"{topic_safe}.avi"
    
    print(f"\n{'='*60}")
    print(f"Video Export Configuration:")
    print(f"  Bag file: {bag_path}")
    print(f"  Bag name: {bag_name}")
    print(f"  Topic: {topic}")
    print(f"  FPS: {fps}")
    print(f"  Output: {output_file}")
    print(f"{'='*60}\n")
    
    # Launch video recorder
    record_video = ExecuteProcess(
        cmd=[
            "ros2", "run", "image_view", "video_recorder",
            "--ros-args",
            "-r", f"/image:={topic}",
            "-p", f"filename:={str(output_file)}",
            "-p", f"fps:={fps}",
        ],
        output="screen"
    )
    
    # Launch bag playback
    play = ExecuteProcess(
        cmd=["ros2", "bag", "play", str(bag_path)],
        output="screen"
    )
    
    return [record_video, play]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "bag",
            description="Path to bag file or just filename (will check <package>/bagfiles/)"
        ),
        DeclareLaunchArgument(
            "topic",
            default_value="/camera/camera/image_raw",
            description="Image topic to export"
        ),
        DeclareLaunchArgument(
            "out",
            default_value="",
            description="Custom output filename (optional, default: <topic_name>.avi)"
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="30.0",
            description="Frames per second for output video"
        ),
        OpaqueFunction(function=setup_output_path)
    ])