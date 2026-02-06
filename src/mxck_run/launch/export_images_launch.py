import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix


# Usage examples:
# ros2 launch mxck_run export_images_launch.py bag:=traffic_sign_seq_vol_II.mcap
# ros2 launch mxck_run export_images_launch.py bag:=/full/path/to/bag.mcap topic:=/camera/depth/image_raw


def setup_output_path(context, *args, **kwargs):
    """
    Creates output directory structure and generates appropriate path.
    Structure: <package>/export/<bagfile_name>/<topic_name>/frame_XXXX.jpg
    """
    bag_input = LaunchConfiguration("bag").perform(context)
    topic = LaunchConfiguration("topic").perform(context)
    custom_out = LaunchConfiguration("out_dir").perform(context)
    
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
    if custom_out and custom_out != "":
        # User provided custom directory
        export_dir = package_root / "export" / bag_name / custom_out
    else:
        # Auto-generate based on topic name
        export_dir = package_root / "export" / bag_name / topic_safe
    
    export_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"\n{'='*60}")
    print(f"Image Export Configuration:")
    print(f"  Bag file: {bag_path}")
    print(f"  Bag name: {bag_name}")
    print(f"  Topic: {topic}")
    print(f"  Output directory: {export_dir}")
    print(f"{'='*60}\n")
    
    # Extract images in the output directory
    extract_images = ExecuteProcess(
        cmd=[
            "bash", "-c",
            f"cd \"{export_dir}\" && "
            f"ros2 run image_view extract_images --ros-args -r image:=\"{topic}\""
        ],
        output="screen"
    )
    
    # Play the bag (start after extractor so no frames are missed)
    play = ExecuteProcess(
        cmd=["ros2", "bag", "play", str(bag_path)],
        output="screen"
    )
    
    return [extract_images, play]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "bag",
            description="Path to bag file or just filename (will check <package>/bagfiles/)"
        ),
        DeclareLaunchArgument(
            "topic",
            default_value="/camera/camera/color/image_raw",
            description="Image topic to extract"
        ),
        DeclareLaunchArgument(
            "out_dir",
            default_value="",
            description="Custom output subdirectory name (optional, default: <topic_name>)"
        ),
        OpaqueFunction(function=setup_output_path)
    ])