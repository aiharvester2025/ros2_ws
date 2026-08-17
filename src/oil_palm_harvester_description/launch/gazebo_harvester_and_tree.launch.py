"""Launch a movable harvester and a static oil-palm tree in Gazebo and RViz.

The harvester model is converted from the same kinematic URDF used by RViz at
launch time.  This avoids the large-URDF ``spawn_entity`` race in Foxy while
keeping the Gazebo joint names identical to the joint-state-publisher GUI.
The tree is deliberately only a static Gazebo world model.
"""

import os
import subprocess
import tempfile
from copy import deepcopy
from pathlib import Path
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression


def _dynamic_world_from_urdf(
        urdf_path: Path, harvester_share: Path, render_mode: str,
        harvester_collision_mode: str, docking_camera: str,
        articulation_control_mode: str) -> str:
    """Create an SDF world containing a movable, commanded harvester.

    Gazebo's own URDF converter handles the joint reference-frame semantics,
    including the fixed-link lumping done by Gazebo Classic.  The harvester
    remains non-static and can move through ``/harvester/cmd_vel``.  In the
    default sensor-development mode, its own collision bodies are removed:
    GUI pose changes are kinematic commands and must not inject contact forces
    that can make the mobile base fly.  The static tree keeps its collision
    geometry for future sensor simulation.
    """
    conversion = subprocess.run(
        ['gz', 'sdf', '-p', str(urdf_path)],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if render_mode not in ('mesh', 'primitive'):
        raise RuntimeError(
            "render_mode must be 'mesh' or 'primitive'; "
            f"received {render_mode!r}.")
    if harvester_collision_mode not in ('off', 'on'):
        raise RuntimeError(
            "harvester_collision_mode must be 'off' or 'on'; "
            f"received {harvester_collision_mode!r}.")
    docking_camera = docking_camera.strip().lower()
    if docking_camera not in ('true', 'false'):
        raise RuntimeError(
            "docking_camera must be 'true' or 'false'; "
            f"received {docking_camera!r}.")
    articulation_control_mode = articulation_control_mode.strip().lower()
    if articulation_control_mode not in ('kinematic', 'pid'):
        raise RuntimeError(
            "articulation_control_mode must be 'kinematic' or 'pid'; "
            f"received {articulation_control_mode!r}.")

    converted_root = ET.fromstring(conversion.stdout)
    model = converted_root.find('model')
    if model is None:
        raise RuntimeError('Gazebo did not produce a model from the harvester URDF.')

    model.set('name', 'oil_palm_harvester')
    static = model.find('static')
    if static is None:
        static = ET.Element('static')
        model.insert(0, static)
    static.text = 'false'

    # The converter produces model://oil_palm_harvester_description/... URIs.
    # Point them at the installed package directly so Gazebo can resolve every
    # mesh regardless of the user's GAZEBO_MODEL_PATH configuration.
    package_uri = 'model://oil_palm_harvester_description/'
    file_uri = 'file://' + str(harvester_share) + '/'
    for uri in model.findall('.//uri'):
        if uri.text and uri.text.startswith(package_uri):
            uri.text = file_uri + uri.text[len(package_uri):]

    for link in model.findall('link'):
        gravity = link.find('gravity')
        if gravity is None:
            gravity = ET.Element('gravity')
            link.insert(0, gravity)
        gravity.text = 'false'

        if link.get('name') == 'base_link':
            # Keep the mobile base kinematic while leaving the articulated
            # boom/platform links dynamic.  The bridge still moves this root
            # with /harvester/cmd_vel, but slider-driven child joints cannot
            # transfer an ODE reaction impulse into the vehicle base.
            kinematic = link.find('kinematic')
            if kinematic is None:
                kinematic = ET.Element('kinematic')
                link.insert(0, kinematic)
            kinematic.text = 'true'

        if render_mode == 'primitive':
            # If an OGRE/OpenGL combination cannot display binary STL meshes,
            # users can launch with ``render_mode:=primitive``.  This replaces
            # mesh visuals with the collision primitives already describing
            # the body, wheels, boom, platform and cutting arm.  The SDF links,
            # joints, collisions and controller plugin remain exactly the same.
            for visual in link.findall('visual'):
                link.remove(visual)
            for index, collision in enumerate(link.findall('collision')):
                geometry = collision.find('geometry')
                if geometry is None:
                    continue
                fallback_visual = ET.Element(
                    'visual',
                    {'name': f"{collision.get('name', 'collision')}_fallback_{index}"},
                )
                collision_pose = collision.find('pose')
                if collision_pose is not None:
                    fallback_visual.append(deepcopy(collision_pose))
                fallback_visual.append(deepcopy(geometry))
                material = ET.SubElement(fallback_visual, 'material')
                ET.SubElement(material, 'ambient').text = '0.95 0.22 0.02 1'
                ET.SubElement(material, 'diffuse').text = '0.95 0.22 0.02 1'
                ET.SubElement(material, 'emissive').text = '0.10 0.02 0.00 1'
                link.append(fallback_visual)

        if harvester_collision_mode == 'off':
            # The robot is still movable and fully rendered.  Removing only
            # its collision bodies stops ODE from transferring an impulse from
            # a slider-driven arm into the free vehicle base.  The tree model
            # remains static and collidable for range/lidar sensors.
            for collision in link.findall('collision'):
                link.remove(collision)

        if docking_camera == 'false':
            # ``always_on=false`` alone still leaves a rendered Gazebo camera
            # attached to the model once a client subscribes.  Remove just the
            # optional docking sensor from the generated SDF for a true
            # low-resource fallback; its URDF mount/TF remains unchanged.
            for sensor in link.findall("sensor[@name='docking_depth_camera']"):
                link.remove(sensor)

    world_root = ET.Element('sdf', {'version': '1.7'})
    world = ET.SubElement(world_root, 'world', {'name': 'harvester_tree'})
    ET.SubElement(ET.SubElement(world, 'include'), 'uri').text = 'model://sun'
    ET.SubElement(ET.SubElement(world, 'include'), 'uri').text = 'model://ground_plane'

    # Start Gazebo with a view of both the harvester at the origin and the
    # tree at x=8.5.  Without this, Gazebo Classic often restores a previous
    # user-camera pose that shows only the tree, even though the harvester is
    # present in the Model list.
    gui = ET.SubElement(world, 'gui', {'fullscreen': '0'})
    camera = ET.SubElement(gui, 'camera', {'name': 'harvester_tree_view'})
    ET.SubElement(camera, 'pose').text = '-11 -14 8 0 0.32 0.70'
    ET.SubElement(camera, 'view_controller').text = 'orbit'

    # The tree belongs to the environment.  It is static inside its SDF model,
    # has no state publisher or control interface in Gazebo, and stays at x=8.5.
    tree_include = ET.SubElement(world, 'include')
    ET.SubElement(tree_include, 'uri').text = 'model://oil_palm_tree'
    ET.SubElement(tree_include, 'name').text = 'oil_palm_tree'
    ET.SubElement(tree_include, 'pose').text = '8.5 0 0 0 0 0'

    model_pose = ET.Element('pose')
    model_pose.text = '0 0 0.05 0 0 0'
    model.insert(1, model_pose)
    bridge_plugin = ET.SubElement(
        model, 'plugin',
        {'name': 'harvester_kinematic_gazebo_bridge',
         'filename': 'libharvester_kinematic_gazebo_plugin.so'})
    # Kinematic control is the safe default for the current sensor-development
    # model: it applies only changed, rate-limited joint poses at 20 Hz and
    # clears residual physics velocities after each batch.  ``pid`` remains a
    # fresh-launch fallback for regression diagnosis only.
    ET.SubElement(bridge_plugin, 'articulation_control_mode').text = (
        articulation_control_mode)
    world.append(model)

    output_path = Path(tempfile.gettempdir()) / 'oil_palm_harvester_dynamic_scene.world'
    ET.ElementTree(world_root).write(output_path, encoding='utf-8', xml_declaration=True)
    return str(output_path)


def _start_gazebo(context, *, urdf_path, harvester_share, gazebo_share, gui):
    """Generate the selected visual representation after launch args resolve."""
    render_mode = LaunchConfiguration('render_mode').perform(context)
    harvester_collision_mode = LaunchConfiguration(
        'harvester_collision_mode').perform(context)
    docking_camera = LaunchConfiguration('docking_camera').perform(context)
    articulation_control_mode = LaunchConfiguration(
        'articulation_control_mode').perform(context)
    dynamic_world_path = _dynamic_world_from_urdf(
        urdf_path, harvester_share, render_mode, harvester_collision_mode,
        docking_camera, articulation_control_mode)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gazebo_share / 'launch' / 'gazebo.launch.py')),
            launch_arguments={
                'world': dynamic_world_path,
                'gui': gui,
                # Do not leave RViz and the GUI running against a stale or
                # missing Gazebo world.  In particular, a Gazebo master-port
                # conflict must shut down this combined launch immediately.
                'server_required': 'true',
            }.items(),
        )
    ]


def generate_launch_description():
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    camera_lidar_view = LaunchConfiguration('camera_lidar_view')
    camera_lidar_projection = LaunchConfiguration('camera_lidar_projection')
    camera_lidar_calibration_file = LaunchConfiguration('camera_lidar_calibration_file')
    range_calibration = LaunchConfiguration('range_calibration')
    range_calibration_file = LaunchConfiguration('range_calibration_file')
    harvester_share = Path(get_package_share_directory('oil_palm_harvester_description'))
    tree_share = Path(get_package_share_directory('oil_palm_tree_description'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))
    harvester_prefix = Path(get_package_prefix('oil_palm_harvester_description'))

    harvester_rviz_urdf_path = harvester_share / 'urdf' / 'oil_palm_harvester_kinematic.urdf'
    tree_urdf_path = tree_share / 'urdf' / 'oil_palm_tree_lowpoly.urdf'
    publisher_script = harvester_share / 'scripts' / 'publish_urdf.py'
    lidar_stamp_bridge_script = harvester_share / 'scripts' / 'lidar_timestamp_bridge.py'
    camera_view_selector_script = harvester_share / 'scripts' / 'camera_view_selector.py'
    camera_lidar_projection_script = harvester_share / 'scripts' / 'camera_lidar_projection.py'
    range_calibration_script = harvester_share / 'scripts' / 'range_sensor_calibration.py'
    cutter_range_marker_script = harvester_share / 'scripts' / 'cutter_range_marker.py'
    nominal_camera_lidar_calibration_file = (
        harvester_share / 'config' / 'camera_lidar_calibration.nominal.json')
    nominal_range_calibration_file = (
        harvester_share / 'config' / 'range_sensor_calibration.nominal.json')

    tree_model_path = str(tree_share / 'gazebo_model')
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    gazebo_model_path = ':'.join(filter(None, (
        tree_model_path, existing_model_path)))
    existing_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    gazebo_plugin_path = ':'.join(filter(None, (
        str(harvester_prefix / 'lib'), existing_plugin_path)))
    # Gazebo 11's GUI waits synchronously for the online model browser cache.
    # Point it to a valid local database with one placeholder model instead of
    # using an empty URI (which this Gazebo version interprets as '/').
    local_model_database_uri = 'file://' + str(
        harvester_share / 'gazebo_model_database')

    # RViz and Gazebo both use the same kinematic joint structure.  Gazebo gets
    # a native SDF world at startup rather than using spawn_entity.py, which
    # previously stalled while importing this large URDF in Foxy.
    harvester_rviz_urdf = harvester_rviz_urdf_path.read_text()
    tree_urdf = tree_urdf_path.read_text()
    gazebo = OpaqueFunction(
        function=_start_gazebo,
        kwargs={
            'urdf_path': harvester_rviz_urdf_path,
            'harvester_share': harvester_share,
            'gazebo_share': gazebo_share,
            'gui': gui,
        },
    )

    harvester_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': harvester_rviz_urdf}],
        # The Gazebo bridge publishes measured feedback here.  RViz must use
        # this actual state rather than the GUI's immediate command target.
        remappings=[('joint_states', '/harvester/joint_states')],
    )
    harvester_joint_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        # Load directly from disk to avoid the description-topic startup race
        # that made the previous slider values reset.
        arguments=[str(harvester_rviz_urdf_path)],
        # Keep commands separate from the Gazebo-measured joint feedback used
        # by robot_state_publisher above; otherwise RViz leads Gazebo/sensors.
        remappings=[('joint_states', '/harvester/joint_commands')],
    )
    tree_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='tree_state_publisher',
        # Foxy robot_state_publisher republishes its model on a
        # ``robot_description`` topic.  Keep the tree's automatic copy out of
        # the harvester RViz display topic; TF frame names remain unchanged.
        namespace='tree',
        parameters=[{'robot_description': tree_urdf}],
    )

    # Gazebo and RViz share the world frame.  The tree is fixed in that frame;
    # the base controller publishes the movable world -> base_link transform.
    world_to_tree_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_tree_tf',
        arguments=['8.5', '0', '0', '0', '0', '0', 'world', 'tree_base'],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', str(harvester_share / 'rviz' / 'harvester_tree_combined.rviz')],
        condition=IfCondition(rviz),
        output='screen',
    )
    # A dedicated, camera-relative 3-D LiDAR window complements the Image
    # display in the combined RViz window.  Arrange the two windows side by
    # side with the desktop window manager for live image/cloud comparison.
    camera_lidar_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='camera_lidar_rviz',
        arguments=['-d', str(harvester_share / 'rviz' / 'harvester_camera_lidar.rviz')],
        condition=IfCondition(PythonExpression([
            "'", rviz, "' == 'true' and '", camera_lidar_view, "' == 'true'",
        ])),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo client GUI.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz with the combined scene.'),
        DeclareLaunchArgument(
            'camera_lidar_view', default_value='false',
            description=(
                'Start an optional second RViz window with the LiDAR cloud in the '
                'cutting-arm camera frame for side-by-side comparison with '
                'the camera Image display. Disabled by default to conserve '
                'embedded-GPU memory and rendering capacity.')),
        DeclareLaunchArgument(
            'docking_camera', default_value='true',
            description=(
                'Enable the low-rate simulated docking depth camera. Set false '
                'to retain the original single-camera Gazebo resource load; the '
                'RViz selector then safely remains on the cutter camera.')),
        DeclareLaunchArgument(
            'camera_lidar_projection', default_value='false',
            description=(
                'Start the optional, low-rate camera/LiDAR projection node. It '
                'uses raw Gazebo timestamps and publishes an RGB overlay plus '
                'camera-frame points in the existing RViz process. Disabled by '
                'default to preserve Xavier CPU/GPU capacity.')),
        DeclareLaunchArgument(
            'camera_lidar_calibration_file',
            default_value=str(nominal_camera_lidar_calibration_file),
            description=(
                'Simulation-only camera/LiDAR calibration JSON. The deployment '
                'template is intentionally rejected by the projection node.')),
        DeclareLaunchArgument(
            'render_mode', default_value='mesh',
            description=(
                "Gazebo harvester visual representation: 'mesh' (detailed default) "
                "or 'primitive' (driver-independent fallback).")),
        DeclareLaunchArgument(
            'harvester_collision_mode', default_value='off',
            description=(
                "Harvester contact geometry: 'off' (stable kinematic sensor "
                "development default) or 'on' (future physics/controller work).")),
        DeclareLaunchArgument(
            'articulation_control_mode', default_value='kinematic',
            description=(
                "Harvester joint control: 'kinematic' (safe, rate-limited 20 Hz "
                "sensor-development default) or 'pid' (legacy diagnostic fallback).")),
        DeclareLaunchArgument(
            'range_calibration', default_value='true',
            description=(
                'Project the five raw Range streams into c_channel_reference and '
                'publish calibrated docking rays, endpoints, and a side-pair '
                'trunk estimate. Raw sensor topics are never changed.')),
        DeclareLaunchArgument(
            'range_calibration_file', default_value=str(nominal_range_calibration_file),
            description=(
                'Simulation-only range calibration JSON. The deployment template '
                'is deliberately rejected by the calibration projector.')),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', gazebo_plugin_path),
        # Do not let Gazebo Classic block its client on the public online
        # model database.  This world already contains explicit local paths
        # for sun, ground, the static tree and the harvester meshes.
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', local_model_database_uri),
        gazebo,
        harvester_state_publisher,
        harvester_joint_gui,
        tree_state_publisher,
        world_to_tree_tf,
        ExecuteProcess(
            cmd=['python3', str(publisher_script), str(tree_urdf_path), '/tree_description', 'tree_description_publisher'],
            output='screen',
        ),
        # Gazebo stamps sensor messages in simulation time, but the stable
        # Foxy GUI/TF path uses wall time.  Preserve the raw Gazebo stream and
        # present a latest-TF copy to RViz on the public topic.  A zero stamp
        # tells TF2 to use the most recent arm transform rather than requiring
        # an exact historical timestamp through the GUI-controlled chain.
        ExecuteProcess(
            cmd=[
                'python3', str(lidar_stamp_bridge_script),
                '/harvester/lidar/raw_points', '/harvester/lidar/points',
            ],
            output='screen',
        ),
        # This is a header-preserving image/CameraInfo selector for the one
        # existing RViz Image display.  It publishes no TF and changes no
        # Gazebo/control topic; the cutter camera remains the default.
        ExecuteProcess(
            cmd=['python3', str(camera_view_selector_script)],
            output='screen',
        ),
        # This optional perception node derives the camera/LiDAR transform from
        # the same fixed URDF joints used by Gazebo and RViz.  It reads the raw
        # sim-time image/cloud pair; unlike the public LiDAR RViz stream it
        # never requests a latest-TF timestamp and never changes control/TF.
        ExecuteProcess(
            cmd=[
                'python3', str(camera_lidar_projection_script),
                camera_lidar_calibration_file, str(harvester_rviz_urdf_path),
            ],
            condition=IfCondition(camera_lidar_projection),
            output='screen',
        ),
        # This node leaves Gazebo ray geometry and the five raw Range topics
        # untouched.  It uses the rigid URDF relationship between every range
        # sensor and c_channel_reference to publish calibrated local endpoints
        # without an unsafe simulation-time / wall-time TF lookup.
        ExecuteProcess(
            cmd=[
                'python3', str(range_calibration_script),
                range_calibration_file, str(harvester_rviz_urdf_path),
            ],
            condition=IfCondition(range_calibration),
            output='screen',
        ),
        # The cutter sensor crosses movable arm joints, so it gets its own
        # frame-locked RViz marker rather than joining the five fixed docking
        # sensors in range_calibration.py.
        ExecuteProcess(
            cmd=['python3', str(cutter_range_marker_script)],
            output='screen',
        ),
        # RViz requires the dynamic joint TF chain emitted after the joint GUI
        # and Gazebo bridge start.  Starting it immediately races that chain
        # and produces harmless but distracting "frame does not exist"
        # warnings for the boom links.  Eight seconds is deliberately longer
        # than the normal Xavier bridge startup while preserving all model and
        # control behaviour.
        TimerAction(
            period=8.0,
            actions=[rviz_node, camera_lidar_rviz_node],
        ),
    ])
