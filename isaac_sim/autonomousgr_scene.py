#!/usr/bin/env python3
"""Isaac Sim 6.0.1 standalone scene for the AutonomousGR ROS 2 stack.

Run this file with Isaac Sim's python.sh, not a system Python interpreter.
It creates a Leatherback Ackermann vehicle, a ZED 2i-like stereo pair,
registered-left depth, a starter cone corridor, ROS 2 publishers, and the
/drive Ackermann subscriber/controller graph.
"""

import argparse
import os
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--livestream", action="store_true")
    parser.add_argument("--test-steps", type=int, default=0)
    parser.add_argument("--resolution-width", type=int, default=1280)
    parser.add_argument("--resolution-height", type=int, default=720)
    parser.add_argument("--camera-rate", type=float, default=30.0)
    parser.add_argument(
        "--obstacle-scenario",
        choices=("clear", "center", "right", "narrow", "blocked"),
        default="center",
    )
    parser.add_argument("--ros-domain-id", type=int, default=None)
    parser.add_argument(
        "--experience",
        default="/isaac-sim/apps/isaacsim.exp.full.streaming.kit",
        help="Streaming Kit experience used with --livestream.",
    )
    return parser.parse_known_args()[0]


ARGS = parse_args()
if ARGS.ros_domain_id is not None:
    os.environ["ROS_DOMAIN_ID"] = str(ARGS.ros_domain_id)

from isaacsim.simulation_app import SimulationApp


APP_CONFIG = {
    "headless": ARGS.headless or ARGS.livestream,
    "width": ARGS.resolution_width,
    "height": ARGS.resolution_height,
    "renderer": "RaytracedLighting",
}
EXPERIENCE = ARGS.experience if ARGS.livestream else ""
if EXPERIENCE:
    SIMULATION_APP = SimulationApp(APP_CONFIG, experience=EXPERIENCE)
else:
    SIMULATION_APP = SimulationApp(APP_CONFIG)

# Isaac/Omniverse imports must occur after SimulationApp is created.
import omni.graph.core as og
import omni.kit.app
import omni.usd
from isaacsim.core.experimental.utils import app as app_utils
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.rtx import RtxCamera
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics


ROBOT_PRIM = "/World/Leatherback"
LEFT_CAMERA = f"{ROBOT_PRIM}/Sensors/ZED2i/left_camera"
RIGHT_CAMERA = f"{ROBOT_PRIM}/Sensors/ZED2i/right_camera"
OPTICAL_FRAME = "zed2i_left_camera_optical_frame"


def enable_extensions():
    manager = omni.kit.app.get_app().get_extension_manager()
    for extension in (
        "isaacsim.core.nodes",
        "isaacsim.ros2.bridge",
        "isaacsim.ros2.nodes",
        "isaacsim.robot.wheeled_robots.nodes",
        "isaacsim.sensors.experimental.rtx",
    ):
        manager.set_extension_enabled_immediate(extension, True)
    for _ in range(5):
        SIMULATION_APP.update()


def create_camera(stage, path, translation):
    camera = UsdGeom.Camera.Define(stage, path)
    camera.CreateFocalLengthAttr(2.8)
    camera.CreateHorizontalApertureAttr(5.12)
    camera.CreateVerticalApertureAttr(2.88)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.1, 50.0))

    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    # USD cameras look down local -Z with +Y up. This points them along
    # vehicle +X with vehicle +Z up.
    xform.AddOrientOp().Set(Gf.Quatf(0.5, 0.5, -0.5, -0.5))
    RtxCamera(path=path, tick_rate=ARGS.camera_rate)


def create_cone(stage, name, x, y, color):
    cone = UsdGeom.Cone.Define(stage, f"/World/Track/{name}")
    cone.CreateAxisAttr("Z")
    cone.CreateHeightAttr(0.35)
    cone.CreateRadiusAttr(0.12)
    cone.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    xform = UsdGeom.Xformable(cone.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.175))
    UsdPhysics.CollisionAPI.Apply(cone.GetPrim())


def create_box_obstacle(stage, name, x, y, length, width, height, color):
    box = UsdGeom.Cube.Define(stage, f"/World/Obstacles/{name}")
    box.CreateSizeAttr(1.0)
    box.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    xform = UsdGeom.Xformable(box.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(x, y, height * 0.5))
    xform.AddScaleOp().Set(Gf.Vec3f(length, width, height))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())


def create_obstacle_scenario(stage):
    UsdGeom.Xform.Define(stage, "/World/Obstacles")
    if ARGS.obstacle_scenario == "clear":
        return
    if ARGS.obstacle_scenario == "center":
        create_box_obstacle(stage, "center", 5.0, 0.0, 0.45, 0.25, 0.55, (0.8, 0.1, 0.1))
    elif ARGS.obstacle_scenario == "right":
        create_box_obstacle(stage, "right", 5.0, -0.25, 0.45, 0.35, 0.55, (0.8, 0.1, 0.1))
    elif ARGS.obstacle_scenario == "narrow":
        create_box_obstacle(stage, "narrow_left", 5.0, 0.50, 0.45, 0.30, 0.55, (0.8, 0.1, 0.1))
        create_box_obstacle(stage, "narrow_right", 5.0, -0.50, 0.45, 0.30, 0.55, (0.8, 0.1, 0.1))
    elif ARGS.obstacle_scenario == "blocked":
        create_box_obstacle(stage, "blocked", 5.0, 0.0, 0.45, 1.35, 0.55, (0.8, 0.1, 0.1))


def create_scene():
    stage_utils.set_stage_up_axis("Z")
    stage_utils.set_stage_units(meters_per_unit=1.0)
    stage = omni.usd.get_context().get_stage()

    assets_root = get_assets_root_path()
    if not assets_root:
        raise RuntimeError("Isaac Sim asset root is unavailable")

    stage_utils.add_reference_to_stage(
        usd_path=assets_root + "/Isaac/Environments/Grid/default_environment.usd",
        path="/World/Ground",
    )
    stage_utils.add_reference_to_stage(
        usd_path=assets_root + "/Isaac/Robots/NVIDIA/Leatherback/leatherback.usd",
        path=ROBOT_PRIM,
    )

    light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    light.CreateIntensityAttr(700.0)

    UsdGeom.Xform.Define(stage, f"{ROBOT_PRIM}/Sensors")
    UsdGeom.Xform.Define(stage, f"{ROBOT_PRIM}/Sensors/ZED2i")
    create_camera(stage, LEFT_CAMERA, (0.20, 0.06, 0.20))
    create_camera(stage, RIGHT_CAMERA, (0.20, -0.06, 0.20))

    UsdGeom.Xform.Define(stage, "/World/Track")
    center_offsets = (0.0, 0.05, 0.12, 0.20, 0.28, 0.35, 0.40)
    for index, offset in enumerate(center_offsets):
        x = 2.0 + index * 1.5
        create_cone(stage, f"yellow_{index}", x, 0.75 + offset, (1.0, 0.8, 0.0))
        create_cone(stage, f"blue_{index}", x, -0.75 + offset, (0.0, 0.2, 1.0))
    create_cone(stage, "orange_start_left", 1.2, 0.35, (1.0, 0.25, 0.0))
    create_cone(stage, "orange_start_right", 1.2, -0.35, (1.0, 0.25, 0.0))
    create_obstacle_scenario(stage)


def create_ros_graph():
    graph_path = "/World/AutonomousGR_ROS"
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(graph_path):
        stage.RemovePrim(graph_path)

    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("LeftRunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("LeftRender", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("LeftRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("LeftDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("RightRender", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RightRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("SubscribeDrive", "isaacsim.ros2.bridge.ROS2SubscribeAckermannDrive"),
                ("Ackermann", "isaacsim.robot.wheeled_robots.AckermannController"),
                ("SteeringController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("WheelController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:useDomainIDEnvVar", True),
                ("PublishClock.inputs:topicName", "/clock"),
                ("LeftRender.inputs:cameraPrim", [Sdf.Path(LEFT_CAMERA)]),
                ("LeftRender.inputs:width", ARGS.resolution_width),
                ("LeftRender.inputs:height", ARGS.resolution_height),
                ("LeftRgb.inputs:type", "rgb"),
                ("LeftRgb.inputs:topicName", "/sim/camera/image_raw"),
                ("LeftRgb.inputs:frameId", OPTICAL_FRAME),
                ("LeftRgb.inputs:useSystemTime", False),
                ("LeftDepth.inputs:type", "depth"),
                ("LeftDepth.inputs:topicName", "/sim/camera/depth"),
                ("LeftDepth.inputs:frameId", OPTICAL_FRAME),
                ("LeftDepth.inputs:useSystemTime", False),
                ("RightRender.inputs:cameraPrim", [Sdf.Path(RIGHT_CAMERA)]),
                ("RightRender.inputs:width", ARGS.resolution_width),
                ("RightRender.inputs:height", ARGS.resolution_height),
                ("RightRgb.inputs:type", "rgb"),
                ("RightRgb.inputs:topicName", "/sim/camera/right/image_raw"),
                ("RightRgb.inputs:frameId", "zed2i_right_camera_optical_frame"),
                ("RightRgb.inputs:useSystemTime", False),
                ("CameraInfo.inputs:topicName", "/sim/camera/camera_info"),
                ("CameraInfo.inputs:topicNameRight", "/sim/camera/right/camera_info"),
                ("CameraInfo.inputs:frameId", OPTICAL_FRAME),
                ("CameraInfo.inputs:frameIdRight", "zed2i_right_camera_optical_frame"),
                ("CameraInfo.inputs:useSystemTime", False),
                ("SubscribeDrive.inputs:topicName", "/drive"),
                ("Ackermann.inputs:frontWheelRadius", 0.052),
                ("Ackermann.inputs:backWheelRadius", 0.052),
                ("Ackermann.inputs:wheelBase", 0.32),
                ("Ackermann.inputs:trackWidth", 0.24),
                ("Ackermann.inputs:maxWheelRotation", 0.7854),
                ("Ackermann.inputs:maxWheelVelocity", 20.0),
                ("Ackermann.inputs:maxAcceleration", 1.0),
                ("Ackermann.inputs:maxSteeringAngleVelocity", 1.0),
                ("SteeringController.inputs:robotPath", ROBOT_PRIM),
                (
                    "SteeringController.inputs:jointNames",
                    ["Knuckle__Upright__Front_Left", "Knuckle__Upright__Front_Right"],
                ),
                ("WheelController.inputs:robotPath", ROBOT_PRIM),
                (
                    "WheelController.inputs:jointNames",
                    [
                        "Wheel__Knuckle__Front_Left",
                        "Wheel__Knuckle__Front_Right",
                        "Wheel__Upright__Rear_Left",
                        "Wheel__Upright__Rear_Right",
                    ],
                ),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("Tick.outputs:tick", "LeftRunOnce.inputs:execIn"),
                ("LeftRunOnce.outputs:step", "LeftRender.inputs:execIn"),
                ("LeftRender.outputs:execOut", "RightRender.inputs:execIn"),
                ("RightRender.outputs:execOut", "LeftRgb.inputs:execIn"),
                ("RightRender.outputs:execOut", "LeftDepth.inputs:execIn"),
                ("LeftRender.outputs:renderProductPath", "LeftRgb.inputs:renderProductPath"),
                ("LeftRender.outputs:renderProductPath", "LeftDepth.inputs:renderProductPath"),
                ("Context.outputs:context", "LeftRgb.inputs:context"),
                ("Context.outputs:context", "LeftDepth.inputs:context"),
                ("RightRender.outputs:execOut", "RightRgb.inputs:execIn"),
                ("RightRender.outputs:renderProductPath", "RightRgb.inputs:renderProductPath"),
                ("Context.outputs:context", "RightRgb.inputs:context"),
                ("RightRender.outputs:execOut", "CameraInfo.inputs:execIn"),
                ("LeftRender.outputs:renderProductPath", "CameraInfo.inputs:renderProductPath"),
                (
                    "RightRender.outputs:renderProductPath",
                    "CameraInfo.inputs:renderProductPathRight",
                ),
                ("Context.outputs:context", "CameraInfo.inputs:context"),
                ("Tick.outputs:tick", "SubscribeDrive.inputs:execIn"),
                ("Context.outputs:context", "SubscribeDrive.inputs:context"),
                ("SubscribeDrive.outputs:execOut", "Ackermann.inputs:execIn"),
                ("SubscribeDrive.outputs:speed", "Ackermann.inputs:speed"),
                (
                    "SubscribeDrive.outputs:steeringAngle",
                    "Ackermann.inputs:steeringAngle",
                ),
                (
                    "SubscribeDrive.outputs:steeringAngleVelocity",
                    "Ackermann.inputs:steeringAngleVelocity",
                ),
                ("SubscribeDrive.outputs:acceleration", "Ackermann.inputs:acceleration"),
                ("Ackermann.outputs:execOut", "SteeringController.inputs:execIn"),
                ("Ackermann.outputs:wheelAngles", "SteeringController.inputs:positionCommand"),
                ("Ackermann.outputs:execOut", "WheelController.inputs:execIn"),
                (
                    "Ackermann.outputs:wheelRotationVelocity",
                    "WheelController.inputs:velocityCommand",
                ),
            ],
        },
    )


def main():
    enable_extensions()
    create_scene()
    create_ros_graph()

    # Graphs must exist before initializing the simulation context in Isaac Sim 6.
    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    app_utils.play()
    for _ in range(10):
        SIMULATION_APP.update()

    print("AUTONOMOUSGR_ISAAC_READY", flush=True)
    steps = 0
    try:
        while SIMULATION_APP.is_running():
            SIMULATION_APP.update()
            steps += 1
            if ARGS.test_steps > 0 and steps >= ARGS.test_steps:
                break
    except KeyboardInterrupt:
        pass
    finally:
        app_utils.stop()
        SIMULATION_APP.close()


if __name__ == "__main__":
    main()
