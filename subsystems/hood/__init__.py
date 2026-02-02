"""
Hood subsystem
"""
from math import atan, sqrt
from typing import Callable

from pathplannerlib.auto import FlippingUtil
from pykit.logger import Logger
from wpilib import Alert, DriverStation
from wpimath.filter import Debouncer
from wpimath.geometry import Pose2d, Rotation2d, Pose3d

from constants import Constants
from subsystems import Subsystem
from subsystems.hood.io import HoodIO

# pylint: disable=too-many-instance-attributes
class HoodSubsystem(Subsystem):
    """subsystem for hood"""

    def __init__(self, io: HoodIO, robot_pose_supplier: Callable[[], Pose2d]):
        super().__init__()

        self.io = io
        self.alliance = DriverStation.getAlliance()

        self.robot_pose_supplier = robot_pose_supplier

        self.inputs = HoodIO.HoodIOInputs()
        self.hood_disconnected_alert = Alert("Hood motor is disconnected.", Alert.AlertType.kError)

        self.at_set_point_debounce = Debouncer(0.1, Debouncer.DebounceType.kRising)

        self.hub_pose = Pose2d # blue hub
        self.launch_speed =  10.03 # meters per second
        self.distance = 1
        self.angle = atan(       # calculated angle using launch speed, distance and starting height
            (self.launch_speed ** 2 +
             sqrt(
                 self.launch_speed ** 4 -
                 9.80665 *
                 (9.80665 * self.distance ** 2 +
                  3 * Constants.FieldConstants.HUB_HEIGHT * self.launch_speed ** 2)))
             / (9.80665 * self.distance))


    def periodic(self):
        """runs stuff periodically (every 20 ms)"""
        self.io.update_inputs(self.inputs)
        Logger.processInputs("Hood", self.inputs)

        self.hub_pose = Constants.FieldConstants.HUB_POSE
        self.distance = (self.robot_pose_supplier()
                         .translation().distance(self.hub_pose.translation()))

        self.io.set_position(Rotation2d.fromDegrees(self.angle)) # convert degrees to rotations,

        self.hood_disconnected_alert.set(not self.inputs.hood_connected)

        if self.alliance != DriverStation.getAlliance():
            self.hub_pose = FlippingUtil.flipFieldPose(self.hub_pose)
            self.alliance = DriverStation.getAlliance()

    def get_component_pose(self) -> Pose3d:
        """for advantage scope modelling (placeholder)"""
