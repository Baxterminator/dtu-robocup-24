from enum import Enum, auto
from raubase_ros.plan.conditions import (
    FlowTaskCondition,
    StartTaskCondition,
    FollowPreviousTask,
    StopTaskCondition,
    OnValue,
)
from raubase_ros.plan import BaseTask, close_to
import numpy as np

from raubase_ros.plan.data import Requirement


class TaskStep(Enum):
    SEESAW_TURN_LEFT = auto()
    SEESAW_BEFORE_RAMP = auto()
    SEESAW_RAMP = auto()
    FALL_ONTO = auto()
    BOARD_FORWARD = auto()
    SEESAW_TURN_RIGHT = auto()
    SEESAW_TO_RAMP = auto()
    DONE = auto()


class SeeSawTask(BaseTask):
    def __init__(self) -> None:
        super().__init__()
        self.state = TaskStep.FALL_ONTO
        self.stop = False
        self.stop_cond = OnValue(lambda: self.stop)

    def start_condition(self) -> StartTaskCondition | FlowTaskCondition:
        return FollowPreviousTask()

    def stop_condition(self) -> StopTaskCondition | FlowTaskCondition:
        return self.stop_cond

    def requirements(self) -> Requirement:
        return Requirement.MOVE | Requirement.ODOMETRY

    def loop(self) -> None:

        match self.state:
            case TaskStep.SEESAW_TURN_LEFT
                self.logger.info("Turn left 90 degree...")
                # adjust pose
                self.data.reset_time()
                self.control.set_vel_h(0.0,0.1)
                if self.data.time_elapsed >= (math.pi/0.2):
                    self.control.set_vel_h(0,0)
                    self.data.reset_time()
                    self.state = TaskStep.SEESAW_BEFORE_RAMP
                    
            case TaskStep.SEESAW_BEFORE_RAMP:
                self.logger.info("Move forward 87cm...")
                self.data.reset_distance()
                self.control.set_vel_h(0.1,0.0)

                if self.data.distance > 0.87
                    self.data.reset_distance()
                    self.state = TaskStep.SEESAW_RAMP
        
            case TaskStep.SEESAW_RAMP:
                self.logger.info("Climbing ramp 310cm...")
                self.data.reset_distance()
                self.control.set_vel_h(0.1,0.0)

                if self.data.distance > 3.1
                    self.data.reset_distance()
                    self.state = TaskStep.SEESAW_RAMP
        
            case TaskStep.FALL_ONTO:
                self.logger.info("Going on the see-saw ...")
                self.control.set_vel_h(0, np.pi / 2)

                if close_to(self.data.odometry.heading, np.pi / 2):
                    self.data.reset_distance()
                    self.state = TaskStep.BOARD_FORWARD

            case TaskStep.BOARD_FORWARD:
                self.logger.info("Going forward 317cm...")
                self.control.set_vel_w(0.1, 0)

                if self.data.distance >= 3.17:
                    self.state = TaskStep.SEESAW_TO_LINE
        
            case TaskStep.SEESAW_TURN_RIGHT:
                self.logger.info("Turn right 90 degree...")
                # adjust pose
                self.data.reset_time()
                self.control.set_vel_h(0.0,-0.1)
                if self.data.time_elapsed >= (math.pi/0.2):
                    self.control.set_vel_h(0,0)
                    self.data.reset_time()
                    self.state = TaskStep.SEESAW_TO_RAMP
        
            case TaskStep.SEESAW_TO_RAMP:
                self.logger.info("Going forward 165cm...")
                self.control.set_vel_w(0.1, 0)
                if self.data.distance >= 1.65:
                    self.state = TaskStep.DONE
        
            case TaskStep.DONE:
                self.logger.info("Exiting see-saw and find the ramp")
                self.control.set_vel_w(0, 0)
                self.stop = True
