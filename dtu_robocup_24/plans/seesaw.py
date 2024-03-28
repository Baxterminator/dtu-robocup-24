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
    FALL_ONTO = auto()
    BOARD_FORWARD = auto()
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
            case TaskStep.FALL_ONTO:
                self.logger.info("Going on the see-saw ...")
                self.control.set_vel_h(0, np.pi / 2)

                if close_to(self.data.odometry.heading, np.pi / 2):
                    self.data.reset_distance()
                    self.state = TaskStep.BOARD_FORWARD

            case TaskStep.BOARD_FORWARD:
                self.logger.info("Going forward ...")
                self.control.set_vel_w(0.15, 0)

                if self.data.distance >= 2.45:
                    self.state = TaskStep.DONE
            case TaskStep.DONE:
                self.logger.info("Exiting see-saw")
                self.control.set_vel_w(0, 0)
                self.done = True
