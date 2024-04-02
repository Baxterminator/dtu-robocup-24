from enum import Enum, auto
from raubase_ros.plan.conditions import (
    FlowTaskCondition,
    StartTaskCondition,
    FollowPreviousTask,
    StopTaskCondition,
    OnValue,
)
from raubase_ros.plan import BaseTask, Requirement, close_to
import numpy as np

class TaskStep(Enum):
    START_FORWARD = auto()
    START_TURN_RIGHT = auto()
    START_GO_TO_RAMP = auto()
    DONE = auto()


class StartTask(BaseTask):
    
    SPEED = 0.2

    def __init__(self) -> None:
        super().__init__()

        self.state = TaskStep.START_FORWARD
        self.stop = False

    def start_condition(self) -> StartTaskCondition | FlowTaskCondition:
        return FollowPreviousTask()

    def stop_condition(self) -> StopTaskCondition | FlowTaskCondition:
        return OnValue(lambda: self.stop)

    def requirements(self) -> Requirement:
        return Requirement.MOVE | Requirement.ODOMETRY | Requirement.MOVE_LINE

    def loop(self) -> None:
        match self.state:
            case TaskStep.START_FORWARD:
                self.logger.info("start and go forward 74.5cm...",
                throttle_duration_sec=0.5,)
                self.control.set_vel_w(StartTask.SPEED,0.0)

                if self.data.distance > 0.70:
                    self.control.set_vel_w(0.0,0.0)
                    self.data.reset_time()
                    self.state = TaskStep.START_TURN_RIGHT
                
            case TaskStep.START_TURN_RIGHT:
                self.logger.info("Turn right 90 degree...",
                throttle_duration_sec=0.5,)
                # adjust pose
                self.control.set_vel_h(0, -1.65806)

                if close_to(self.data.odometry.heading, -1.65806):
                    self.data.reset_distance()
                    self.state = TaskStep.START_GO_TO_RAMP

            case TaskStep.START_GO_TO_RAMP:
                self.logger.info("Move forward 629cm...",
                throttle_duration_sec=0.5,)
                self.control.set_vel_w(StartTask.SPEED,0.0)

                if self.data.distance > 4.88:
                    self.control.set_vel_w(0.0,0.0)
                    self.data.reset_distance()
                    self.state = TaskStep.DONE
        
            case TaskStep.DONE:
                self.control.set_vel_w(0, 0)
                self.stop = True
