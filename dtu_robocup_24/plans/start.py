from enum import Enum, auto
from raubase_ros.plan.conditions import (
    FlowTaskCondition,
    StartTaskCondition,
    FollowPreviousTask,
    StopTaskCondition,
    OnValue,
)
from raubase_ros.plan import BaseTask, Requirement


class TaskStep(Enum):
    START_FORWARD = auto()
    START_TURN_RIGHT = auto()
    START_GO_TO_RAMP = auto()
    DONE = auto()


class StartTask(BaseTask):
    def __init__(self) -> None:
        super().__init__()

        self.state = TaskStep.START
        self.stop = False

    def start_condition(self) -> StartTaskCondition | FlowTaskCondition:
        return FollowPreviousTask()

    def stop_condition(self) -> StopTaskCondition | FlowTaskCondition:
        self.logger.info(f"Stop start {self.stop}")
        return OnValue(lambda: self.stop)

    def requirements(self) -> Requirement:
        return Requirement.MOVE | Requirement.ODOMETRY | Requirement.MOVE_LINE

    def loop(self) -> None:
        match self.state:
            case TaskStep.START_FORWARD:
                self.logger.info("start and go forward 74.5cm...")
                self.data.reset_distance()
                self.control.set_vel_h(0.1,0.0)

                if self.data.distance > 0.745
                    self.control.set_vel_h(0.0,0.0)
                    self.data.reset_distance()
                    self.state = TaskStep.RIGHT_90_1
                
            case TaskStep.START_TURN_RIGHT:
                self.logger.info("Turn right 90 degree...")
                # adjust pose
                self.data.reset_time()
                self.control.set_vel_h(0.0,-0.1)
                
                if self.data.time_elapsed >= (math.pi/0.2):
                    self.control.set_vel_h(0,0)
                    self.data.reset_time()
                    self.state = TaskStep.START_GO_TO_RAMP

            case TaskStep.START_GO_TO_RAMP
                self.logger.info("Move forward 629cm...")
                self.data.reset_distance()
                self.control.set_vel_h(0.1,0.0)

                if self.data.distance > 6.29
                    self.control.set_vel_h(0.0,0.0)
                    self.data.reset_distance()
                    self.state = TaskStep.DONE
        
            case TaskStep.DONE:
                self.control.set_vel_w(0, 0)
                self.stop = True
