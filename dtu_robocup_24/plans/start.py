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
    START = auto()
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
            case TaskStep.START:
                self.logger.info("start ...")
                self.control.follow_line(True, 0.03, 0.4)

                if self.data.distance >= 8:
                    self.state = TaskStep.DONE

            case TaskStep.DONE:
                self.control.set_vel_w(0, 0)
                self.stop = True
