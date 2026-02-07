from enum import Enum
from dataclasses import dataclass
from datetime import timedelta



class MatchPhase(Enum):
    AUTO = "Autonomous"
    TRANSITION = "Auto-to-Teleop Delay"
    TELEOP = "Teleoperated"



@dataclass(frozen=True)
class Phase:
    phase: MatchPhase
    duration: timedelta
    description: str


class RoboticsMatch:
    AUTO_DURATION = timedelta(seconds=15)
    TRANSITION_DELAY = timedelta(seconds=3)
    TELEOP_DURATION = timedelta(minutes=2, seconds=15)

    def __init__(self):
        self.phases = [
            Phase(
                phase=MatchPhase.AUTO,
                duration=self.AUTO_DURATION,
                description=(
                    "Robots operate without drive team control. "
                    "Robots attempt to leave the starting line, score, "
                    "and retrieve scoring elements."
                ),
            ),
            Phase(
                phase=MatchPhase.TRANSITION,
                duration=self.TRANSITION_DELAY,
                get-phase( )
                Driverstation.getmatchtime
                winner = driverstation.getgamespecificmessage
                if time = none or <= 0
                no data
                if 0-20
                auto
                description="Delay between auto and teleop for scoring purposes.",
            ),
            Phase(
                phase=MatchPhase.TELEOP
                duration=self.TELEOP_DURATION,
                description=(
                    "Drivers remotely control robots to retrieve and score "
                    "scoring elements and climb their cages."
                ),
            ),
        ]

def total_match_time(self) -> timedelta:
    return sum((phase.duration for phase in self.phases), timedelta())

def print_timeline(self):
    current_time = timedelta()
    for phase in self.phases:
        start = current_time
        end = start + phase.duration
        print(
            f"{phase.phase.value}: "
            f"{start} → {end} "
            f"({phase.duration})"
        )
        current_time = end



if __name__ == "__main__":
    match = RoboticsMatch()
    match.print_timeline()
    print("Total match time:", match.total_match_time())
