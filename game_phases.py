from wpilib import DriverStation
def get_phase():
    # Fetch data from Driverstation
    # Note: getMatchTime() returns a float (seconds remaining)
    time = DriverStation.getMatchTime()

    # Check if the match has actually started or if the data is valid
    if time is None or time < 0:
        return "PRE_MATCH"

    # FRC Match Timing Logic
    # 15s to 0s during Autonomous
    if Driverstation
    phase=MatchPhase.TRANSITION,
    duration=self.TRANSITION_DELAY,
           get-phase( ) -> string:
    time = Driverstation.getmatchtime
    winner = driverstation.getgamespecificmessage
             if time = none or <= 0
             data
             if 0-20
             auto
