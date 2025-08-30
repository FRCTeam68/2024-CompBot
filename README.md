This is the 2024 season code.
Swerve drive is from CTRE SwerveWizzard.
Pigeon2 and Canivore with Pro license.
Krakens on the drive motors.  Falcon on the steer.
Falcons all onther manipulator motors.

NoteSubsystem uses Angle, Shooter, Intake, Feeder1 and Feeder2 subsystems.

Using PathPlanner for Auton paths with named commands.

Vision is PhotonVision with OrangePI .

LEDS are Candle.



Mechanical Changes Made:
--Swerve--
    drive motors changed from Kraken X60's to Falcon 500's
    drive ratio changed (check robot for drive pinion)

--Angle--
    motors changed from Kraken X60's to Falcon 500's
    gear ratio in planetary gearbox changed (I think it is 5:1, but not sure and I do not believe it was stickered. May need to check with physical tests)

--Climber--
    removed

--CAN--
    canivore removed (all devices on rio bus)

Fuctions for Open House:
    drive
    intake
    spit note
    shoot (low and high power)
        avoid hitting the ceiling
        shut down after every shot. No need to make extra noise when people can wait 2 seconds for spin up.
    stop all
    zero chassis rotation
    toggle for chassis speeds (slow down for novice drivers)