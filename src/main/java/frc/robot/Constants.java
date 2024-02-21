/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    private static RobotType robotType = RobotType.DEVBOT;

    public enum RobotType {
        CHASSISBOT,
        DEVBOT,
        COMPBOT
    }

    public static RobotType getRobot() {
    return robotType;
  }

    public static final class LED {
        public static final int PWMPORT = 0;
        public static final int BUFFERSIZE = 120;
    }
    
    public static final class ROLLER {
        public static final double MAX_SPEED = 100;
        public static final double MAX_VOLTAGE = 12;
    }

    public static final class INTAKE {
        public static final int CANID = 20;
        public static final String CANBUS = "DRIVEbus";
        public static final double TAKE_NOTE_SPEED = 40;
        public static final double SPIT_NOTE_SPEED = 30;
        public static final double BUMP_VALUE = 0.2;  // 50 counts / second = 10rps
    }

    public static final class FEEDER1 {
        public static final int CANID = 35;
        public static final String CANBUS = "rio";
        public static final double TAKE_NOTE_SPEED = 20;
        public static final double SPIT_NOTE_SPEED = 40;
    }

        public static final class FEEDER2 {
        public static final int CANID = 36;
        public static final String CANBUS = "rio";
        public static final double TAKE_NOTE_SPEED = 20;
        public static final double SHOOT_SPEED = 20;
    }

    public static final class SHOOTER {
        public static final int LEFT_CANID = 30;
        public static final int RIGHT_CANID = 31;
        public static final String CANBUS = "rio";
        public static final double MAX_SPEED = 100;  // rps
        public static final double SHOOT_SPEED = 80;
        public static final double RIGHT_OFFSET = 0;
        public static final double BUMP_VALUE = 1;   // rps
        public static final double SPINUP_TIME = 2;  // seconds
        public static final double STOP_TIME = 2; 
        public static final double ATSPEED_TIMEOUT = 1;  //seconds
    }

    public static final class ANGLE {
        public static final int LEFT_CANID = 32;
        public static final int RIGHT_CANID = 33;
        public static final String CANBUS = "rio";
        //positive value makes shooter side go up
        //negative value makes shooter side go down
        public static final double MIN_POSITION = 0;   //in intake position, -12 will put steepest angel
        public static final double MAX_POSITION = 24;   //motor rotatiions, in intake position, 12 is a low we need to shoot
        public static final double AMP = 0; // -6;
        public static final double TRAP = 0; //-4;
        public static final double SPEAKER = 0; //-2;
        public static final double INTAKE = 0;
        public static final double BUMP_VALUE = .5;    //rotations
        public static final double ATANGLE_TIMEOUT = 1;  //seconds
    }

    public static final class CLIMBER {
        public static final int LEFT_CANID = 40;
        public static final int RIGHT_CANID = 41;
        public static final String CANBUS = "DRIVEbus";
        public static final double MAX_HEIGHT = 9;   //inches
    }
}
