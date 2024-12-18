/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.Function;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

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

    public static CurrentLimitsConfigs limit80 = new CurrentLimitsConfigs().withStatorCurrentLimit(80);
    public static CurrentLimitsConfigs limit60 = new CurrentLimitsConfigs().withStatorCurrentLimit(60);

    public enum RobotType {
        CHASSISBOT,
        DEVBOT,
        COMPBOT
    }

    public static RobotType getRobot() {
        return robotType;
    }

    public static double STICK_DEADBAND = 0.08;

    // public static final Function<Double, Double> inputCurve = (x) -> x;
    public static final Function<Double, Double> inputCurve = (x) -> .8 * x + .2 * (x * x * x);

    public static final class Swerve {
        /* DriveSubsystem Profiling Values */
        /** Meters per Second */
        public static final double MAX_SPEED = 5.2; // TODO: This must be tuned to specific robot
        /**
         * Radians per Second
         *
         * <p>Can likely be figured out using an equation. Or we can just tornado spin it and see
         * what happens.
         *
         * <p>public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
         * MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
         * DRIVETRAIN_WHEELBASE_METERS / 2.0);
         *
         * <p>Assuming our robot can still go at 4.6 meters per second (which it can't, this value
         * was taken when we had like nothing on our robot, we can go
         * 10.348178456877813130498318828226534894488969421353632604171 radians per second while
         * spinning
         */
        //  5.2 m/s  / sqrt((9.75in + 11.25in)),  5.2 / sqrt(0.5334m) = 7.12 rad/s = 387 deg/s
        // what do realword measuremest say ???
        public static final double MAX_ANGULAR_VELOCITY = 7.12; // TODO: This must be tuned to specific robot
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
        public static final double TAKE_NOTE_SPEED = 20;
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
        public static final double SPEAKER_SHOOT_SPEED = 80;
        public static final double TRAP_SHOOT_SPEED = 43; 
        public static final double AMP_SHOOT_SPEED = 23;   //20 layup // 26 dunk VelFOC.
        public static final double AMP_RIGHT_OFFSET = 15;  //14 layup // 16 dunk VelFOC.  //0 to tune shooter pid - 26 ,top does not roll at 26
        public static final double RIGHT_OFFSET = 0;
        public static final double BUMP_VALUE = 1;   // rps
        public static final double SPINUP_TIME = 2;  // seconds
        public static final double STOP_TIME = 2; 
        public static final double ATSPEED_TIMEOUT = 1;  //seconds
        public static final double DISLODGE_SHOOT_SPEED = -60;
        public static final double PASS1_SPEED = 60;
        public static final double PASS2_SPEED = 60; //40;
        public static final double SPINLOW_SPEED = 10;
    }

    public static final class ANGLE {
        public static final int LEFT_CANID = 32;
        public static final int RIGHT_CANID = 33;
        public static final String CANBUS = "rio";

        //increasing value makes shooter side go down
        //decreasing value makes shooter side go up
        // as of Feb 22nd we cannot go steeper than speaker position of 0
        // so AMP and TRAP are also 0
        //motor rotations
        public static final double MIN_POSITION = 0;  
        public static final double MAX_POSITION = 50;
        public static final double AMP = 2;
        public static final double TRAP = 0;   //distance 1.57M
        public static final double SPEAKER = 0;
        public static final double SPEAKER_1M = 21; //18 ; //16 //15 //20
        public static final double SPEAKER_PODIUM = 24; //26; //24; //16 //22 //24
        public static final double INTAKE = 17; // 12;   //14
        public static final double BUMP_VALUE = .5;    //rotations
        public static final double ATANGLE_TIMEOUT = 1;  //seconds
        public static final double SPEAKER_PODIUM_SOURCE = 16;
        public static final double SPEAKER_STAGE = 35; // 36; //34;
        
    }

    public static final class CLIMBER {
        public static final int LEFT_CANID = 40;
        public static final int RIGHT_CANID = 41;
        public static final String CANBUS = "DRIVEbus";
        public static final double MAX_HEIGHT = 132; //122;  // old hooks: 115;   //100 rotates is about 9in
    }

    public static final class Vision {
        public static final String camLName = "Arducam_OV9782_1"; //TODO: change names!
        public static final String camRName = "Arducam_OV9782_2";

        public static final String aprilTagLayoutPath = ""; // Not needed to be used unless custom

        // Front camera constants... Faces past intake
        public static final Transform3d frontCameraLocation = new Transform3d(new Translation3d(Units.inchesToMeters(9.5), 0, Units.inchesToMeters(9.5)), new Rotation3d(0, 0, 0)); // TODO: Fix these values

        // Back Camera Constants... Faces through shooter
        public static final Transform3d backCameraLocation = new Transform3d(new Translation3d(Units.inchesToMeters(9.5), 0, Units.inchesToMeters(14.5)), new Rotation3d(0, Units.degreesToRadians(5), 0)); // TODO: Fix these values
    
        // Field Constants...P
        public static final int tallThingFiscal = 8;
        public static final double tallThingHeight = Units.inchesToMeters(57);
    }

    public static final class RED_TAGS {
        public static final int[] stage = {11,12,13};
    }

    public static final class BLUE_TAGS {
        public static final int[] stage = {14,15,16};
    }

    public static final class LightsConstants {
        public static final int CANDLE_PORT = 60;
    }
}
