/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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

    public static final class Pathfind {
        public static final double mVelMS = 2
        ;
        public static final double mAccMS = 2;
        public static final double mRVelRS = Math.PI/2;
        public static final double mRAccRS = Math.PI;
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
        public static final double PASS2_SPEED = 40;
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
        public static final double SPEAKER_PODIUM = 26; //24; //16 //22 //24
        public static final double INTAKE = 17; // 12;   //14
        public static final double BUMP_VALUE = .5;    //rotations
        public static final double ATANGLE_TIMEOUT = 1;  //seconds
        public static final double SPEAKER_PODIUM_SOURCE = 16;
        
    }

    public static final class CLIMBER {
        public static final int LEFT_CANID = 40;
        public static final int RIGHT_CANID = 41;
        public static final String CANBUS = "DRIVEbus";
        public static final double MAX_HEIGHT = 115;   //100 rotates is about 9in
    }

    public static final class Vision {
        public static final String camBlName = "Arducam_OV9281_USB_Camerabl"; //TODO: change names!
        public static final String camBrName = "Arducam_OV9281_USB_Camerabr";

        public static final String aprilTagLayoutPath = ""; // Not needed to be used unless custom

        // Front camera constants... Faces past intake
        public static final Transform3d frontCameraLocation = new Transform3d(new Translation3d(Units.inchesToMeters(9.5), 0, Units.inchesToMeters(9.5)), new Rotation3d(0, 0, 0)); // TODO: Fix these values

        // Back Camera Constants... On back left Motor
        public static final Transform3d backrCameraLocation = 
        new Transform3d(new Translation3d(Units.inchesToMeters(-8.25),
         Units.inchesToMeters(-11.2), Units.inchesToMeters(11.85)), 
         new Rotation3d(0, Units.degreesToRadians(28.125), Units.degreesToRadians(210))); // TODO: Fix these values
    
        public static final Transform3d backlCameraLocation = 
        new Transform3d(new Translation3d(Units.inchesToMeters(-8.25), 
        Units.inchesToMeters(11.2), Units.inchesToMeters(11.85)), 
        new Rotation3d(0, Units.degreesToRadians(28.125), Units.degreesToRadians(150))); // TODO: Fix these values

        // Field Constants...P
        public static final int tallThingFiscal = 8;
        public static final double tallThingHeight = Units.inchesToMeters(57);

         /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
        public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
        public static final double MAX_DISTANCE = 4;

        /**
         * Standard deviations of model states. Increase these numbers to trust your
         *`` model's state estimates less. This
         * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
         * meters.
         */

         public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final class RED_TAGS {
        public static final int[] stage = {11,12,13};
    }

    public static final class BLUE_TAGS {
        public static final int[] stage = {14,15,16};
        public static final Pose2d goal = new Pose2d(2.01, 6.00, new Rotation2d(Units.degreesToRadians(-178.57)));
    }

    public static final class LightsConstants {
        public static final int CANDLE_PORT = 60;
    }
}
