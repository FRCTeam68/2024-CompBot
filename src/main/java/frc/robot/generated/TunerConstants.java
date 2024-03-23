package frc.robot.generated;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.CommandSwerveDrivetrain;

public class TunerConstants {


    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public static final Slot0Configs steerGains = new Slot0Configs()
        //.withKP(100).withKI(0).withKD(0.2)  //swerve wizard
        //.withKS(0).withKV(1.5).withKA(0);   //swerve wizard
        .withKP(40).withKI(0).withKD(0)   //16,0,0
        .withKS(0).withKV(0).withKA(0);

    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs driveGains = new Slot0Configs()
        //.withKP(3).withKI(0).withKD(0)      //swerve wizard
        .withKP(28).withKI(0).withKD(0)    //38,0,0  //4,0,0
        .withKS(0).withKV(0).withKA(0);


        //Monkey PID stuff
        public static final PIDController steerPID = new PIDController(steerGains.kP, steerGains.kI, steerGains.kD);
        public static final PIDController drivePID = new PIDController(driveGains.kP, driveGains.kI, driveGains.kD);
        
    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 80.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    // public static final double kSpeedAt12VoltsMps = 4.81;   //swerve wizard
    public static final double kSpeedAt12VoltsMps = 5.2;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 0;

    private static final double kDriveGearRatio = 5.88; 
    private static final double kSteerGearRatio = 1;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = false; //false
    private static final boolean kInvertLeftSide = false;  //true; //false
    private static final boolean kInvertRightSide = true;  //false; //true

    private static final String kCANbusName = "DRIVEbus";
    private static final int kPigeonId = 50;


    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


// Was changing XPOS and YPOS still unsure about if it is correct. Still not driving right. Not really Consitant with 
// what direction it goes with the joy sticks. Slip current was changed to 80 becuase it wasn't rotating and 20 made it worse, it was a 40 before.
// There is a chance 40 will still work for slip current.

    // Front Left
    private static final int kFrontLeftDriveMotorId = 1;
    private static final int kFrontLeftSteerMotorId = 2;
    private static final int kFrontLeftEncoderId = 15;
    private static final double kFrontLeftEncoderOffset = -0.15673828125;

    private static final double kFrontLeftXPosInches = 9.75;
    private static final double kFrontLeftYPosInches = 11.25;

    // Front Right
    private static final int kFrontRightDriveMotorId = 3;
    private static final int kFrontRightSteerMotorId = 4;
    private static final int kFrontRightEncoderId = 16;
    private static final double kFrontRightEncoderOffset = -0.143798828125;

    private static final double kFrontRightXPosInches = 9.75;
    private static final double kFrontRightYPosInches = -11.25;

    // Back Left
    private static final int kBackLeftDriveMotorId = 5;
    private static final int kBackLeftSteerMotorId = 6;
    private static final int kBackLeftEncoderId = 17;
    private static final double kBackLeftEncoderOffset = -0.433837890625;

    private static final double kBackLeftXPosInches = -9.75;
    private static final double kBackLeftYPosInches = 11.25;

    // Back Right
    private static final int kBackRightDriveMotorId = 7;
    private static final int kBackRightSteerMotorId = 8;
    private static final int kBackRightEncoderId = 18;
    private static final double kBackRightEncoderOffset = 0.04833984375; //-0.049072265625;

    private static final double kBackRightXPosInches = -9.75;
    private static final double kBackRightYPosInches = -11.25;


    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    public static final CommandSwerveDrivetrain DriveTrain = new CommandSwerveDrivetrain(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);

public static CurrentLimitsConfigs driveLimit = new CurrentLimitsConfigs().withStatorCurrentLimit(80);
public static CurrentLimitsConfigs steerLimit = new CurrentLimitsConfigs().withStatorCurrentLimit(60);
public static TalonFXConfiguration driveConfig = new TalonFXConfiguration().withCurrentLimits(driveLimit);
public static TalonFXConfiguration steerConfig = new TalonFXConfiguration().withCurrentLimits(steerLimit);
}
