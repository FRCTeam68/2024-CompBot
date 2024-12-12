package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
// import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    private double lastTurnCommandSeconds;
    private boolean keepHeadingSetpointSet;

    private final PIDController keepHeadingPid;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        // configNeutralMode(NeutralModeValue.Coast);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        keepHeadingPid = new PIDController(.1, 0, 0);
        keepHeadingPid.enableContinuousInput(-180, 180);
    }
    
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        // configNeutralMode(NeutralModeValue.Coast);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        keepHeadingPid = new PIDController(.1, 0, 0);
        keepHeadingPid.enableContinuousInput(-180, 180);
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0.1, .05),
                                            new PIDConstants(5, 0, 0),
                                            TunerConstants.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
            this); // Subsystem for requirements
    }

    public Command drive(Supplier<SwerveRequest.RobotCentric> requestSupplier, CommandXboxController xboxController) { 
        return run(() -> this.actuallyDrive(requestSupplier.get(), xboxController));
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));

    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    // simple proportional turning control with Limelight.
    // "proportional control" is a control algorithm in which the output is proportional to the error.
    // in this case, we are going to return an angular velocity that is proportional to the 
    // "tx" value from the Limelight.
    double limelight_aim_proportional(double tx)
    {    
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = 0.015;  //.035;

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
        // your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = tx * kP;

        // convert to radians per second for our drive method
        targetingAngularVelocity *= Constants.Swerve.MAX_ANGULAR_VELOCITY;

        //invert since tx is positive when the target is to the right of the crosshair
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }

    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
    double limelight_range_drive(double stopAngle)
    {    
        // double kP = .05;
        double targetAngle = LimelightHelpers.getTY("limelight-shoot");
        double targetDriveSpeed = 0;

        if(targetAngle<0){
            //crosshair is above target, giving a negative TY, so invert it so robot will drive forward
            targetDriveSpeed = -1.0;
        }
        else if(targetAngle < stopAngle)  //30 is for AMP target.  17 for speaker
        {
            // targetingForwardSpeed *= Constants.Swerve.MAX_SPEED;
            targetDriveSpeed = -1.0;
        }
        return targetDriveSpeed;
    }


    public void actuallyDrive(SwerveRequest.RobotCentric request,CommandXboxController xboxController) {
     
        /*
        if (xboxController.y().getAsBoolean()) // When Y is pressed Hopefully you will lock onto Fiscal Target 8.
        {
           
            PhotonTrackedTarget wantedTarget = vision.getFiscalIDTarget(15, vision.getCurrentTargets());
            if (wantedTarget != null) {
                System.out.println("Robot Angle: " + vision.getYawToTarget(wantedTarget));
                double angleSpeed = Math.toRadians(vision.aimWithYawAtTarget(wantedTarget))*0.1;
                double ForwardSpeed = vision.driveDistFromTarget(wantedTarget, Constants.Vision.tallThingHeight, 0, 4);
                System.out.println("Started Visioning AngleSpeed %s | Forward Speed %s | Aiming at %s".formatted(angleSpeed, ForwardSpeed, wantedTarget.getFiducialId()));
                
                this.setControl(Robot.m_robotContainer.drive.withRotationalRate(1));
                return;
            }

        }
        */

        double rotationRadPerSec;
        // double yawDeg;

        // rotationRadPerSec = request.RotationalRate;
        // yawDeg = getPigeon2().getYaw().getValue();
        
        // Logger.recordOutput("Swerve/RotRateInput", rotationRadPerSec);
        // Logger.recordOutput("Swerve/yawRad", yawDeg);

        // if (rotationRadPerSec != 0) {
        //     lastTurnCommandSeconds = Timer.getFPGATimestamp();
        //     keepHeadingSetpointSet = false;
        // }
        // if (lastTurnCommandSeconds + .5 <= Timer.getFPGATimestamp()
        //         && !keepHeadingSetpointSet) { // If it has been at least .5 seconds.
        //     keepHeadingPid.setSetpoint(yawDeg);
        //     keepHeadingSetpointSet = true;
        // }
        
        // Logger.recordOutput("Swerve/KeepHeading", keepHeadingSetpointSet);
        // if (keepHeadingSetpointSet) {
        //     rotationRadPerSec = Rotation2d.fromDegrees(keepHeadingPid.calculate(yawDeg)).getRadians();
        //     request.withRotationalRate(rotationRadPerSec);
        // }
        // Logger.recordOutput("Swerve/RotRateCmd", rotationRadPerSec);

        var xSpeed = request.VelocityX;
        var ySpeed = request.VelocityY;
        rotationRadPerSec = request.RotationalRate;
        var llAim = xboxController.x().getAsBoolean();
        var targetCount = LimelightHelpers.getTargetCount("limelight-shoot");
        double stopThreshold = 0;

            // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
        if(llAim && targetCount>0)
        {
            int i;
            RawFiducial fid[] = LimelightHelpers.getRawFiducials("limelight-shoot");

            for (i = 0; i < fid.length; i++) {
                if (fid[i].id == 5){    //amp
                    stopThreshold = 30;
                    break;
                }
                else if (fid[i].id == 4){    //speaker
                    stopThreshold = 17;
                    break;
                }

            }
            if (stopThreshold != 0){
                final var rot_limelight = limelight_aim_proportional(fid[i].txnc);
                rotationRadPerSec = rot_limelight;

                final var forward_limelight = limelight_range_drive(stopThreshold);
                xSpeed = forward_limelight;
            }
        }
        
        Logger.recordOutput("Swerve/stopThreshold", stopThreshold);   
        Logger.recordOutput("Swerve/LLaim", llAim);   
        Logger.recordOutput("Swerve/LLaim", llAim);   
        Logger.recordOutput("Swerve/LLrot", rotationRadPerSec);
        Logger.recordOutput("Swerve/LLXspeed", xSpeed);
        Logger.recordOutput("Swerve/Yspeed", ySpeed);

        // Test driving with this in and see how it feels.
        ChassisSpeeds speeds = ChassisSpeeds.discretize(xSpeed, ySpeed, rotationRadPerSec, 0.02);

        this.setControl(request.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void setCurrentLimits(){
        this.getModule(0).getDriveMotor().getConfigurator().apply(TunerConstants.driveLimit);
        this.getModule(1).getDriveMotor().getConfigurator().apply(TunerConstants.driveLimit);
        this.getModule(2).getDriveMotor().getConfigurator().apply(TunerConstants.driveLimit);
        this.getModule(3).getDriveMotor().getConfigurator().apply(TunerConstants.driveLimit);

        this.getModule(0).getSteerMotor().getConfigurator().apply(TunerConstants.steerLimit);
        this.getModule(1).getSteerMotor().getConfigurator().apply(TunerConstants.steerLimit);
        this.getModule(2).getSteerMotor().getConfigurator().apply(TunerConstants.steerLimit);
        this.getModule(3).getSteerMotor().getConfigurator().apply(TunerConstants.steerLimit);
    
        
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void periodic() {
        /* Periodically try to apply the operator perspective */
        /* If we haven't applied the operator perspective before, then we should apply it regardless of DS state */
        /* This allows us to correct the perspective in case the robot code restarts mid-match */
        /* Otherwise, only check and apply the operator perspective if the DS is disabled */
        /* This ensures driving behavior doesn't change until an explicit disable event occurs during testing*/
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                                : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }
}
