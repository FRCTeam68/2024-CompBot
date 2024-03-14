package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;


    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    // public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
    //     super(driveTrainConstants, OdometryUpdateFrequency, modules);
    //     configurePathPlanner();
    //     if (Utils.isSimulation()) {
    //         startSimThread();
    //     }
    // }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

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
            new HolonomicPathFollowerConfig(new PIDConstants(1, 0, 0),
                                            new PIDConstants(1, 0, 0),
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

    public Command drive(Supplier<SwerveRequest.FieldCentric> requestSupplier, CommandXboxController xboxController) { 
        return run(() -> this.actuallyDrive(requestSupplier.get(), xboxController));
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));

    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public void actuallyDrive(SwerveRequest.FieldCentric request,CommandXboxController xboxController) {
        Vision vision = Robot.m_robotContainer.m_Vision;
        double stageD = vision.distanceToStage();
        
        SmartDashboard.putNumber("TrapDistance", stageD);
        Logger.recordOutput("Vision/Distance",  stageD);
        if (stageD > 1.5 && stageD < 1.7) {
            Logger.recordOutput("Vision/TrapGood",  true);
            SmartDashboard.putBoolean("TrapGood", true);
        }
        else{           
            Logger.recordOutput("Vision/TrapGood",  false);
            SmartDashboard.putBoolean("TrapGood", false);
        }
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

        // Test driving with this in and see how it feels.
        ChassisSpeeds speeds = ChassisSpeeds.discretize(request.VelocityX, request.VelocityY, request.RotationalRate, 0.02);

        this.setControl(request.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void setCurrentLimits(){
        this.getModule(0).getDriveMotor().getConfigurator().refresh(TunerConstants.driveConfig);
        this.getModule(2).getDriveMotor().getConfigurator().refresh(TunerConstants.driveConfig);
        this.getModule(4).getDriveMotor().getConfigurator().refresh(TunerConstants.driveConfig);
        this.getModule(6).getDriveMotor().getConfigurator().refresh(TunerConstants.driveConfig);

        this.getModule(1).getSteerMotor().getConfigurator().refresh(TunerConstants.steerConfig);
        this.getModule(3).getSteerMotor().getConfigurator().refresh(TunerConstants.steerConfig);
        this.getModule(5).getSteerMotor().getConfigurator().refresh(TunerConstants.steerConfig);
        this.getModule(7).getSteerMotor().getConfigurator().refresh(TunerConstants.steerConfig);
    
        
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
}
