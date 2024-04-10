package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    
    SwerveDrivePoseEstimator poseEstimator;
    public Pose2d lastEstimate = new Pose2d();

    HolonomicDriveController trajCont  = new HolonomicDriveController(new PIDController(1,0,0), 
        new PIDController(10, 0, 0), new ProfiledPIDController(2.5, 0, 0, new TrapezoidProfile.Constraints(Math.PI*2, Math.PI)));

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withSteerRequestType(SteerRequestType.MotionMagic);

    // public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
    //     super(driveTrainConstants, OdometryUpdateFrequency, modules);
    //     configurePathPlanner();
    //     if (Utils.isSimulation()) {
    //         startSimThread();
    //     }
    // }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, m_pigeon2.getRotation2d(), m_modulePositions, new Pose2d());
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

    }

    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        System.out.println("X: %s|  Y: %s".formatted(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond));
        this.setControl(autoRequest.withSpeeds(targetSpeeds));
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Pathfind.mVelMS);
        int i =0;
        for (SwerveModule mod : this.Modules) {
            i++;
            mod.apply(desiredStates[i], SwerveModule.DriveRequestType.Velocity);
        }
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        // AutoBuilder.configureHolonomic(
        //     ()->this.getState().Pose, // Supplier of current robot pose
        //     this::seedFieldRelative,  // Consumer for seeding pose against auto
        //     this::getCurrentRobotChassisSpeeds,
        //     (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
        //     new HolonomicPathFollowerConfig(new PIDConstants(1, 0, 0),
        //                                     new PIDConstants(1, 0, 0),
        //                                     TunerConstants.kSpeedAt12VoltsMps,
        //                                     driveBaseRadius,
        //                                     new ReplanningConfig()),
        //     () -> {
        //             // Boolean supplier that controls when the path will be mirrored for the red alliance
        //             // This will flip the path being followed to the red side of the field.
        //             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        //             var alliance = DriverStation.getAlliance();
        //             if (alliance.isPresent()) {
        //                 return alliance.get() == DriverStation.Alliance.Red;
        //             }
        //             return false;
        //         },
        //     this); // Subsystem for requirements

        
        AutoBuilder.configureHolonomic(
            this::getEstimatedPose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            ()->m_kinematics.toChassisSpeeds(this.m_moduleStates),
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
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

    public void setPose (Pose2d pose) {
        poseEstimator.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, pose);
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
                */
        // Seems to drive fine enough.
        ChassisSpeeds speeds = ChassisSpeeds.discretize(request.VelocityX, request.VelocityY, request.RotationalRate, 0.02);

        this.setControl(request.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond));
    
    }

    public Command angleDrive(int tagID, Supplier<Double> xSpeed, Supplier<Double> ySpeed) {
        return run(() -> angle(tagID, xSpeed.get(), ySpeed.get()));
    }

    public void angle(int tagID, double xSpeed, double ySpeed) {
        Optional<Pose3d> tagPose  = Constants.Vision.aprillayout.getTagPose(tagID);
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, 0);

        if (tagPose.isPresent()){
            Pose2d tag = tagPose.get().toPose2d();
            Pose2d robotPose = getEstimatedPose();
            double dx = tag.getX()-robotPose.getX();
            double dy = tag.getY()-robotPose.getY();
            double hyp = Math.hypot(dx, dy);
            double ySign = (Math.abs(dy)/dy)-1;
            double  theta = -(Math.asin(dx/hyp) + (ySign*Math.PI/8) + (Math.PI/2));
            Pose2d referencePose = new Pose2d(getEstimatedPose().getTranslation(), new Rotation2d(theta));
            speeds.omegaRadiansPerSecond = trajCont.calculate(getEstimatedPose(), referencePose, 0, new Rotation2d(theta)).omegaRadiansPerSecond;
            if (Math.abs(speeds.omegaRadiansPerSecond) < 0.1) {
                speeds.omegaRadiansPerSecond = 0;
            }
            System.out.println("Deg: "+ theta*180/Math.PI + " | OMEGA: " + speeds.omegaRadiansPerSecond);
            this.setControl(this.autoRequest.withSpeeds(speeds));
            return;
        }
        
        this.setControl(this.autoRequest.withSpeeds(speeds));
    }

    public void updatePoseEstimator() {
        poseEstimator.update(this.m_pigeon2.getRotation2d(), m_modulePositions);
    }
    
    public void addBrVisionMeasurement(Pose2d pose, EstimatedRobotPose estPose){
        if (pose == null)
            pose = new Pose2d();
        poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp()); 
        // THEN I SHALL BEGIN AGAIN, WITH MY WORD
        //                AS LAW
    }

    public void addBlVisionMeasurement(Pose2d pose, EstimatedRobotPose estPose){
        if (pose == null)
            pose = new Pose2d();
        poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp()); 
        // THEN I SHALL BEGIN AGAIN, WITH MY WORD
        //                AS LAW
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public SwerveDrivePoseEstimator getOdometry(){ 
        return this.m_odometry;
    }
    public void setCurrentLimits(){
        this.getModule(0).getDriveMotor().getConfigurator().refresh(TunerConstants.driveConfig);
        this.getModule(1).getDriveMotor().getConfigurator().refresh(TunerConstants.driveConfig);
        this.getModule(2).getDriveMotor().getConfigurator().refresh(TunerConstants.driveConfig);
        this.getModule(3).getDriveMotor().getConfigurator().refresh(TunerConstants.driveConfig);

        this.getModule(0).getSteerMotor().getConfigurator().refresh(TunerConstants.steerConfig);
        this.getModule(1).getSteerMotor().getConfigurator().refresh(TunerConstants.steerConfig);
        this.getModule(2).getSteerMotor().getConfigurator().refresh(TunerConstants.steerConfig);
        this.getModule(3).getSteerMotor().getConfigurator().refresh(TunerConstants.steerConfig);
    
        
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