package frc.robot;

import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private final SwerveRequest.FieldCentricFacingAngle aimBotRequestion = new SwerveRequest.FieldCentricFacingAngle().withSteerRequestType(SteerRequestType.MotionMagic).withDeadband(0.1);

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
    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
        
        AutoBuilder.configureHolonomic(
            this::getEstimatedPose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            ()->m_kinematics.toChassisSpeeds(this.m_moduleStates),
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

    public void setPose (Pose2d pose) {
        poseEstimator.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, pose);
        m_odometry.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, pose);
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

    public Command aimTheBot(Rotation2d angle, double xSpeed, double ySpeed) {
        return run(() -> this.setControl(this.aimBotRequestion.withTargetDirection(angle).withVelocityX(xSpeed).withVelocityY(ySpeed)));
    }

    public void actuallyDrive(SwerveRequest.FieldCentric request,CommandXboxController xboxController) {         
        // poseEstimator.update(this.m_pigeon2.getRotation2d(), m_modulePositions);
        // vision.updateblCam();
        // poseEstimator.addVisionMeasurement(vision.estimatePoseBack(), this.m_lastSimTime);
        // vision.updatebrCam();
        // poseEstimator.addVisionMeasurement(vision.estimatePoseBack(), Timer.getFPGATimestamp());

        // Seems to drive fine enough.
        ChassisSpeeds speeds = ChassisSpeeds.discretize(request.VelocityX, request.VelocityY, request.RotationalRate, 0.02);

        this.setControl(request.withVelocityX(speeds.vxMetersPerSecond).withVelocityY(speeds.vyMetersPerSecond).withRotationalRate(speeds.omegaRadiansPerSecond));
    
    }

    public void updatePoseEstimator() {
        poseEstimator.update(this.m_pigeon2.getRotation2d(), m_modulePositions);
        this.m_odometry.update(this.m_pigeon2.getRotation2d(), m_modulePositions);
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
