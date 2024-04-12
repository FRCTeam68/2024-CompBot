// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubSystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.NoteSubSystem;
import frc.robot.subsystems.NoteSubSystem.ActionRequest;
import frc.robot.subsystems.NoteSubSystem.Target;
import frc.robot.subsystems.Vision;
import frc.robot.commands.*;

public class RobotContainer {
  private double MaxSpeed = 5.2; // meters per second desired top speed, see tuner consts kSpeedAt12VoltsMps
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController m_xboxController = new CommandXboxController(0); // drive controller
  public final CommandSwerveDrivetrain m_DriveSubSystem = TunerConstants.DriveTrain;

  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);
  
  CommandPS4Controller m_ps4Controller = new CommandPS4Controller(1);
  
  private final LightsSubsystem m_lightsSubsystem = new LightsSubsystem();
  NoteSubSystem m_NoteSubSystem = new NoteSubSystem();
  ClimberSubSystem m_Climber = new ClimberSubSystem();
  // DigitalInput m_noteSensor1 = new DigitalInput(0);   will be I2C sensor
  DigitalInput m_noteSensor2 = new DigitalInput(0);
  // DigitalInput m_noteSensor3 = new DigitalInput(1);
  // Trigger m_NoteSensorTrigger1 = new Trigger(m_noteSensor1::get);
  Trigger m_NoteSensorTrigger2 = new Trigger(m_noteSensor2::get);
  // Trigger m_NoteSensorTrigger3 = new Trigger(m_noteSensor3::get);

  DigitalInput m_angleZeroLimitSwitch = new DigitalInput(2);
  Trigger m_angleZeroLimitSwitchTrigger = new Trigger(m_angleZeroLimitSwitch::get);

  private boolean m_climbActive = false;
  
  // Dashboard inputs
  private double m_autoWaitTimeSelected = 0;
  private final SendableChooser<String> m_autoWaitTimeChooser = new SendableChooser<>();

  private final LoggedDashboardChooser<Command> autoChooser;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
  
    // Register Named Commands
    // NamedCommands.registerCommand("shoot_spinup", Commands.runOnce(()->SmartDashboard.putBoolean("shoot", true))
    //                                                     .andThen(()->m_NoteSubSystem.setTarget(Target.SPEAKER))
    //                                                     .andThen(()->m_NoteSubSystem.setAction(ActionRequest.SHOOT_SPINUP))    );

    // NamedCommands.registerCommand("shoot", Commands.runOnce(()->SmartDashboard.putBoolean("shoot", true))
    //                                                     .andThen(()->m_NoteSubSystem.setTarget(Target.SPEAKER))
    //                                                     .andThen(()->m_NoteSubSystem.setAction(ActionRequest.SHOOT))    );

    // NamedCommands.registerCommand("intake2", Commands.runOnce(()->SmartDashboard.putBoolean("intake", true))
    //                                                     .andThen(()->m_NoteSubSystem.setTarget(Target.INTAKE))
    //                                                     .andThen(()->m_NoteSubSystem.setAction(ActionRequest.INTAKENOTE))    );

    // NamedCommands.registerCommand("shoot2", Commands.runOnce(()->SmartDashboard.putBoolean("shoot2", true))
    //                                                     .andThen(()->m_NoteSubSystem.setTarget(Target.SPEAKER_1M))
    //                                                     .andThen(()->m_NoteSubSystem.setAction(ActionRequest.SHOOT))   );

    
    NamedCommands.registerCommand("shoot_spinup", Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.SHOOT_SPINUP))    );
    NamedCommands.registerCommand("target_speaker", new SetTargetCustomCmd(m_NoteSubSystem, Constants.ANGLE.SPEAKER, Constants.SHOOTER.SPEAKER_SHOOT_SPEED));
    NamedCommands.registerCommand("shoot", Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.SHOOT)));

    NamedCommands.registerCommand("target_intake", Commands.runOnce(()->m_NoteSubSystem.setTarget(Target.INTAKE)));
    NamedCommands.registerCommand("intake", Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.INTAKENOTE))    );

    NamedCommands.registerCommand("target_speaker_1m", new SetTargetCustomCmd(m_NoteSubSystem, Constants.ANGLE.SPEAKER_1M, Constants.SHOOTER.SPEAKER_SHOOT_SPEED));
    NamedCommands.registerCommand("target_speaker_podium", new SetTargetCustomCmd(m_NoteSubSystem, Constants.ANGLE.SPEAKER_PODIUM, Constants.SHOOTER.SPEAKER_SHOOT_SPEED));
    NamedCommands.registerCommand("target_speaker_podium_source", new SetTargetCustomCmd(m_NoteSubSystem, Constants.ANGLE.SPEAKER_PODIUM_SOURCE, Constants.SHOOTER.SPEAKER_SHOOT_SPEED));

    NamedCommands.registerCommand("DelayStart", new WaitCommand(m_autoWaitTimeSelected));

     m_DriveSubSystem.setCurrentLimits();

    configureBindings();

    // setupPidTuningCommandShuffleboard();

    
    

    // Put subsystems to dashboard.
    Shuffleboard.getTab("NoteSubsystem").add(m_NoteSubSystem);
    // Shuffleboard.getTab("Drivetrain").add(m_DriveSubSystem);
    Shuffleboard.getTab("ClimberSubSystem").add(m_Climber);
    SmartDashboard.putBoolean("shoot", false);
    SmartDashboard.putBoolean("shoot2", false);
    SmartDashboard.putBoolean("intake", false);
    SmartDashboard.putBoolean("wait2s", false);
    //SmartDashboard.putBoolean("NoteSensor1", false);
    SmartDashboard.putBoolean("NoteSensor2", false);
    SmartDashboard.putBoolean("NoteSensor3", false);
    SmartDashboard.putBoolean("AngleLimitLowSwitch", false);
    SmartDashboard.putBoolean("ClimberPitMode", false);

    m_autoWaitTimeChooser.setDefaultOption("none", "0");
    m_autoWaitTimeChooser.addOption("one", "1");
    m_autoWaitTimeChooser.addOption("two", "2");
    SmartDashboard.putData("Auto DelayStart (s)", m_autoWaitTimeChooser);

    System.out.println("note sensor switch: " + m_noteSensor2.get());
    System.out.println("low limit switch: " + m_angleZeroLimitSwitch.get());
    

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  }
 
  /**
   * Use this method to define your button->command mappings.
   */
  private void configureBindings() {
    System.out.println("config bindings");
    
    m_DriveSubSystem.setDefaultCommand( // Drivetrain will execute this command periodically
        m_DriveSubSystem.drive(() -> drive.withVelocityX(-m_xboxController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-m_xboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_xboxController.getRightX() * MaxAngularRate)
           // Drive counterclockwise with negative X (left)
        , m_xboxController).ignoringDisable(true));

    // m_xboxController.a().whileTrue(m_DriveSubSystem.applyRequest(() -> brake));
    // m_xboxController.b().whileTrue(m_DriveSubSystem
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-m_xboxController.getLeftY(), -m_xboxController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    m_xboxController.back().onTrue(m_DriveSubSystem.runOnce(() -> m_DriveSubSystem.seedFieldRelative()));

    if (Utils.isSimulation()) {
      m_DriveSubSystem.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    m_DriveSubSystem.registerTelemetry(logger::telemeterize);

    // m_xboxController.pov(0).whileTrue(m_DriveSubSystem.applyRequest(() -> forwardStraight.withVelocityX(0.2 * MaxSpeed).withVelocityY(0)));
    // m_xboxController.pov(180).whileTrue(m_DriveSubSystem.applyRequest(() -> forwardStraight.withVelocityX(-0.2 * MaxSpeed).withVelocityY(0)));
    // m_xboxController.pov(90).whileTrue(m_DriveSubSystem.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(-0.2 * MaxSpeed)));
    // m_xboxController.pov(270).whileTrue(m_DriveSubSystem.applyRequest(() -> forwardStraight.withVelocityX(0).withVelocityY(0.2 * MaxSpeed)));
    // m_xboxController.povDown().onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.STOP)));  replaced with coastdown + LOWSPEED

    m_xboxController.y().onTrue(Commands.runOnce(()->m_NoteSubSystem.setTarget(Target.SPEAKER_PODIUM)));
    m_xboxController.b().onTrue(Commands.runOnce(()->m_NoteSubSystem.setTarget(Target.AMP)));
    m_xboxController.x().onTrue(Commands.runOnce(()->m_NoteSubSystem.setTarget(Target.TRAP)));
    m_xboxController.a().onTrue(Commands.runOnce(()->m_NoteSubSystem.setTarget(Target.SPEAKER)));
    //for trial
    // m_xboxController.a().onTrue(new SetTargetCustomCmd(m_NoteSubSystem, Constants.ANGLE.SPEAKER, Constants.SHOOTER.SPEAKER_SHOOT_SPEED));

    m_xboxController.leftTrigger().onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.INTAKENOTE)));
    m_xboxController.rightTrigger().onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.SHOOT)));
    m_xboxController.leftBumper().onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.SPIT_NOTE2)));
    m_xboxController.rightBumper().onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.SHOOT_SPINUP)));
    m_xboxController.start().onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.STOP_ALL)));

    m_ps4Controller.triangle().onTrue(Commands.runOnce(()->m_NoteSubSystem.setTarget(Target.SPEAKER_PODIUM)));
    m_ps4Controller.circle().onTrue(Commands.runOnce(()->m_NoteSubSystem.setTarget(Target.AMP)));
    m_ps4Controller.square().onTrue(Commands.runOnce(()->m_NoteSubSystem.setTarget(Target.TRAP)));
    m_ps4Controller.cross().onTrue(Commands.runOnce(()->m_NoteSubSystem.setTarget(Target.SPEAKER)));

    //m_ps4Controller.L1().onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.FEEDSTATION_SPIN)));
    m_ps4Controller.L2().onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.DISLODGE_WITH_SHOOTER)));
    m_ps4Controller.R1().onTrue(Commands.runOnce(()->m_NoteSubSystem.setPassSpeed(Constants.SHOOTER.PASS1_SPEED)));
    m_ps4Controller.R2().onTrue(Commands.runOnce(()->m_NoteSubSystem.setPassSpeed(Constants.SHOOTER.PASS2_SPEED)));

    // m_ps4Controller.L2().onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.INTAKENOTE)));
    // m_ps4Controller.R2().onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.SHOOT)));
    // m_ps4Controller.L1().onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.SPIT_NOTE2)));
    // m_ps4Controller.R1().onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.SHOOT_SPINUP)));
    // m_ps4Controller.touchpad().onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.STOP)));

    m_ps4Controller.options().onTrue(Commands.runOnce(()->m_NoteSubSystem.setHaveNote1(false)));

    m_ps4Controller.povLeft().onTrue(Commands.runOnce(()->m_NoteSubSystem.bumpShooterSpeed((-Constants.SHOOTER.BUMP_VALUE))));
    m_ps4Controller.povRight().onTrue(Commands.runOnce(()->m_NoteSubSystem.bumpShooterSpeed((Constants.SHOOTER.BUMP_VALUE))));
    m_ps4Controller.povUp().onTrue(Commands.runOnce(()->m_NoteSubSystem.bumpAnglePosition((-Constants.ANGLE.BUMP_VALUE))));
    m_ps4Controller.povDown().onTrue(Commands.runOnce(()->m_NoteSubSystem.bumpAnglePosition((Constants.ANGLE.BUMP_VALUE))));

    // //Left Joystick Y
    // m_ps4Controller.axisGreaterThan(1,0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake1Speed((-Constants.INTAKE.BUMP_VALUE))));
    // m_ps4Controller.axisLessThan(1,-0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake1Speed((Constants.INTAKE.BUMP_VALUE))));
    // //Right Joystick Y
    // m_ps4Controller.axisGreaterThan(5,0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake2Speed((-Constants.INTAKE.BUMP_VALUE))));
    // m_ps4Controller.axisLessThan(5,-0.7).whileTrue(Commands.run(()->m_NoteSubSystem.bumpIntake2Speed((Constants.INTAKE.BUMP_VALUE))));

    m_ps4Controller.share().onTrue(Commands.runOnce(()->m_NoteSubSystem.resetSetpoints()));

    m_ps4Controller.PS().onTrue(Commands.runOnce(()->m_climbActive=!m_climbActive)
                                              .andThen(()->m_Climber.setPitMode(m_climbActive))
                                              .andThen(()->SmartDashboard.putBoolean("ClimberPitMode", m_climbActive)));

    m_Climber.setDefaultCommand(Commands.run( ()->m_Climber.setSpeedVout(m_ps4Controller.getLeftY() * 12, 
                                                                          -m_ps4Controller.getRightY() * 12), m_Climber));

    // m_NoteSensorTrigger1.onTrue(Commands.runOnce(()->SmartDashboard.putBoolean("NoteSensor1", true)))
    //                    .onFalse(Commands.runOnce(()->SmartDashboard.putBoolean("NoteSensor1", false)));
    m_NoteSensorTrigger2.onTrue(Commands.runOnce(()->SmartDashboard.putBoolean("NoteSensor2", true)))
                       .onFalse(Commands.runOnce(()->SmartDashboard.putBoolean("NoteSensor2", false)));
    // m_NoteSensorTrigger3.onTrue(Commands.runOnce(()->m_NoteSubSystem.setAction(ActionRequest.BEAM3))
    //                                     .andThen(()->SmartDashboard.putBoolean("NoteSensor3", true)))
    //                    .onFalse(Commands.runOnce(()->SmartDashboard.putBoolean("NoteSensor3", false)));
    // m_NoteSensorTrigger3.onTrue(Commands.runOnce(()->SmartDashboard.putBoolean("NoteSensor3", true)))
    //                    .onFalse(Commands.runOnce(()->SmartDashboard.putBoolean("NoteSensor3", false)));


    // m_angleZeroLimitSwitchTrigger.onTrue(Commands.runOnce(()->m_NoteSubSystem.zeroAngleSubsystem())
    //                                             .andThen(()->SmartDashboard.putBoolean("AngleLimitLowSwitch", true)))
    //                             .onFalse(Commands.runOnce(()->SmartDashboard.putBoolean("AngleLimitLowSwitch", false)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    m_autoWaitTimeSelected = Double.parseDouble(m_autoWaitTimeChooser.getSelected());
    System.out.println("Auto wait time selected: " + m_autoWaitTimeSelected);

    return autoChooser.get();
  }

  @SuppressWarnings("unused")
  private void setupPidTuningCommandShuffleboard(){
    // First, assign a local variable the Tab that we are going to use
    // for pid tuning in Shuffleboard
    Shuffleboard.getTab("PID Tuning").add(new ShooterPIDTuning(m_NoteSubSystem));
  }

  public void StopSubSystems(){
    m_NoteSubSystem.setAction(ActionRequest.STOP_ALL);
  }
}
