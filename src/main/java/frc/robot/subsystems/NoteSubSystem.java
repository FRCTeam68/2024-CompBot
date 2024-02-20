package frc.robot.subsystems;


import java.util.Optional;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.RollerSubSystem;
import frc.robot.subsystems.AngleSubSystem;
import frc.robot.subsystems.ShooterSubSystem;
// import frc.robot.subsystems.LEDSubsystem;

import frc.robot.Constants;

public class NoteSubSystem extends SubsystemBase {

    public enum State{
        EMPTY,
        INTAKING_NOTE1,
        HAVE_NOTE1,
        INTAKING_NOTE2,
        HAVE_NOTE2,
        SHOOTING
    }

    public enum Target{
        SPEAKER,
        AMP,
        TRAP,
        INTAKE,
        FEEDSTATION,
        SPEAKER_PODIUM
    }

    public enum ActionRequest{
        IDLE,
        STOP,
        INTAKENOTE,
        BEAM3,
        BEAM1,
        BEAM2,
        SPIT_NOTE2,
        SHOOT
    }

    private State m_presentState;
    private ActionRequest m_wantedAction;
    private Target m_target;
    private RollerSubSystem m_Intake;
    private RollerSubSystem m_Feeder1;
    private RollerSubSystem m_Feeder2;
    private ShooterSubSystem m_Shooter;
    private AngleSubSystem m_Angle;
    private Timer m_shootRunTime;
    private Timer m_timeout;
    private double m_angle_setpoint;
    private double m_shooter_setpoint;
    private double m_shooterRight_setpoint;
    private double m_shooterfeeder2_setpoint;
    private double m_feeder2_setpoint;
    private double m_feeder1_setpoint;
    private double m_intake_setpoint;

    public NoteSubSystem(){
        m_presentState = State.EMPTY;
        m_target = Target.INTAKE;
        m_wantedAction = ActionRequest.IDLE;

        m_Intake = new RollerSubSystem("Intake", Constants.INTAKE.CANID, Constants.INTAKE.CANBUS, true);
        m_Feeder1 = new RollerSubSystem("Feeder1", Constants.FEEDER1.CANID, Constants.FEEDER1.CANBUS, false);
        m_Feeder2 = new RollerSubSystem("Feeder2", Constants.FEEDER2.CANID, Constants.FEEDER2.CANBUS, true);
        m_Shooter = new ShooterSubSystem();
        m_Angle = new AngleSubSystem();

        Shuffleboard.getTab("IntakeSubsystem").add(m_Intake);
        Shuffleboard.getTab("Feeder1Subsystem").add(m_Feeder1);
        Shuffleboard.getTab("Feeder2Subsystem").add(m_Feeder2);
        Shuffleboard.getTab("ShooterSubystem").add(m_Shooter);
        Shuffleboard.getTab("AngleSubsystem").add(m_Angle);
        
        m_shootRunTime = new Timer();
        m_timeout = new Timer();

        m_angle_setpoint=0;
        m_shooter_setpoint = Constants.SHOOTER.SHOOT_SPEED;
        m_shooterRight_setpoint=0;
        m_shooterfeeder2_setpoint = Constants.FEEDER2.SHOOT_SPEED;
        m_feeder2_setpoint = Constants.FEEDER2.TAKE_NOTE_SPEED;
        m_feeder1_setpoint=Constants.FEEDER1.TAKE_NOTE_SPEED;
        m_intake_setpoint=Constants.INTAKE.TAKE_NOTE_SPEED;

        System.out.println("Note subsystem created");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Note");
        builder.addStringProperty("State", () -> m_presentState.toString(),null);
        builder.addStringProperty("Target", () -> m_target.toString(),null);
        builder.addStringProperty("Action", () -> m_wantedAction.toString(),null);
        builder.addDoubleProperty("setpoint/shooter", this::getShooterSetpointSpeed,this::setShooterSetpointSpeed);
        builder.addDoubleProperty("setpoint/shooterR", this::getShooterRSetpointSpeed,this::setShooterRSetpointSpeed);
        builder.addDoubleProperty("setpoint/shooterFD2", this::getShooterFD2SetpointSpeed,this::setShooterFD2SetpointSpeed);
        builder.addDoubleProperty("setpoint/feeder2", this::getFeeder2SetpointSpeed,this::setFeeder2SetpointSpeed);
        builder.addDoubleProperty("setpoint/feeder1", this::getFeeder1SetpointSpeed,this::setFeeder1SetpointSpeed);
        builder.addDoubleProperty("setpoint/intake", this::getIntakeSetpointSpeed,this::setIntakeSetpointSpeed);
    }

    
    public double getShooterSetpointSpeed(){
        return this.m_shooter_setpoint;
    }
    public double getShooterRSetpointSpeed(){
        return this.m_shooterRight_setpoint;
    }
    public double getShooterFD2SetpointSpeed(){
        return this.m_shooterfeeder2_setpoint;
    }
    public double getFeeder2SetpointSpeed(){
        return this.m_feeder2_setpoint;
    }
    public double getFeeder1SetpointSpeed(){
        return this.m_feeder1_setpoint;
    }
    public double getIntakeSetpointSpeed(){
        return this.m_intake_setpoint;
    }

    public void setShooterSetpointSpeed(double desiredSpeed){
        m_shooter_setpoint=desiredSpeed;
    }
    public void setShooterRSetpointSpeed(double desiredSpeed){
        m_shooterRight_setpoint=desiredSpeed;
    }
    public void setShooterFD2SetpointSpeed(double desiredSpeed){
        m_shooterfeeder2_setpoint=desiredSpeed;
    }
    public void setFeeder2SetpointSpeed(double desiredSpeed){
        m_feeder2_setpoint=desiredSpeed;
    }
    public void setFeeder1SetpointSpeed(double desiredSpeed){
        m_feeder1_setpoint=desiredSpeed;
    }
    public void setIntakeSetpointSpeed(double desiredSpeed){
        m_intake_setpoint=desiredSpeed;
    }

    public void setTarget(Target wantedTarget) {
		m_target = wantedTarget;
        System.out.println("set note target: " + m_target.toString());

        switch(m_target){
            default:
            case SPEAKER:
                m_Angle.setState(AngleSubSystem.State.SPEAKER);
                break;
            case AMP:
                m_Angle.setState(AngleSubSystem.State.AMP);
                break;
            case TRAP:
                m_Angle.setState(AngleSubSystem.State.TRAP);
                break;
            case INTAKE:
                m_Angle.setState(AngleSubSystem.State.INTAKE);
                break;
            case FEEDSTATION:
                m_Angle.setState(AngleSubSystem.State.FEEDSTATION);
                break;
            case SPEAKER_PODIUM:
                m_Angle.setState(AngleSubSystem.State.SPEAKER_PODIUM);
                break;
        }

    }

    public void setAction(ActionRequest wantedAction) {
		m_wantedAction = wantedAction;
        System.out.println("set note action request: " + m_wantedAction.toString());
    }

    // this is the state machine of the notesubsystem
    @Override
    public void periodic() {
        switch(m_wantedAction){
            default:
            case IDLE:
                m_shootRunTime.reset();
                m_timeout.reset();
                break;
            case STOP:
                m_Intake.setSpeed(0);
                m_Feeder1.setSpeed(0);
                m_Feeder2.setSpeed(0);
                m_Shooter.setSpeed(0);
                m_Angle.setState(AngleSubSystem.State.BRAKE);
                setAction(ActionRequest.IDLE);

                switch (m_presentState){
                    case INTAKING_NOTE1:
                        setState(State.EMPTY);
                        break;
                    case INTAKING_NOTE2:
                        setState(State.HAVE_NOTE1);
                        break;
                    case SHOOTING:
                        setState(State.HAVE_NOTE1);
                        break;
                }
                break;
            case INTAKENOTE:
                if (m_presentState == State.EMPTY){
                    setTarget(Target.INTAKE);   //incase user did not press X button

                    m_timeout.restart();
                    if ((m_Angle.atAngle()) || 
                        (m_timeout.hasElapsed(Constants.ANGLE.ATANGLE_TIMEOUT))) {

                        System.out.println("  do note action: " + m_wantedAction.toString());
                        m_Intake.setSpeed(m_intake_setpoint);
                        m_Feeder1.setSpeed(m_feeder1_setpoint);
                        m_Feeder2.setSpeed(m_feeder2_setpoint);
                        setState(State.INTAKING_NOTE1);
                        setAction(ActionRequest.IDLE);
                    }
                }

                break;
            case BEAM3:
                if (m_presentState == State.INTAKING_NOTE1){
                    System.out.println("  do note action: " + m_wantedAction.toString() + " - STOP INTAKE");
                    m_Feeder2.setSpeed(0);
                    m_Feeder1.setSpeed(0);
                    m_Intake.setSpeed(0);
                    setState(State.HAVE_NOTE1);
                    setAction(ActionRequest.IDLE);
                }
                break;
            case BEAM1:
                
                break;
            case BEAM2:
                
                break;
            case SPIT_NOTE2:

                break;
            case SHOOT:
                if (m_presentState == State.HAVE_NOTE1){
                    System.out.println("  do note action1: " + m_wantedAction.toString());
                    m_Shooter.setRightOffsetSpeed(m_shooterRight_setpoint);
                    m_Shooter.setSpeed(m_shooter_setpoint);

                    m_timeout.restart();
                    if ((m_Angle.atAngle() && m_Shooter.atSpeed()) || 
                        (m_timeout.hasElapsed(Constants.ANGLE.ATANGLE_TIMEOUT))) {

                        m_Feeder2.setSpeed(m_shooterfeeder2_setpoint);
                        m_shootRunTime.restart();
                        setState(State.SHOOTING);
                    }
                }
                else if ((m_presentState == State.SHOOTING) && 
                         ( m_shootRunTime.hasElapsed(Constants.SHOOTER.STOP_TIME))) {
                    System.out.println("  do note action2 - stop shooting");
                    m_Shooter.setSpeed(0);
                    m_Feeder2.setSpeed(0);
                    setState(State.EMPTY);
                    setAction(ActionRequest.IDLE);
                }
                break;
        }
    }

    private void setState(State desiredStation){
        m_presentState = desiredStation;
        System.out.println("  set note state: " + m_presentState.toString());
    }
    public State getState(){
        return this.m_presentState;
    }
    public ActionRequest getAction(){
        return this.m_wantedAction;
    }
    public Target getTarget(){
        return this.m_target;
    }

    public void bumpIntake1Speed(double bumpAmount){
        m_Intake.bumpSpeed(bumpAmount);
        m_intake_setpoint=m_Intake.getSpeed();
        m_Feeder1.bumpSpeed(bumpAmount);
        m_feeder1_setpoint=m_Feeder1.getSpeed();
        m_Feeder2.bumpSpeed(bumpAmount);
        m_feeder2_setpoint=m_Feeder2.getSpeed();
    }

    public void bumpIntake2Speed(double bumpAmount){
        m_Intake.bumpSpeed(bumpAmount);
        m_intake_setpoint=m_Intake.getSpeed();
        m_Feeder1.bumpSpeed(bumpAmount);
        m_feeder1_setpoint=m_Feeder1.getSpeed();
    }

    public void bumpShooterSpeed(double bumpAmount){
        m_Shooter.setSpeed(m_shooter_setpoint);
        m_Shooter.bumpSpeed(bumpAmount);
        m_shooter_setpoint=m_Shooter.getSpeed();

        m_Feeder2.setSpeed(m_shooterfeeder2_setpoint);
        m_Feeder2.bumpSpeed(bumpAmount);
        m_shooterfeeder2_setpoint=m_Feeder2.getSpeed();
    }

    public void bumpAnglePosition(double bumpAmount){
        m_Angle.bumpPosition(bumpAmount);
    }

}