package frc.robot.subsystems;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NoteSubSystem extends SubsystemBase {

    public enum State{
         IDLE,
        INTAKING_NOTE1,
        SHOOTING, 
        SHOOT_SPINNING, 
        SPITTING_NOTE
    }

    public enum Target{
        SPEAKER,
        AMP,
        TRAP,
        INTAKE,
        FEEDSTATION,
        SPEAKER_PODIUM, 
        SPEAKER_1M, 
        SPEAKER_PODIUM_SOURCE
    }

    public enum ActionRequest{
        IDLE,
        STOP,
        STOP_ALL,
        INTAKENOTE,
        BEAM3,
        // BEAM1,
        // BEAM2,
        SPIT_NOTE2,
        SHOOT, 
        SHOOT_SPINUP,
        //FEEDSTATION_SPIN,
        DISLODGE_WITH_SHOOTER
    }

    private State m_presentState;
    private ActionRequest m_wantedAction;
    private Target m_target;
    private boolean m_haveNote1;
    private boolean m_spunShooterUp;
    private RollerSubSystem m_Intake;
    private RollerSubSystem m_Feeder1;
    private RollerSubSystem m_Feeder2;
    private ShooterSubSystem m_Shooter;
    private AngleSubSystem m_Angle;
    private Timer m_shootStopTime;
    private double m_shooter_setpoint;
    private double m_shooterRight_setpoint;
    private double m_shooterfeeder2_setpoint;
    private double m_feeder2_setpoint;
    private double m_feeder1_setpoint;
    private double m_intake_setpoint;

    public NoteSubSystem(){
        m_presentState = State.IDLE;
        m_target = Target.SPEAKER;
        m_wantedAction = ActionRequest.IDLE;
        setHaveNote1(false);
        setShooterSpunUp(false);

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
        
        m_shootStopTime = new Timer();

        resetSetpoints();

        System.out.println("Note subsystem created");
        Logger.recordOutput("Note/Comment",  "Note subsystem created");
        Logger.recordOutput("Note/State",  m_presentState);
        Logger.recordOutput("Note/Target",  m_target);
        Logger.recordOutput("Note/Action",  m_wantedAction);
    }

    public void resetSetpoints(){
        
        switch(m_target){
            default:
            case SPEAKER, SPEAKER_1M, SPEAKER_PODIUM:
                m_shooter_setpoint = Constants.SHOOTER.SPEAKER_SHOOT_SPEED;
                break;
            case AMP:
                m_shooter_setpoint = Constants.SHOOTER.AMP_SHOOT_SPEED;
                break;
            case TRAP:
                m_shooter_setpoint = Constants.SHOOTER.TRAP_SHOOT_SPEED;
                break;
            // case FEEDSTATION:
            //    not implemented yet
            //     break;
        }
        
        m_shooterRight_setpoint = Constants.SHOOTER.RIGHT_OFFSET;
        m_shooterfeeder2_setpoint = Constants.FEEDER2.SHOOT_SPEED;

        m_feeder2_setpoint = Constants.FEEDER2.TAKE_NOTE_SPEED;
        m_feeder1_setpoint = Constants.FEEDER1.TAKE_NOTE_SPEED;
        m_intake_setpoint = Constants.INTAKE.TAKE_NOTE_SPEED;
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
        builder.addBooleanProperty("HaveNote1", this::getHaveNote1,null);
        builder.addBooleanProperty("ShooterSpunUp", this::getShooterSpunUp,null);
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

        if (wantedTarget != m_target){
            m_target = wantedTarget;
            Logger.recordOutput("Note/Comment",  "target change");
            Logger.recordOutput("Note/Target",  m_target);
            switch(m_target){
                default:
                case SPEAKER:
                    m_Angle.setState(AngleSubSystem.State.SPEAKER);
                    m_shooter_setpoint = Constants.SHOOTER.SPEAKER_SHOOT_SPEED;
                    if (!m_spunShooterUp){spinUp();}
                    break;
                case SPEAKER_1M:
                    m_Angle.setState(AngleSubSystem.State.SPEAKER_1M);
                    m_shooter_setpoint = Constants.SHOOTER.SPEAKER_SHOOT_SPEED;
                    if (!m_spunShooterUp){spinUp();}
                    break;
                case AMP:
                    m_Angle.setState(AngleSubSystem.State.AMP);
                    m_shooter_setpoint = Constants.SHOOTER.AMP_SHOOT_SPEED;
                    if (!m_spunShooterUp){spinUp();}
                    break;
                case TRAP:
                    m_Angle.setState(AngleSubSystem.State.TRAP);
                    m_shooter_setpoint = Constants.SHOOTER.TRAP_SHOOT_SPEED;
                    if (!m_spunShooterUp){spinUp();}
                    break;
                case INTAKE:
                    m_Angle.setState(AngleSubSystem.State.INTAKE);
                    // for when we want to shoot with this angle.
                    m_shooter_setpoint = Constants.SHOOTER.SPEAKER_SHOOT_SPEED;
                    break;
                // case FEEDSTATION:
                //   not implemented yet
                //     m_Angle.setState(AngleSubSystem.State.FEEDSTATION);
                //     break;
                case SPEAKER_PODIUM:
                    m_Angle.setState(AngleSubSystem.State.SPEAKER_PODIUM);
                    m_shooter_setpoint = Constants.SHOOTER.SPEAKER_SHOOT_SPEED;
                    break;
                case SPEAKER_PODIUM_SOURCE:
                    m_Angle.setState(AngleSubSystem.State.SPEAKER_PODIUM_SOURCE);
                    m_shooter_setpoint = Constants.SHOOTER.SPEAKER_SHOOT_SPEED;
                    break;
            }

            if (m_spunShooterUp){
                m_Shooter.setRightOffsetSpeed(m_shooterRight_setpoint);
                m_Shooter.setSpeed(m_shooter_setpoint);
            }
        }


    }

    public void setAction(ActionRequest wantedAction) {
		m_wantedAction = wantedAction;
        Logger.recordOutput("Note/Action",  m_wantedAction);
    }

    // this is the state machine of the notesubsystem
    @Override
    public void periodic() {

        boolean isAtAngle = false;

        switch(m_wantedAction){
            default:
            case IDLE:

                if (m_shootStopTime.hasElapsed(.3)){
                    // shooting  has started and timer elasped
                    // account for BEAM3 action was missed that would have set back no NOTE1
                    m_shootStopTime.stop();
                    m_shootStopTime.reset();
                    Logger.recordOutput("Note/Comment",  "shoot timer elapsed");
                    if (m_haveNote1){
                        // we can leave it running  m_Feeder2.setSpeed(0);  
                        setHaveNote1(false);
                        setState(State.IDLE);
                    }
                }
                break;
            case STOP:
                Logger.recordOutput("Note/Comment",  "stop 40");
                m_Intake.setSpeed(0);
                m_Feeder1.setSpeed(0);
                m_Feeder2.setSpeed(0);
                m_Shooter.setSpeed(40);
                m_shootStopTime.stop();
                m_shootStopTime.reset();
                setShooterSpunUp(true);
                setState(State.IDLE);
                setAction(ActionRequest.IDLE);
                break;
            case STOP_ALL:
                Logger.recordOutput("Note/Comment",  "stop all");
                m_Intake.setSpeed(0);
                m_Feeder1.setSpeed(0);
                m_Feeder2.setSpeed(0);
                m_Shooter.setSpeed(0);
                m_shootStopTime.stop();
                m_shootStopTime.reset();
                setShooterSpunUp(false);
                setState(State.IDLE);
                setAction(ActionRequest.IDLE);
                break;
            case INTAKENOTE:
                if (m_target != Target.INTAKE){
                    setTarget(Target.INTAKE);
                }
                if (!m_haveNote1){
                    if (m_Angle.atAngle()){
                        Logger.recordOutput("Note/Comment",  "start intake");
                        m_Intake.setSpeed(m_intake_setpoint);
                        m_Feeder1.setSpeed(m_feeder1_setpoint);
                        m_Feeder2.setSpeed(m_feeder2_setpoint);
                        setState(State.INTAKING_NOTE1);
                        setAction(ActionRequest.IDLE);
                     }
                }
                else{
                    Logger.recordOutput("Note/Comment",  "no intake, have note");
                    spinUp();
                    //we have a note.  do not intake
                    setAction(ActionRequest.IDLE);
                }
                break;
            case BEAM3:
                if (m_presentState == State.INTAKING_NOTE1){
                    Logger.recordOutput("Note/Comment",  "stop intake");
                    m_Feeder2.setSpeed(0);
                    m_Feeder1.setSpeed(0);
                    m_Intake.setSpeed(0);
                    m_shootStopTime.stop();
                    m_shootStopTime.reset();
                    setHaveNote1(true);
                    setState(State.IDLE);
                    setAction(ActionRequest.IDLE);
                }
                else if(m_presentState == State.SHOOTING){
                    Logger.recordOutput("Note/Comment",  "note shot");
                    //backside of note coming through
                    // it can keep running -- m_Feeder2.setSpeed(0);
                    m_shootStopTime.stop();
                    m_shootStopTime.reset();
                    setHaveNote1(false);
                    setState(State.IDLE);
                    setAction(ActionRequest.IDLE);
                }
                break;
            case SPIT_NOTE2:
                setTarget(Target.INTAKE);
                if (m_Angle.atAngle()){
                    Logger.recordOutput("Note/Comment",  "spit note");
                    m_Intake.setSpeed(-m_intake_setpoint);
                    m_Feeder1.setSpeed(-m_feeder1_setpoint);
                    m_Feeder2.setSpeed(-m_feeder2_setpoint);
                    setState(State.SPITTING_NOTE);
                    setAction(ActionRequest.IDLE);
                    }
                break;
            case DISLODGE_WITH_SHOOTER:
                Logger.recordOutput("Note/Comment",  "reverse spin shooter");
                m_Shooter.setRightOffsetSpeed(0);
                m_Feeder2.setSpeed(-m_shooterfeeder2_setpoint);
                m_Shooter.setSpeed(Constants.SHOOTER.DISLODGE_SHOOT_SPEED);
                setShooterSpunUp(false);
                setAction(ActionRequest.IDLE);
                break;
            case SHOOT_SPINUP:
                    Logger.recordOutput("Note/Comment",  "spinup shooter");
                    spinUp();
                    setAction(ActionRequest.IDLE);
                break;
            case SHOOT:

                //  if (m_Shooter.atSpeed()) -- not implemented yet

                if (m_Angle.atAngle()){
                    Logger.recordOutput("Note/Comment",  "feed shooter");
                    m_Feeder2.setSpeed(m_shooterfeeder2_setpoint);
                    setState(State.SHOOTING);
                    m_shootStopTime.restart();
                    setAction(ActionRequest.IDLE);
                 }
                break;
                    }

        isAtAngle = m_Angle.atAngle();
        SmartDashboard.putBoolean("AtAngle AMP", (m_target == Target.AMP)&&(isAtAngle));
        SmartDashboard.putBoolean("AtAngle TRAP", (m_target == Target.TRAP)&&(isAtAngle));
        SmartDashboard.putBoolean("AtAngle Podium", (m_target == Target.SPEAKER_PODIUM)&&(isAtAngle));
        SmartDashboard.putBoolean("AtAngle Intake", (m_target == Target.INTAKE)&&(isAtAngle));
        SmartDashboard.putBoolean("AtAngle Speaker", (m_target == Target.SPEAKER)&&(isAtAngle));
    }

    private void spinUp(){
        m_Shooter.setRightOffsetSpeed(m_shooterRight_setpoint);
        m_Shooter.setSpeed(m_shooter_setpoint);
        setShooterSpunUp(true);
    }

    public void setHaveNote1(boolean haveNote1){
        m_haveNote1 = haveNote1;
        Logger.recordOutput("Note/HaveNote1",  m_haveNote1);
    }
    public boolean getHaveNote1(){
        return this.m_haveNote1;
    }

    public void setShooterSpunUp(boolean spunUp){
        m_spunShooterUp = spunUp;
        Logger.recordOutput("Note/shooterSpunUp",  m_spunShooterUp);
    }
    public boolean getShooterSpunUp(){
        return this.m_spunShooterUp;
    }

    private void setState(State desiredStation){
        m_presentState = desiredStation;
        Logger.recordOutput("Note/State",  m_presentState);
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

        // m_Feeder2.setSpeed(m_shooterfeeder2_setpoint);
        // m_Feeder2.bumpSpeed(bumpAmount);
        // m_shooterfeeder2_setpoint=m_Feeder2.getSpeed();
    }

    public void setPassSpeed(double speed){
        setTarget(Target.INTAKE);
        m_shooter_setpoint = speed;
        m_Shooter.setSpeed(m_shooter_setpoint);
        // setAction(ActionRequest.SHOOT);   cannot do this unless AtSpeed works
        // so use user delay for spinup to happen.
    }

    public void bumpAnglePosition(double bumpAmount){
        m_Angle.bumpPosition(bumpAmount);
    }

    public void zeroAngleSubsystem(){
        m_Angle.zeroAngleSensor();
    }

}