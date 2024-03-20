package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.RollerSubSystem.Mode;

// --45 rotations/s at full throttle.
// --left motor 98 rotation range 
// --right motor 107 rotation range and needs negative voltage
// --distance traveled is 9"
// --45:1 (3:3:5) gear ratio
// --spool diameter 1.25"

public class ClimberSubSystem extends SubsystemBase {

    private double m_setPoint_Left_Speed;
    private double m_setPoint_Right_Speed;
    private TalonFX m_climberLeftMotor;
    private TalonFX m_climberRightMotor;
    private VoltageOut m_voltageOut;
    private NeutralOut m_neutral;
    private Mode m_presentMode;
    private double m_setPoint_Left_Voltage;
    private double m_setPoint_Right_Voltage;
    private double m_leftPosition;
    private double m_rightPosition;
    private boolean m_pitMode;

    public ClimberSubSystem(){
        m_setPoint_Left_Speed = 0;
        m_setPoint_Right_Speed = 0;
        m_leftPosition = 0;
        m_rightPosition = 0;

        climberMotorsInit();
    }

    private void climberMotorsInit(){
        m_climberLeftMotor = new TalonFX(Constants.CLIMBER.LEFT_CANID, Constants.CLIMBER.CANBUS);
        m_climberRightMotor = new TalonFX(Constants.CLIMBER.RIGHT_CANID, Constants.CLIMBER.CANBUS);

        m_climberLeftMotor.setInverted(true);
        m_climberRightMotor.setInverted(false);  // pick CW versus CCW

        m_voltageOut = new VoltageOut(0);

        /* Keep a neutral out so we can disable the motor */
        m_neutral = new NeutralOut();
  
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.Feedback.SensorToMechanismRatio = 1.0;

        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;
          
        // Peak output of 40 amps
        // configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        // configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.withCurrentLimits(Constants.limit60);


        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        // configs.CurrentLimits.SupplyCurrentLimit = 30.0;
        
        // configs.CurrentLimits.SupplyTimeThreshold = 0.01;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
          status = m_climberLeftMotor.getConfigurator().apply(configs);
          if (status.isOK()) break;
        }
        if(!status.isOK()) {
          System.out.println("Could not apply configs to left climber motor, error code: " + status.toString());
        }

        // Right motor needs negative voltage.  Invert it so position reports positive 
        configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        /* Retry config apply up to 5 times, report if failure */
        for (int i = 0; i < 5; ++i) {
          status = m_climberRightMotor.getConfigurator().apply(configs);
          if (status.isOK()) break;
        }
        if(!status.isOK()) {
          System.out.println("Could not apply configs to right climber motor, error code: " + status.toString());
        }

        System.out.println("climber subsystem created");
        Logger.recordOutput("Climber/Comment",  "subsystem created");
    }

    public void setSpeedVout(double leftDesiredVoltage, double rightDesiredVoltage){
        // System.out.println("climber setSpeedesired" + desiredVoltage);


        if(m_presentMode == Mode.VOLTAGE_OUT){

            m_leftPosition = m_climberLeftMotor.getPosition().getValueAsDouble();
            if ((Math.abs(leftDesiredVoltage) <= 1)){  //|| (m_leftPosition > 12)) {
                System.out.println("set left zero");
                m_setPoint_Left_Voltage = 0;
            }
            else {
                m_setPoint_Left_Voltage = leftDesiredVoltage;
            }

            m_rightPosition = m_climberLeftMotor.getPosition().getValueAsDouble();
            if ((Math.abs(rightDesiredVoltage) <= 1)){  //|| (m_rightPosition > 12)) {
                System.out.println("set right zero");
                m_setPoint_Right_Voltage = 0;
            }
            else {
                m_setPoint_Right_Voltage = rightDesiredVoltage;
            }


            System.out.println("climber setSpeedVout, L: " + m_setPoint_Left_Voltage 
                                                + " , R: " + m_setPoint_Right_Voltage);
            m_climberLeftMotor.setControl(m_voltageOut.withOutput(m_setPoint_Left_Voltage));
            m_climberRightMotor.setControl(m_voltageOut.withOutput(m_setPoint_Right_Voltage));
        }
    }


    // public void setSpeed(double desiredRotationsPerSecond){

    //     System.out.println("set desired speed: " + desiredRotationsPerSecond);

    //     if (Math.abs(desiredRotationsPerSecond) <= 1) { // Joystick deadzone
    //         desiredRotationsPerSecond = 0;
    //         m_setPoint_Left_Speed = 0;
    //         m_setPoint_Right_Speed = 0;
    //         m_climberLeftMotor.setControl(m_neutral);
    //         m_climberRightMotor.setControl(m_neutral);
    //     }
    //     else
    //         m_setPoint_Left_Speed = desiredRotationsPerSecond;
    //         m_setPoint_Right_Speed = -desiredRotationsPerSecond + m_rightOffset_Speed;

    //         System.out.println("climber present mode: " + m_presentMode.toString());
    //         switch(m_presentMode){
    //             case VOLTAGE_OUT:
    //                 System.out.println("voltage out mode - use setSpeedVout instead");
    //                 break;
    //             default:
    //             case VOLTAGE_FOC:
    //                 /* Use voltage velocity */
    //                 //m_intakeMotor.setControl(m_voltageVelocity.withVelocity(desiredRotationsPerSecond));
    //                 System.out.println("call motor voltage foc");
    //                 m_climberLeftMotor.setControl(m_voltageVelocity.withVelocity(m_setPoint_Left_Speed));
    //                 m_climberRightMotor.setControl(m_voltageVelocity.withVelocity(m_setPoint_Right_Speed));
    //                 break;
        
    //             case CURRENTTORQUE_FOC:
    //                 double friction_torque = (desiredRotationsPerSecond > 0) ? 1 : -1; // To account for friction, we add this to the arbitrary feed forward
    //                 /* Use torque velocity */
    //                 //m_intakeMotor.setControl(m_torqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(friction_torque));
    //                 System.out.println("call motor torqueVelocity");
    //                 m_climberLeftMotor.setControl(m_torqueVelocity.withVelocity(m_setPoint_Left_Speed).withFeedForward(friction_torque));
    //                 m_climberRightMotor.setControl(m_torqueVelocity.withVelocity(m_setPoint_Right_Speed).withFeedForward(friction_torque));
    //                 break;
    //         }
    // }


    public double getLeftSpeed(){
        return this.m_setPoint_Left_Speed;
    }

    public double getRightSpeed(){
        return this.m_setPoint_Right_Speed;
    }

    public double getLeftPosition(){
        return this.m_leftPosition;
    }

    public double getRightPosition(){
        return this.m_rightPosition;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Climber");
        builder.setActuator(true);
        //builder.setSafeState(() -> setState(State.BREAK));
        //builder.addDoubleProperty("setpoint speed", null,this::setSpeed);
        builder.addDoubleProperty("left speed", this::getLeftSpeed,null);
        builder.addDoubleProperty("right speed", this::getRightSpeed,null);
        builder.addDoubleProperty("left position", this::getLeftPosition,null);
        builder.addDoubleProperty("right position", this::getRightPosition,null);
    }

    public Object setPitMode(boolean m_climbActive) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPitMode'");
    }

}