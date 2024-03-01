package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubSystem extends SubsystemBase {

    public enum State{
        IDLE,
        CLIMB
    }

    public enum Mode{
        VOLTAGE_OUT,
        VOLTAGE_FOC,
        CURRENTTORQUE_FOC
    }

    private State m_presentState;
    private Mode m_presentMode;
    private double m_setPoint_Left_Speed;
    private double m_setPoint_Right_Speed;
    private TalonFX m_climberLeftMotor;
    private TalonFX m_climberRightMotor;
    private VoltageOut m_voltageOut;
    private VelocityVoltage m_voltageVelocity;
    private VelocityTorqueCurrentFOC m_torqueVelocity;
    private NeutralOut m_neutral;
    private double m_setPoint_Left_Voltage;
    private double m_setPoint_Right_Voltage;
    private double m_leftPosition;
    private double m_rightPosition;
    private boolean m_resetMode;

    public ClimberSubSystem(){
        m_presentState = State.IDLE;
        m_presentMode = Mode.VOLTAGE_OUT;
        m_setPoint_Left_Speed = 0;
        m_setPoint_Right_Speed = 0;
        m_leftPosition = 0;
        m_rightPosition = 0;
        m_resetMode = false;

        climberMotorsInit();
    }

    private void climberMotorsInit(){
        m_climberLeftMotor = new TalonFX(Constants.CLIMBER.LEFT_CANID);
        m_climberRightMotor = new TalonFX(Constants.CLIMBER.RIGHT_CANID);

        m_climberLeftMotor.setInverted(true);
        m_climberRightMotor.setInverted(false);  // pick CW versus CCW

        m_voltageOut = new VoltageOut(0);

          /* Start at velocity 0, enable FOC, no feed forward, use slot 0 */
        m_voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, 
                                 false, false, false);
          /* Start at velocity 0, no feed forward, use slot 1 */
        m_torqueVelocity = new VelocityTorqueCurrentFOC(0, 0, 0, 1, 
                                         false, false, false);
        /* Keep a neutral out so we can disable the motor */
        m_neutral = new NeutralOut();
  
        System.out.println("climber subsystem created");
        

        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.Feedback.SensorToMechanismRatio = 1.0;

        /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
        //0.11, 0.5, 0.0001, 0.12.  , .21 got there faster,  1 was too high (oscilated around zero when off)
        // still only getting to 20 instead of 40 that was being commanded
        configs.Slot0.kP = .2; // An error of 1 rotation per second results in 2V output  
        configs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
        configs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
        configs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
        // Peak output of 8 volts
        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;
        
        /* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
        configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 amps output
        configs.Slot1.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
        configs.Slot1.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
    
        // Peak output of 40 amps
        // configs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        // configs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
          status = m_climberLeftMotor.getConfigurator().apply(configs);
          if (status.isOK()) break;
        }
        if(!status.isOK()) {
          System.out.println("Could not apply configs to left climber motor, error code: " + status.toString());
        }

        /* Retry config apply up to 5 times, report if failure */
        for (int i = 0; i < 5; ++i) {
          status = m_climberRightMotor.getConfigurator().apply(configs);
          if (status.isOK()) break;
        }
        if(!status.isOK()) {
          System.out.println("Could not apply configs to right climber motor, error code: " + status.toString());
        }
    }

    public void setLeftSpeedVout(double desiredVoltage){
        // System.out.println("climber setSpeedesired" + desiredVoltage);
        if(m_presentMode == Mode.VOLTAGE_OUT){

            m_leftPosition = m_climberLeftMotor.getPosition().getValueAsDouble();

            if ((Math.abs(desiredVoltage) <= 1)){
                //deadzone for motor control, just make it zero volts
                // System.out.println("set left zero");
                m_setPoint_Left_Voltage = 0;
            }
            else if ((desiredVoltage > 1) && (m_leftPosition > Constants.CLIMBER.MAX_HEIGHT)){
                //asking to travel up, but already at max height
                System.out.println("left climber max height reached");
                m_setPoint_Left_Voltage = 0;
            } 
            else if ((desiredVoltage < -1) && (m_leftPosition <= 0)){
                //asking to travel down, but already at min height
                //    resetMode active needs negative values to respool climber in the pits
                //    because when you turn robot on position will be zero.
                //    So do not limit at zero.
                if (!m_resetMode){
                    System.out.println("left climber start height 0 reached");
                    m_setPoint_Left_Voltage = 0;
                } 
            }  
            else {
                m_setPoint_Left_Voltage = desiredVoltage;
            }

            System.out.println("left climber setSpeedVout: " + m_setPoint_Left_Voltage);
            Logger.recordOutput("Climber/LeftPos", m_leftPosition );
            Logger.recordOutput("Climber/LeftVolt", m_setPoint_Left_Voltage );
            m_climberLeftMotor.setControl(m_voltageOut.withOutput(m_setPoint_Left_Voltage));
        }
    }

    public void setRightSpeedVout(double desiredVoltage){
        // System.out.println("climber setSpeedesired" + desiredVoltage);
        if(m_presentMode == Mode.VOLTAGE_OUT){

            m_rightPosition = m_climberRightMotor.getPosition().getValueAsDouble();

            if ((Math.abs(desiredVoltage) <= 1)){
                //deadzone for motor control, just make it zero volts
                // System.out.println("set left zero");
                m_setPoint_Right_Voltage = 0;
            }
            else if ((desiredVoltage > 1) && (m_rightPosition > Constants.CLIMBER.MAX_HEIGHT)){
                //asking to travel up, but already at max height
                System.out.println("right climber max height reached");
                m_setPoint_Right_Voltage = 0;
            } 
            else if ((desiredVoltage < -1) && (m_rightPosition <= 0)){
                //asking to travel down, but already at min height
                //    resetMode active needs negative values to respool climber in the pits
                //    because when you turn robot on position will be zero.
                //    So do not limit at zero.
                if (!m_resetMode){
                    System.out.println("right climber start height 0 reached");
                    m_setPoint_Right_Voltage = 0;
                }
            } 
            else {
                m_setPoint_Right_Voltage = desiredVoltage;
            }

            System.out.println("right climber setSpeedVout: " + m_setPoint_Right_Voltage);
            Logger.recordOutput("Climber/RightPos", m_rightPosition );
            Logger.recordOutput("Climber/RightVolt", m_setPoint_Right_Voltage );
            m_climberRightMotor.setControl(m_voltageOut.withOutput(m_setPoint_Right_Voltage));
        }
    }

    public void setResetMode(boolean resetMode){
        m_resetMode = resetMode;
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
        builder.addStringProperty("State", () -> m_presentState.toString(),null);
        builder.addStringProperty("Mode", () -> m_presentMode.toString(),null);
    }


    public void setState(State wantedState) {
		m_presentState = wantedState;

        double desiredSpeed = 0;

        System.out.println("set Climber state: " + wantedState.toString());

        switch(wantedState){

            case CLIMB:
            // 2/10/2024 do nothing right now
                // desiredSpeed = m_spinUp_Speed;
                break;
            default:
            case IDLE:
                //nothing, stay with zero speed
                break;
        }

        // this.setSpeed(desiredSpeed);
	}

    public State getState(){
        return this.m_presentState;
    }

}