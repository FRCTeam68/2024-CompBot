package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BumpClimbSubSystem extends SubsystemBase {

    private NoteSubSystem m_myNoteSubSys;
    private ClimberSubSystem m_myClimbSubSys;
    private boolean m_climbMode;
    private boolean m_pitMode;

    public BumpClimbSubSystem(NoteSubSystem noteSubSys, ClimberSubSystem climbSubSys){
        m_myNoteSubSys = noteSubSys;
        m_myClimbSubSys = climbSubSys;
        m_climbMode = false;
        m_pitMode = false;
    }

    public void toggleClimbMode(){
        m_climbMode = !m_climbMode;
    }
    public boolean getClimbMode(){
        return m_climbMode;
    }
    public void togglePitMode(){
        m_pitMode = !m_pitMode;
    }
    public boolean getPitMode(){
        return m_pitMode;
    }


    public void bumpOrClimb(double leftjoyvalue, double rightjoyValue){

        System.out.println("bumpOrClimb/leftjoyvalue: " + leftjoyvalue);
        Logger.recordOutput("bumpOrClimb/rightjoyValue", rightjoyValue );

        if(m_pitMode){
            m_myClimbSubSys.setLeftSpeedVout(leftjoyvalue * 12);
            m_myClimbSubSys.setRightSpeedVout(rightjoyValue * 12);
        }
        else{
            if (leftjoyvalue > .7){
                m_myNoteSubSys.bumpIntake1Speed((-Constants.INTAKE.BUMP_VALUE));
            }
            else if (leftjoyvalue < -.7){
                m_myNoteSubSys.bumpIntake1Speed((Constants.INTAKE.BUMP_VALUE));
            }

            if (rightjoyValue > .7){
                m_myNoteSubSys.bumpIntake2Speed((-Constants.INTAKE.BUMP_VALUE));
            }
            else if (rightjoyValue < -.7){
                m_myNoteSubSys.bumpIntake2Speed((Constants.INTAKE.BUMP_VALUE));
            }
        }
    }
}
