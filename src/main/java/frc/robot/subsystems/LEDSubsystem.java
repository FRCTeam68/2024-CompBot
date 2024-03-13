package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import frc.robot.subsystems.NoteSubSystem.Target;;


public class LEDSubSystem extends SubsystemBase {
    CANdle candle1 = new CANdle(60);
    //create a rainbow anim.
    RainbowAnimation rainbowAnimation = new RainbowAnimation(1, 1, 40);

    // LarsonAnimation manualModeSegment1 = new LarsonAnimation(0,0,0,0,1,10,0,3,8);
    //int r, int g, int b, int w, double speed, int numLed, BounceMode mode, int size, int ledOffset)

   
    public LEDSubSystem() {
        candle1.configLEDType(LEDStripType.RGB);
        candle1.clearAnimation(0);
        candle1.configBrightnessScalar(1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void setEmpty(){
        //all solid orange
        candle1.setLEDs(255, 24, 0, 0, 0, 48);
    }

    public void setIntaking(){
        //orange rolling
    }

    public void setHaveNote(){
        //solid blue
        candle1.setLEDs(0, 0, 255, 0, 0, 48);
    }

    public void setPrepareToShoot(Target mytarget, boolean isVisionMode){
        //blue rolling
        // for manual mode, this is when: waiting for angle to be at its setpoint
        //                                or for shooter to be up to speed
        // for vision mode, this also when: target not found yet,
        //                                  or outside distance we can shoot 
        //                                  or heading is still not correct

        //use the top LED to indicate manual mode target
        //red = speaker at the subwoofer
        //red blinking = speaker at podium
        //yellow = amp
        //orange = trap
        //white = other

        //top 3 LED lite means vision mode active
        // red = speaker,  
        // yellow = amp
        // orange = trap
        // all 3 top blink if target not found
        //                 target found, top is solid
        //                 within distance, 2nd is solid
        //                 heading locked on, 3rd solid  (and remaining solid green)



    }

    public void setReadyToShoot(Target mytarget, boolean isVisionMode){
        //green solid
        //this is at the setpoint angle and speed when shooting manually
        //    or in the case of vision, also within distance and the heading is correct

        //use the top LED to indicate manual mode target
        //red = speaker at the subwoofer
        //green = speaker at podium
        //yellow = amp
        //orange = trap
        //white = other

        //top 3 LED lite means vision mode
        // red = speaker
        // yellow = amp
        // orange = trap

        


    }




    public void candlePurple(){
        //set brightness
        candle1.configBrightnessScalar(1);
        //set color
        candle1.setLEDs(80, 45, 127);

    }

    public void candleGold(){
        //set brightness
        candle1.configBrightnessScalar(1);
        //set color
        candle1.setLEDs(255, 200, 46);
    }
    
    public void candleOrange(){
        //set brightness
        candle1.configBrightnessScalar(1);
        //set color
        candle1.setLEDs(255, 24, 0, 0, 48, 10);
        candle1.setLEDs(255, 24, 0, 0, 58, 10);
        candle1.setLEDs(255, 24, 0, 0, 68, 10);
        candle1.setLEDs(255, 24, 0, 0, 78, 10);
        //237,125,49 - robbie
        //230,126,34
    }

    public void candleBlue(){
        //set brightness
        candle1.configBrightnessScalar(1);
        //set color
        candle1.setLEDs(0, 0, 255, 0, 0, 8);
        candle1.setLEDs(0, 0, 255, 0, 8, 40);
        // 39,59,140 - robbie
        //0,0,255
    }

    public void candleRainbow(){
        //set brightness
        candle1.configBrightnessScalar(1);
        //set color
        candle1.animate(rainbowAnimation);
    }
}

