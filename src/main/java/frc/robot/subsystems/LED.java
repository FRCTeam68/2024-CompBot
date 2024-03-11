package frc.robot.subsystems;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj.PowerDistribution;


public class LED extends SubsystemBase {
    //CANdle stuff
    //purple = 80, 45, 127
    //gold = 255, 200, 46
    //define candle
    CANdle candle1 = new CANdle(60);
    //create a rainbow anim.
    RainbowAnimation rainbowAnimation = new RainbowAnimation(1, 1, 0);

    // private PowerDistribution pDH;

   
    public LED() {
        // pDH = new PowerDistribution();
        // addChild("PDH",pDH);
 
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    //CANdle
    //purple = 80, 45, 127
    //gold = 255, 200, 46

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

    public void candleRainbow(){
        //set brightness
        candle1.configBrightnessScalar(1);
        //set color
        candle1.animate(rainbowAnimation);

    }
}

