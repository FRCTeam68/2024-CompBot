package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.LED;

public class LEDSubsystem implements Subsystem {

    private AddressableLED stripL;
    private AddressableLED stripR;

  
    private AddressableLEDBuffer bufferL;
    private AddressableLEDBuffer bufferR;

    public LEDSubsystem() {
        stripL = new AddressableLED(LED.PWMPORT); //TODO: change port
        // stripR = new AddressableLED(0); //TODO: change port
        bufferL = new AddressableLEDBuffer(LED.BUFFERSIZE); //TODO: change length
        // bufferR = new AddressableLEDBuffer(0); //TODO: change length

        stripL.setLength(bufferL.getLength());
        // stripR.setLength(bufferR.getLength());

        stripL.setData(bufferL);
        // stripR.setData(bufferR);

        stripL.start();
        // stripR.start();
    }
    /**
     * Try to call only once when you change. Maybe create a toggle?
     * @param color
     */
    public void setLeftColor(Color color) {
        for (int i = 0; i < bufferL.getLength(); i++) {
            bufferL.setLED(i, color);
        }

        stripL.setData(bufferL);
    }

    /**
     * Try to call only once when you change. Maybe create a toggle?
     * @param color
     */
    public void setRightColor(Color color) {
        for (int i = 0; i < bufferR.getLength(); i++) {
            bufferR.setLED(i, color);
        }

        stripR.setData(bufferR);
    }

    public void setLeftLED(int index, Color color) {
        bufferL.setLED(index, color);
    }

    public void setRightLED(int index, Color color) {
        bufferR.setLED(index, color);
    }

    public void updateLeftLED(){
        stripL.setData(bufferL);
    }

    public void updateRightLED(){
        stripR.setData(bufferR);
    }
    
}