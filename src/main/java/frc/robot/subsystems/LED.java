package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import frc.robot.Constants;

public class LED extends SubsystemBase{
    CANdle leds = new CANdle(60);

    private CommandXboxController controller;

    public LED(CommandXboxController controller) {
        this.controller = controller;
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.5;   
        configAll.vBatOutputMode = VBatOutputMode.On;
        leds.configAllSettings(configAll, 100);
    }
    
    @Override
    public void periodic(){
        leds.setLEDs(255, 0, 255);
    }
}
