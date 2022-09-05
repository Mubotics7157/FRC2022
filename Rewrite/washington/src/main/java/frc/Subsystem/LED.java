package frc.Subsystem;


import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import frc.util.OrangeUtility;

public class LED {
    CANdle candle;
    public static LED instance = new LED();
    CANdleConfiguration config;
    RainbowAnimation rainbow;
    boolean toggle;
    public LED(){
        candle = new CANdle(31);
        config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = .5;
        candle.setLEDs(255, 255, 255);
        rainbow = new RainbowAnimation(1, 1, 211);
        candle.configAllSettings(config);
        toggle = true;  
    }

    public static LED getInstance(){
      return instance;
    }

    public synchronized void setRainbow(){
        candle.animate(rainbow);
    }

    public void setRED(){
        candle.setLEDs(255, 0, 0);
    }

    public void setGREEN(){
        candle.setLEDs(0, 255, 0);
    }

    public void setBLUE(){
        candle.setLEDs(0, 0, 255);
    }

    public void setWHITE(){
        candle.setLEDs(255, 255, 255);
    }

    public void setOFF(){
        candle.setLEDs(0, 0, 0);
    }

    public synchronized void setORANGE(){
        candle.setLEDs(255, 45, 0);
    }

    public boolean getToggle(){
        return toggle;
    }

    public void changeToggle(){
        toggle = !toggle;
    }


    public synchronized void setSeizure(){
        candle.setLEDs(255, 45, 0);
        OrangeUtility.sleep(500);
        candle.animate(rainbow);
    }
}