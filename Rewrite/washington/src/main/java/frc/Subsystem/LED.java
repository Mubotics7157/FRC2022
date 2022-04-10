package frc.Subsystem;


import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

public class LED {
    CANdle candle;
    public static LED instance = new LED();
    CANdleConfiguration config;
    RainbowAnimation rainbow;
    public LED(){
        candle = new CANdle(31);
        config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = .5;
        candle.setLEDs(255, 255, 255);
        rainbow = new RainbowAnimation(1, 1, 200);
        candle.configAllSettings(config);
        
    }

    public static LED getInstance(){
      return instance;
    }

    public void setRainbow(){
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

    public void setORANGE(){
        candle.setLEDs(255, 45, 0);
    }
}