package frc.Subsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED {
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;

    int red;
    int green;
    int blue;

    private static LED instance = new LED();
    
    public LED(){
      led = new AddressableLED(1);
      //^^makes the led with the PWM port of 0 
      ledBuffer = new AddressableLEDBuffer(130);
      
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
  }

  public static LED getInstance(){
    return instance;
  }

  public void setLED(int red, int green, int blue){
      
    for(var bob = 0; bob < ledBuffer.getLength(); bob++){
        ledBuffer.setRGB(bob, Math.abs(red), Math.abs(green), Math.abs(blue));
        //^^ orange is 255, 25, 0
        }
    
        led.setData(ledBuffer);

        //SmartDashboard.putNumber("red value", red);
        //SmartDashboard.putNumber("green value", green);
        //SmartDashboard.putNumber("blue value", blue);
  }

  public void setRED(){
    for(var b0b = 0; b0b < ledBuffer.getLength(); b0b++)
      ledBuffer.setRGB(b0b, 255, 0, 0);

      led.setData(ledBuffer);
  }
  public void setGREEN(){
    for(var b0b = 0; b0b< ledBuffer.getLength(); b0b++)
      ledBuffer.setRGB(b0b, 0, 255, 0);

      led.setData(ledBuffer);
  }
  public void setORANGE(){
    for(var b0b = 0; b0b< ledBuffer.getLength(); b0b++)
    ledBuffer.setRGB(b0b, 255, 40, 0);

    led.setData(ledBuffer);
}
  
}