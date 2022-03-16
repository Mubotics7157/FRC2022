package frc.Subystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LED {
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;

    int red;
    int green;
    int blue;

    public boolean isAligning;
    
    public LED(){
      led = new AddressableLED(0);
      //^^makes the led with the PWM port of 0 
      ledBuffer = new AddressableLEDBuffer(300);
      
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        red = 0;
        blue = 0;
        green = 0;

        MathUtil.clamp(red, 0, 255);
        MathUtil.clamp(green, 0, 255);
        MathUtil.clamp(blue, 0, 255);  
  }

  public void setLED(){
      
    for(var bob = 0; bob < ledBuffer.getLength(); bob++){
        ledBuffer.setRGB(bob, Math.abs(red), Math.abs(green), Math.abs(blue));
        //^^ orange is 255, 25, 0
        }
    
        led.setData(ledBuffer);

        SmartDashboard.putNumber("red value", red);
        SmartDashboard.putNumber("green value", green);
        SmartDashboard.putNumber("blue value", blue);
  }
}
