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
    
    public LED(){
      led = new AddressableLED(0);
      //^^makes the led with the PWM port of 0 
      ledBuffer = new AddressableLEDBuffer(300);
      
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
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
}
