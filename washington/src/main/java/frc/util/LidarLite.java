package frc.util;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;

public class LidarLite {
    Counter lidar;
    double distance;
    final double angleOffsetRadians = 0; 

    public LidarLite(DigitalSource source){
        lidar = new Counter(source);
        lidar.setMaxPeriod(1.0);
        lidar.setSemiPeriodMode(true);
        lidar.reset();
    }

    public double getDistance(){
        if(lidar.get()<1)
            return 0;
        else{
        distance = (lidar.getPeriod()*1000000/10)-angleOffsetRadians;
        distance*=Math.cos(angleOffsetRadians);
        return distance;
        }


    }
    
}
