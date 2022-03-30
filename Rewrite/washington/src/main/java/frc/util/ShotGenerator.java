package frc.util;

import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShotGenerator {

    public class ShooterSpeed {

        public final double topSpeed;
        public final double bottomSpeed; 
        public ShooterSpeed(double topSpeed, double bottomSpeed) { 
          this.topSpeed = topSpeed; 
          this.bottomSpeed = bottomSpeed;
        } 
    }


    
    //log data as {top wheel RPM, distance it made it in}
    private Double[][] topRPM = {
        {0d,1.96,2.36,2.45,2.81},
        {1275d,1365d,1565d,1700d,2650d}
    };
    private Double[][] ratio = {
        {0d,1.96,2.36,2.45,2.81},
        {1.08,1.0,.85,.78,.5}
    };
    SplineInterpolator normalInterpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(topRPM[0]), Arrays.asList(topRPM[1]));
    SplineInterpolator ratioInterpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(ratio[0]), Arrays.asList(ratio[1]));

    public ShooterSpeed getShot(double distance) {
        SplineInterpolator speedInterpolator = normalInterpolator;
        SplineInterpolator spinInterpolator = ratioInterpolator;

        double rpm = speedInterpolator.interpolate(distance);
        double shotRatio = spinInterpolator.interpolate(distance);
        SmartDashboard.putNumber("interpolated top", rpm);
        SmartDashboard.putNumber("interpolated ratio", shotRatio);
        return new ShooterSpeed(rpm, rpm*shotRatio);
    }

    public ShooterSpeed generateArbitraryShot(double top, double bot){
        return new ShooterSpeed(top, bot);
    }
}