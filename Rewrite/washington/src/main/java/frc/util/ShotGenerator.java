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
        {2.41,3.28,3.96},
        {1525d,1790d,3250d}//#region}
    };
    private Double[][] ratio = {
        {2.41,3.28,3.96},
        {1.92,.725,.025}
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