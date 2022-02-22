/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util.Shooting;

import java.util.Arrays;

import frc.robot.Constants.ShooterConstants;

/**
 * Add your docs here.
 */
public class ShotGenerator {
    private enum BACKSPINRATIOS{
        NORMAL(ShooterConstants.NORMAL_RATIO);

        private final double value;

        BACKSPINRATIOS(final double newValue) {
            value = newValue;
        }
        public double getValue() { return value; }
    }


    public class ShooterSpeed {

        public final double top;
        public final double bot; 
        public ShooterSpeed(double topSpeed, double bottomSpeed) { 
          top = topSpeed; 
          bot = bottomSpeed;
        } 
    }


    //log data as {top wheel RPM, distance it made it in}
    private Double[][] normal = {
        {494d,495d,496d}, // X (distance)
        {3450d,9999d,10000d} // Y (RPM)
    };
    
    

    SplineInterpolator normalInterpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(normal[1]), Arrays.asList(normal[0]));

    public ShooterSpeed getShot(double distance) {
        SplineInterpolator interpolator;
        BACKSPINRATIOS backSpin = BACKSPINRATIOS.NORMAL;

        switch (backSpin) {
            case NORMAL:
                interpolator = normalInterpolator;
                break;
            default:
                return null;
        }

        double rpm = interpolator.interpolate(distance);
        return new ShooterSpeed(rpm/backSpin.getValue(), rpm);
    }
}