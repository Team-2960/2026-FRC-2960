package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public class LaserCAN {
    private static LaserCan laserCanLeft = new LaserCan(Constants.leftLaserCanID);
    private static LaserCan laserCanRight  = new LaserCan(Constants.rightLaserCanID);
    public Distance distance;

    public LaserCAN() {
    }

    public static Distance getLeftDistance() {
        return Millimeters.of(laserCanLeft.getMeasurement().distance_mm);
    }
    
    public static Distance getRightDistance() {
        return Millimeters.of(laserCanRight.getMeasurement().distance_mm);
    }

    public static Distance getAvgDistance(){
        return getLeftDistance().plus(getRightDistance()).div(2);
    }

    public static Distance getMaxDistance(){
        return getLeftDistance().gte(getRightDistance()) ? getLeftDistance() : getRightDistance();
    }

    public static Distance getMinDistance(){
        return getLeftDistance().lte(getRightDistance()) ? getLeftDistance() : getRightDistance();
    }
}
