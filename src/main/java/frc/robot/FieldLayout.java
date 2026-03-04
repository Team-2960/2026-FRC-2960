package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldLayout {
    public static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Overall Field Dimensions
    public static final Distance fieldSizeX = Inches.of(651.22);
    public static final Distance fieldSizeY = Inches.of(317.69);

    public static final Distance fieldCenterX = fieldSizeX.div(2);
    public static final Distance fieldCenterY = fieldSizeY.div(2);

    public static final Angle blueForwardAngle = Degrees.of(0);
    public static final Angle redForwardAngle = Degrees.of(180);

    public static final Distance fieldWidth = Meters.of(field.getFieldWidth());
    public static final Distance fieldLength = Meters.of(field.getFieldLength());
    // Start Line Dimensions
    public static final Distance startLineToWallOffset = Inches.of(156.61);
    public static final Distance startLineWidth = Inches.of(2);

    // Hub Dimensions
    public static class Hub{
        public static final Distance hubXToCenterOffset = Inches.of(143.5);
        public static final Distance hubBaseWidth = Inches.of(47);
        public static final Distance hubFunnelDiam = Inches.of(41.5);

        public static final Distance blueHubCenterX = fieldCenterX.minus(hubXToCenterOffset);
        public static final Distance redHubCenterX = fieldCenterX.plus(hubXToCenterOffset);

        public static final Distance blueHubLeftY = fieldCenterY.plus(hubBaseWidth.div(2));
        public static final Distance blueHubRightY = fieldCenterY.minus(hubBaseWidth.div(2));

        public static final Distance redHubLeftY = blueHubRightY;
        public static final Distance redHubRightY = blueHubLeftY;

        public static final Distance blueHubFront = startLineToWallOffset.plus(startLineWidth);
        public static final Distance redHubFront = fieldSizeX.minus(blueHubFront);

        public static final Translation2d blueHubCenter = new Translation2d(fieldWidth.div(2), fieldCenterY);
        public static final Translation2d redHubCenter = new Translation2d(fieldWidth.div(2), fieldCenterY);

        public static final Translation2d blueHubCenterFront = new Translation2d(blueHubFront, fieldCenterY);
        public static final Translation2d redHubCenterFront = new Translation2d(redHubFront, fieldCenterY);
    }

    //Tower Dimensions
    public static class Tower{
        public static final Distance towerXToCenterOffset = Inches.of(300.985);
        public static final Distance towerYToCenterOffset = Inches.of(11.38);
        public static final Distance towerBaseWidth = Inches.of(45);

        public static final Distance blueTowerCenterX = fieldCenterX.minus(towerXToCenterOffset);
        public static final Distance redTowerCenterX = fieldCenterX.plus(towerXToCenterOffset);

        public static final Distance blueTowerYOffset = fieldCenterY.minus(towerYToCenterOffset);

        public static final Distance redTowerYOffset = fieldCenterY.plus(towerYToCenterOffset);

        public static final Distance blueTowerFront = startLineToWallOffset.plus(startLineWidth);
        public static final Distance redTowerFront = fieldSizeX.minus(blueTowerFront);
        
        public static final Translation2d blueTowerCenter = new Translation2d(blueTowerCenterX, blueTowerYOffset);
        public static final Translation2d redTowerCenter = new Translation2d(redTowerCenterX, redTowerYOffset);

        public static final Translation2d blueTowerCenterFront = new Translation2d(blueTowerFront, blueTowerYOffset);
        public static final Translation2d redTowerCenterFront = new Translation2d(redTowerFront, redTowerYOffset);
    }
    //Trench Dimensions
    public static class Trench{
        public static final Distance trenchOpeningWidth = Inches.of(50.34);
        public static final Distance trenchDivider = Inches.of(12);
        public static final Distance trenchOpeningCenterOffset = Hub.hubBaseWidth.div(2)
            .plus(Bump.bumpWidth)
                .plus(trenchDivider)
                .plus(trenchOpeningWidth.div(2));

        public static final Translation2d blueTrenchRight = Hub.blueHubCenter.minus(
            new Translation2d(Meters.of(0), 
            trenchOpeningCenterOffset));

        public static final Translation2d blueTrenchLeft =  Hub.blueHubCenter.plus(
            new Translation2d(Meters.of(0), 
            trenchOpeningCenterOffset));
        
        public static final Translation2d redTrenchRight = Hub.redHubCenter.plus(
            new Translation2d(Meters.of(0), 
            trenchOpeningCenterOffset));
        
        public static final Translation2d redTrenchLeft =  Hub.redHubCenter.minus(
            new Translation2d(Meters.of(0), 
            trenchOpeningCenterOffset));


        public static Pose2d getNearestAllianceTrench(Pose2d curPose){

            Alliance curAlliance = Alliance.Blue;
            if (DriverStation.getAlliance().isPresent()){
                curAlliance = DriverStation.getAlliance().get();
            }

            if (curAlliance.equals(Alliance.Red)){
                if (curPose.getY() < fieldCenterY.in(Meters)){
                    return new Pose2d(redTrenchLeft, Rotation2d.fromDegrees(-90));
                }else{
                    return new Pose2d(redTrenchRight, Rotation2d.fromDegrees(90));
                }
            }else{
                if (curPose.getY() < fieldCenterY.in(Meters)){
                    return new Pose2d(blueTrenchRight, Rotation2d.fromDegrees(90));
                }else{
                    return new Pose2d(blueTrenchLeft, Rotation2d.fromDegrees(-90));
                }
            }
        }
    }

    public static class Bump{
        public static final Distance bumpWidth = Inches.of(73);
    }
    /**
     * Checks if the current alliance is red.
     * 
     * @return True if the current alliance is Red. False if alliance is Blue or the
     *         alliance is not set
     */
    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    /**
     * Gets the field orientation that is considered forward for the current
     * alliance.
     * 
     * @return forward angle for the current alliance. Defaults to blue if alliance
     *         is not set.
     */
    public static Angle getForwardAngle() {
        return isRedAlliance() ? redForwardAngle : blueForwardAngle;
    }

    /**
     * Gets the center of the hub for the current alliance
     * 
     * @return center of the hub for the current alliance. Defaults to blue if
     *         alliance is not set.
     */
    public static Translation2d getHubCenter() {
        return isRedAlliance() ? Hub.redHubCenter : Hub.blueHubCenter;
    }

    /**
     * Gets the front center of the hub for the current alliance
     * 
     * @return front center of the hub for the current alliance. Defaults to blue if
     *         alliance is not set.
     */
    public static Translation2d getHubCenterFront() {
        return isRedAlliance() ? Hub.redHubCenterFront : Hub.blueHubCenterFront;
    }

    /**
     * Calculates the distance to the current alliance hub from a given position
     * @param position  position to check to distance of
     * @return  distance to the current alliance hub 
     */
    public static Distance getHubDist(Translation2d position) {
        return Meters.of(position.getDistance(FieldLayout.getHubCenter()));
    }

        public static Translation2d getTowerCenter() {
        return isRedAlliance() ? Tower.redTowerCenter : Tower.blueTowerCenter;
    }

    /**
     * Gets the front center of the tower for the current alliance
     * 
     * @return front center of the tower for the current alliance. Defaults to blue if
     *         alliance is not set.
     */
    public static Translation2d getTowerCenterFront() {
        return isRedAlliance() ? Tower.redTowerCenterFront : Tower.blueTowerCenterFront;
    }

    /**
     * Calculates the distance to the current alliance tower from a given position
     * @param position  position to check to distance of
     * @return  distance to the current alliance tower
     */
    public static Distance getTowerDist(Translation2d position) {
        return Meters.of(position.getDistance(FieldLayout.getTowerCenter()));
    }
}
