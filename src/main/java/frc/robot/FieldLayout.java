package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Util.GeomUtil;

public class FieldLayout {
    public enum FieldSide{
        BLUELEFT,
        BLUERIGHT,
        REDLEFT,
        REDRIGHT
    }

    public static final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Overall Field Dimensions
    public static final Distance fieldSizeX = Inches.of(651.22);
    public static final Distance fieldSizeY = Inches.of(317.69);

    public static final Distance fieldCenterX = fieldSizeX.div(2.0);
    public static final Distance fieldCenterY = fieldSizeY.div(2.0);

    public static final Angle blueForwardAngle = Degrees.of(0);
    public static final Angle redForwardAngle = Degrees.of(180);

    public static final Distance fieldWidth = Meters.of(field.getFieldWidth());
    public static final Distance fieldLength = Meters.of(field.getFieldLength());
    // Start Line Dimensions
    public static final Distance startLineToWallOffset = Inches.of(156.61);
    public static final Distance startLineWidth = Inches.of(2);
    
    public static final Translation2d bluePassingRight = new Translation2d(Feet.of(6), Feet.of(6));
    public static final Translation2d bluePassingLeft = new Translation2d(Feet.of(6), fieldSizeY.minus(Feet.of(6)));

    public static final Translation2d redPassingRight = new Translation2d(fieldSizeX.minus(Feet.of(6)), fieldSizeY.minus(Feet.of(6)));
    public static final Translation2d redPassingLeft = new Translation2d(fieldSizeX.minus(Feet.of(6)), Feet.of(6));

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

        public static final Translation2d blueHubCenter = new Translation2d(blueHubCenterX, fieldCenterY);
        public static final Translation2d redHubCenter = new Translation2d(redHubCenterX, fieldCenterY);

        public static final Translation2d blueHubCenterFront = new Translation2d(blueHubFront, fieldCenterY);
        public static final Translation2d redHubCenterFront = new Translation2d(redHubFront, fieldCenterY);

        public static final Translation2d blueHubCenterBack = blueHubCenter.plus(new Translation2d(hubBaseWidth.div(2), Meters.zero()));
        public static final Translation2d redHubCenterBack = redHubCenter.minus(new Translation2d(hubBaseWidth.div(2), Meters.zero()));

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
         * Gets the back center of the hub for the current alliance
         * 
         * @return back center of the hub for the current alliance. Defaults to blue if
         *         alliance is not set.
         */
        public static Translation2d getHubCenterBack() {
            return isRedAlliance() ? Hub.redHubCenterBack : Hub.blueHubCenterBack;
        }

        /**
         * Gets the back center of the hub for the current alliance
         * 
         * @param offset is relative to the blue alliance, automatically flipped when applied to Red
         * @return back center of the hub for the current alliance. Defaults to blue if
         *         alliance is not set.
         */
        public static Translation2d getHubCenterBack(Translation2d offset) {
            return isRedAlliance() ? Hub.redHubCenterBack.minus(offset) : Hub.blueHubCenterBack.plus(offset);
        }

        /**
         * Calculates the distance to the current alliance hub from a given position
         * @param position  position to check to distance of
         * @return  distance to the current alliance hub 
         */
        public static Distance getHubDist(Translation2d position) {
            return Meters.of(position.getDistance(getHubCenter()));
        }

        public static Pose2d getHubBackAlign(Pose2d curPose){
            Translation2d translation = getHubCenterBack(new Translation2d(Inches.of(35.0/2.0 + 10), Meters.zero()));
            return new Pose2d(translation, getInwardAngle(curPose));
        }
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
            return Meters.of(position.getDistance(getTowerCenter()));
        }
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

            if (isRedAlliance()){
                if (curPose.getY() < fieldCenterY.in(Meters)){
                    return new Pose2d(redTrenchLeft, getInwardAngle(curPose));
                }else{
                    return new Pose2d(redTrenchRight, getInwardAngle(curPose));
                }
            }else{
                if (curPose.getY() < fieldCenterY.in(Meters)){
                    return new Pose2d(blueTrenchRight, getInwardAngle(curPose));
                }else{
                    return new Pose2d(blueTrenchLeft, getInwardAngle(curPose));
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
        return alliance.isPresent() && alliance.get().equals(DriverStation.Alliance.Red);
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

    public static Pose2d getFeedPosition(Supplier<Pose2d> curPos){
        FieldSide fieldSide = getFieldSide(curPos);

        switch (fieldSide) {
            case BLUELEFT:
                return GeomUtil.toPose2d(bluePassingLeft);
            case BLUERIGHT:
                return GeomUtil.toPose2d(bluePassingRight);
            case REDLEFT:
                return GeomUtil.toPose2d(redPassingLeft);
            default:
                return GeomUtil.toPose2d(redPassingRight);
        }
    }

    public static FieldSide getFieldSide(Supplier<Pose2d> curPos){
        Pose2d pose = curPos.get();
        if (isRedAlliance()){
                if (pose.getY() < fieldCenterY.in(Meters)){
                    return FieldSide.REDLEFT;
                }else{
                    return FieldSide.REDRIGHT;
                }
            }else{
                if (pose.getY() < fieldCenterY.in(Meters)){
                    return FieldSide.BLUERIGHT;
                }else{
                    return FieldSide.BLUELEFT;
                }
        }
    }

    public static Rotation2d getInwardAngle(Pose2d curPose){
        if (isRedAlliance()){
                if (curPose.getY() < fieldCenterY.in(Meters)){
                    return Rotation2d.fromDegrees(-90);
                }else{
                    return Rotation2d.fromDegrees(90);
                }
            }else{
                if (curPose.getY() < fieldCenterY.in(Meters)){
                    return Rotation2d.fromDegrees(90);
                }else{
                    return Rotation2d.fromDegrees(-90);
                }
        }
    }



}
