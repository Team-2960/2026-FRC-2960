package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Revolution;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Util.AprilTagPipelineSettings;
import frc.robot.Util.ShotSpeedTable;
import frc.robot.generated.TunerConstants;

public class Constants {
    // Robot Constants
    public static final LinearVelocity maxLinVel = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity maxAngVel = RotationsPerSecond.of(2); 
    public static final AngularAcceleration maxAngAccel = DegreesPerSecondPerSecond.of(1292);
    public static final LinearAcceleration maxLinAccel = MetersPerSecondPerSecond.of(5.9);
    public static final LinearVelocity slowdownLinVel = maxLinVel.div(2);
    public static final AngularVelocity slowdownAngVel = RotationsPerSecond.of(2);
    public static final Distance linRampDownDist = Meters.of(1.2);

    public static final Current krakenX60CurrentLimit = Amps.of(80);

    public static final CANBus rioBus = CANBus.roboRIO();
    public static final CANBus canivoreBus = new CANBus("canivore");

    // Motor IDs
    public static final int shooterMotorLeaderID = 20;
    public static final int shooterMotorFollowerID = 23;
    public static final int shooterHoodMotor = 18;
    public static final int intakeMotorID = 19;
    public static final int intakeAngleID = 14;
    public static final int indexMotorID = 22;
    public static final int leftClimbMotorID = 17;
    public static final int rightClimbMotorID = 15;

    //Other CAN IDs
    public static final int leftLaserCanID = 20;
    public static final int rightLaserCanID = 21;
    public static final int intakeAngleEncoderID = 16;
    public static final int shooterHoodEncoderID = 21;

    // Shooter Constants
    public static final Rotation2d shooterOrientation = Rotation2d.fromDegrees(180);
    public static final Distance shootingDistance = Inches.of(92);

    public static final AngularVelocity shooterWheelTol = Rotations.per(Minute).of(100);
    public static final Angle shooterHoodTol = Degrees.of(2);
    public static final Angle shotAngleTol = Degrees.of(2);

    public static final ShotSpeedTable shooterWheelTable = new ShotSpeedTable()
            .addEntry(Meters.of(2.058), Revolution.per(Minute).of(1535))
            .addEntry(Meters.of(2.59), Revolution.per(Minute).of(1620))
            .addEntry(Meters.of(2.72), Revolution.per(Minute).of(1750))
            .addEntry(Meters.of(3.13), Revolution.per(Minute).of(1820))
            .addEntry(Meters.of(3.4), Revolution.per(Minute).of(2020))
            .addEntry(Meters.of(3.702), Revolution.per(Minute).of(2220));


    public static final AngularVelocity shootVelocity = Rotations.per(Minute).of(1900);
    public static final AngularVelocity passVelocity = Rotations.per(Minute).of(1500);
    public static final AngularVelocity idleVelocity = Rotations.per(Minute).of(1400);
    public static final AngularAcceleration shooterMaxAccel = Rotations.per(Minute).per(Second).of(6209);

    public static final double shooterWheelGearRatio = 20.0/12.0;
    public static final double shooterHoodGearRatio = 29.0 + (1.0/3.0);

    // Indexer Constants
    public static final Voltage indexerForwardVolt = Volts.of(12);
    public static final Voltage indexerReverseVolt = Volts.of(-12);
    public static final AngularAcceleration indexerMaxAccel = Rotations.per(Minute).per(Second).of(6209);

    public static final double indexerGearRatio = 5.0;

    // Intake Constants
    public static final Angle intakeOutAngle = Degrees.of(0);
    public static final Angle intakeInAngle = Degrees.of(105);

    public static final Voltage intakeRollerInVolt = Volts.of(12.0);
    public static final Voltage intakeRollerOutVolt = Volts.of(-12.0);

    public static final double intakeRollerGearRatio = 24.0 / 18.0;
    public static final double intakeAngleGearRatio = 50.0;

    // Drivetrain Constants
    public static final LinearVelocity linDeadband = maxLinVel.times(.07); 
    public static final AngularVelocity angDeadband = maxAngVel.times(.07);

    // Camera Constants
    public static final Transform3d leftCameraOffsets = new Transform3d(
        Inches.of(-11.274), 
        Inches.of(13.476), 
        Inches.of(7.553), 
        new Rotation3d(0, Math.toRadians(-40), Math.toRadians(-130))
    );

    public static final Transform3d rightCameraOffsets = new Transform3d(
         Inches.of(-11.274), 
        Inches.of(-13.476), 
        Inches.of(7.553), 
        //new Rotation3d(37.8, 37.8, -45)
        new Rotation3d(0, Math.toRadians(-40), Math.toRadians(130))
    );

    public static final Transform3d sideCameraOffsets = new Transform3d(
         Inches.of(-9.074),    
        Inches.of(-15.228), 
        Inches.of(12.205), 
        //new Rotation3d(37.8, 37.8, -45)
        new Rotation3d(0, Math.toRadians(-40), Math.toRadians(130))
    );

    public static final Vector<N3> singleStds = VecBuilder.fill(4, 4, 16);
    public static final Vector<N3> multiStds = VecBuilder.fill(0.5, 0.5, 1);

    public static final AprilTagPipelineSettings leftCameraSettings = new AprilTagPipelineSettings(AprilTagFields.k2026RebuiltWelded,
        Constants.leftCameraOffsets,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        4, 
        singleStds , 
        multiStds,
        .2
    );

    public static final AprilTagPipelineSettings rightCameraSettings = new AprilTagPipelineSettings(AprilTagFields.k2026RebuiltWelded,
        Constants.rightCameraOffsets,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        4, 
        singleStds , 
        multiStds,
        .2
    );

    public static final AprilTagPipelineSettings sideCameraSettings = new AprilTagPipelineSettings(AprilTagFields.k2026RebuiltWelded,
        Constants.sideCameraOffsets,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        4, 
        singleStds , 
        multiStds,
        .2
    );


    //Distance Constants
    public static final Distance maxRobotTrenchDistance = Meters.of(2);
    public static final Distance autonPointToPointTolerance = Meters.of(0.1);
}
