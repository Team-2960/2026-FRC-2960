package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Revolution;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.Util.AprilTagPipelineSettings;
import frc.robot.Util.ShotSpeedTable;
import frc.robot.generated.TunerConstants;

public class Constants {
    // Robot Constants
    public static final LinearVelocity maxLinVel = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity maxAngVel = RotationsPerSecond.of(2); 
    public static final LinearVelocity slowdownLinVel = maxLinVel.div(2);
    public static final AngularVelocity slowdownAngVel = RotationsPerSecond.of(2);

    public static final CANBus rioBus = CANBus.roboRIO();
    public static final CANBus canivoreBus = new CANBus("canivore");

    // Motor IDs
    public static final int shooterMotorLeaderID = 31;
    public static final int shooterMotorFollowerID = 32;
    public static final int IntakeMotorID = 19;
    public static final int IndexMotorID = 41;

    // Shooter Constants
    public static final Rotation2d shooterOrientation = Rotation2d.fromDegrees(180);
    public static final Distance shootingDistance = Inches.of(92);

    public static final AngularVelocity shooterWheelTol = Rotations.per(Minute).of(100);
    public static final Angle shooterHoodTol = Degrees.of(2);
    public static final Angle shotAngleTol = Degrees.of(2);

    public static final ShotSpeedTable shooterWheelTable = new ShotSpeedTable()
            .addEntry(Meters.of(1.7369), Revolution.per(Minute).of(1700))
            .addEntry(Meters.of(2.1828), Revolution.per(Minute).of(1800))
            .addEntry(Meters.of(2.5179), Revolution.per(Minute).of(1850))
            .addEntry(Meters.of(3.1897), Revolution.per(Minute).of(2000))
            .addEntry(Meters.of(3.6278), Revolution.per(Minute).of(2050));


    public static final AngularVelocity shootVelocity = Rotations.per(Minute).of(1900);
    public static final AngularAcceleration shooterMaxAccel = Rotations.per(Minute).per(Second).of(6209);

    // Indexer Constants
    public static final Voltage indexerFeedVolt = Volts.of(12);
    public static final AngularAcceleration indexerMaxAccel = Rotations.per(Minute).per(Second).of(6209);

    // Intake Constants
    public static final double intakeGearRatio = 1;

    public static final Voltage intakeInVolt = Volts.of(12.0);
    public static final Voltage intakeOutVolt = Volts.of(-12.0);

    public static final Angle intakeDownAngle = Degrees.of(0);
    public static final Angle intakeUpAngle = Degrees.of(90);
    public static final Angle intakeAutoRetractLimit = Degrees.of(80);
    public static final AngularVelocity intakeAutoRetractVel = RotationsPerSecond.of(.25);

    // Drivetrain Constants
    public static final LinearVelocity linDeadband = maxLinVel.times(.07); 
    public static final AngularVelocity angDeadband = maxAngVel.times(.07
    );

    // Camera Constants
    public static final Transform3d leftCameraOffsets = new Transform3d(
        Inches.of(-29/2).plus(Millimeters.of(37.492)), 
        Inches.of(29/2).minus(Millimeters.of(55.321)), 
        Inches.of(5.375), 
        new Rotation3d(0, Math.toRadians(-30), Math.toRadians(-135))
    );

    public static final Transform3d rightCameraOffsets = new Transform3d(
        Inches.of(-29/2).plus(Millimeters.of(37.492)), 
        Inches.of(-29/2).plus(Millimeters.of(55.321)), 
        Inches.of(5.375), 
        //new Rotation3d(37.8, 37.8, -45)
        new Rotation3d(0, Math.toRadians(-30), Math.toRadians(135))
    );

    public static final Vector<N3> singleStds = VecBuilder.fill(4, 4, 16);
    public static final Vector<N3> multiStds = VecBuilder.fill(0.5, 0.5, 1);

    public static final AprilTagPipelineSettings leftCameraSettings = new AprilTagPipelineSettings(AprilTagFields.k2026RebuiltWelded,
        Constants.leftCameraOffsets,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        3, 
        singleStds , 
        multiStds,
        .2
    );

    public static final AprilTagPipelineSettings rightCameraSettings = new AprilTagPipelineSettings(AprilTagFields.k2026RebuiltWelded,
        Constants.rightCameraOffsets,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        3, 
        singleStds , 
        multiStds,
        .2
    );


    //Distance Constants
    public static final Distance maxRobotTrenchDistance = Meters.of(2);
}
