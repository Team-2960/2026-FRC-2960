package frc.robot.Util.CustomSwerveRequests;

import static edu.wpi.first.units.Units.MetersPerSecond;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldCentricAutoAlign implements SwerveRequest {   
    
    /**
     * The desired velocity to travel along the circle created around the orbital point using the radius.
     * The travel velocity is eventually split into an X and Y Velocity to feed into the FieldCentric Request.
     */
    public double MaxVelocity = 3.5;

    /**
     * The target point for the robot to move to.
     */
    public Translation2d TargetPoint = new Translation2d();

    /**
     * The PID Controller used to calculate the translation speed toward the point.
     * kP should be tuned so the robot slows down as it approaches the target.
     */
    public PIDController TranslationPID = new PIDController(0, 0, 0);

    /**
     * The desired direction to face while moving to the point.
     */
    public Rotation2d TargetDirection = new Rotation2d();

    /**
     * Offset applied to the rotation.
     */
    public Rotation2d RotationOffset = new Rotation2d();

    public double TargetRateFeedforward = 0;
    public double Deadband = 0;
    public double RotationalDeadband = 0;
    public double MaxAbsRotationalRate = 0;
    public Translation2d CenterOfRotation = new Translation2d();

    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.Velocity;
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
    public boolean DesaturateWheelSpeeds = true;

    /**
     * PID used to maintain the desired heading
     */
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    private final FieldCentricFacingAngle m_fieldCentricFacingAngle = new FieldCentricFacingAngle();

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    public FieldCentricAutoAlign() {
        HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        Pose2d robotPose = parameters.currentPose;

        // 1. Calculate vector to target
        Translation2d relativeVector = TargetPoint.minus(robotPose.getTranslation());
        double distanceToTarget = relativeVector.getNorm();

        // 2. Prevent division by zero and handle arrival deadband
        double vx = 0;
        double vy = 0;

        if (distanceToTarget > 0.01) { // 1cm tolerance to avoid NaN
            // Calculate speed toward the point using PID
            // The setpoint is 0 (we want distance to target to be 0)
            // TargetPoint is already alliance-mirrored (by WaypointFactory), so this speed
            // is applied directly with no further alliance-based correction.
            double translationMag = Math.abs(TranslationPID.calculate(distanceToTarget, 0));

            //Limit to MaxVelocity
            if (Math.abs(translationMag) > MaxVelocity && MaxVelocity > 0) {
                translationMag = Math.copySign(MaxVelocity, translationMag);
            }

            // Convert distance vector into a unit vector and multiply by calculated velocity
            Translation2d unitVector = relativeVector.div(distanceToTarget);
            vx = unitVector.getX() * translationMag;
            vy = unitVector.getY() * translationMag;
        }

        // RotationOffset (unlike TargetPoint) is NOT alliance-mirrored upstream, so this
        // request corrects it here: add 180 degrees on Red alliance so the heading target
        // mirrors the same way the translation target already does.
        Rotation2d allianceHeadingCorrection = isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero;

        // 3. Apply to the underlying FacingAngle request
        return m_fieldCentricFacingAngle
                .withCenterOfRotation(CenterOfRotation)
                .withDeadband(Deadband)
                .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
                .withDriveRequestType(DriveRequestType)
                .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
                .withHeadingPID(HeadingController.getP(), HeadingController.getI(), HeadingController.getD())
                .withMaxAbsRotationalRate(MaxAbsRotationalRate)
                .withRotationalDeadband(RotationalDeadband)
                .withSteerRequestType(SteerRequestType)
                .withTargetDirection(TargetDirection.plus(RotationOffset).plus(allianceHeadingCorrection))
                .withTargetRateFeedforward(TargetRateFeedforward)
                .withVelocityX(vx)
                .withVelocityY(vy)
                .apply(parameters, modulesToApply);
    }

    /* Builder Methods */

    public FieldCentricAutoAlign withTargetPoint(Translation2d point) {
        this.TargetPoint = point;
        return this;
    }

    public FieldCentricAutoAlign withTranslationPID(double kP, double kI, double kD) {
        this.TranslationPID.setPID(kP, kI, kD);
        return this;
    }

    public FieldCentricAutoAlign withMaxVelocity(double velocity) {
        this.MaxVelocity = velocity;
        return this;
    }

    public FieldCentricAutoAlign withMaxVelocity(LinearVelocity velocity) {
        this.MaxVelocity = velocity.in(MetersPerSecond);
        return this;
    }

    public FieldCentricAutoAlign withHeadingPID(double kP, double kI, double kD) {
        this.HeadingController.setPID(kP, kI, kD);
        return this;
    }

    public FieldCentricAutoAlign withTargetDirection(Rotation2d direction) {
        this.TargetDirection = direction;
        return this;
    }

    public FieldCentricAutoAlign withRotationalOffset(Rotation2d offset) {
        this.RotationOffset = offset;
        return this;
    }

    // ... (Keep existing withDeadband, withForwardPerspective, etc. from your original class)
}