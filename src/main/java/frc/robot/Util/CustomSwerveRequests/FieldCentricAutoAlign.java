package frc.robot.Util.CustomSwerveRequests;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

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
     * The maximum acceleration used by the trapezoidal profile, in meters per second squared.
     * This is both the ramp-up rate and the braking rate, so it must stay at or below what the
     * drivetrain can actually achieve on carpet - if it is set too high the profile plans a stop
     * the robot cannot physically make and overshoots the target.
     */
    public double MaxAcceleration = Constants.maxLinAccel.in(MetersPerSecondPerSecond);

    /**
     * How far ahead to look when deciding where to start braking, in seconds - roughly the
     * drivetrain's closed-loop velocity response time.
     * <p>
     * The modules do not reach a commanded velocity instantly, so while braking the robot is
     * genuinely travelling faster than the profile believes, and it covers extra ground before
     * the brake command takes effect. Planning the stop against the distance the robot will have
     * at that point instead of the distance it has now cancels that out, which is what lets
     * MaxAcceleration stay high without overshooting.
     * <p>
     * Overestimating this is safe - it brakes a little early and costs a fraction of a second.
     * Underestimating leaves overshoot, so prefer the high side.
     */
    public double LagCompensation = 0.10;

    /**
     * Trapezoidal profile used to generate the translation speed toward the point.
     * Rebuilt by {@link #apply} whenever MaxVelocity or MaxAcceleration changes, since
     * TrapezoidProfile takes its constraints at construction.
     */
    private Constraints m_profileConstraints = new Constraints(MaxVelocity, MaxAcceleration);
    private TrapezoidProfile m_translationProfile = new TrapezoidProfile(m_profileConstraints);

    /** The profile goal: sitting on the target point, at rest. */
    private static final TrapezoidProfile.State kArrived = new TrapezoidProfile.State(0, 0);

    /**
     * The translation speed commanded on the previous loop, which the profile advances from.
     * This has to follow the profile's own output rather than the measured chassis speed: seeding
     * the profile from measurement makes the command a function of how fast the robot is already
     * going, which cannot start from rest (the first command would only be MaxAcceleration * dt,
     * too small to break traction loose, so the robot never moves and the command never grows).
     */
    private double m_lastCommandedSpeed = 0;

    /** Timestamp of the previous apply() call, used to measure dt and spot a fresh engagement. */
    private double m_lastApplyTimestamp = Double.NEGATIVE_INFINITY;

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

        // 2. Work out the profile timestep and whether this is a fresh engagement.
        // apply() is not called from the command loop - setControl() hands the request to the
        // native control thread - so measure the real elapsed time instead of assuming a rate.
        // An implausible gap means the request was not running last loop, so the carried-over
        // command is stale. Detecting that here stands in for the initialize hook that
        // applyRequest()'s plain run() does not give us.
        double elapsed = parameters.timestamp - m_lastApplyTimestamp;
        boolean freshStart = !(elapsed > 0.0 && elapsed < 0.1);
        m_lastApplyTimestamp = parameters.timestamp;
        double timestep = freshStart
                ? (parameters.updatePeriod > 0 ? parameters.updatePeriod : 0.020)
                : elapsed;

        // 3. Prevent division by zero and handle arrival deadband
        double vx = 0;
        double vy = 0;

        if (distanceToTarget > 0.01) { // 1cm tolerance to avoid NaN
            // Convert the distance vector into a unit vector pointing at the target.
            // TargetPoint is already alliance-mirrored (by WaypointFactory), so this speed
            // is applied directly with no further alliance-based correction.
            Translation2d unitVector = relativeVector.div(distanceToTarget);

            // Measure how fast we are currently closing on the target by projecting the robot's
            // velocity onto that unit vector. currentChassisSpeed is robot-relative, so rotate
            // it into the field frame first to match the frame TargetPoint lives in.
            ChassisSpeeds chassisSpeeds = parameters.currentChassisSpeed;
            Translation2d fieldVelocity = new Translation2d(
                    chassisSpeeds.vxMetersPerSecond,
                    chassisSpeeds.vyMetersPerSecond)
                    .rotateBy(robotPose.getRotation());
            double closingSpeed = fieldVelocity.getX() * unitVector.getX()
                    + fieldVelocity.getY() * unitVector.getY();

            // On a fresh engagement, pick the profile up from how fast the robot is genuinely
            // closing, so holding the trigger mid-sprint does not brake to restart the ramp.
            // Negative (receding) is clamped away - the profile will accelerate toward the
            // target and the drivetrain's velocity loop handles the reversal.
            if (freshStart) {
                m_lastCommandedSpeed = Math.max(0, closingSpeed);
            }

            // TrapezoidProfile fixes its constraints at construction, so pick up any changes
            // made through the builder methods (or direct field writes) here.
            if (m_profileConstraints.maxVelocity != MaxVelocity
                    || m_profileConstraints.maxAcceleration != MaxAcceleration) {
                m_profileConstraints = new Constraints(MaxVelocity, MaxAcceleration);
                m_translationProfile = new TrapezoidProfile(m_profileConstraints);
            }

            // Look LagCompensation seconds ahead of where the robot is now, so the profile plans
            // its stop against the distance it will actually have once a brake command takes
            // effect. Without this the modules' velocity-loop lag eats the stopping distance and
            // the robot overshoots (see LagCompensation). Only a closing speed counts - receding
            // must not push the braking point further out.
            double brakingDistance = Math.max(0,
                    distanceToTarget - Math.max(0, closingSpeed) * LagCompensation);

            // Run a trapezoidal profile on distance-remaining and drive it to zero. The profiled
            // coordinate is distance-to-go, so while we are closing it is shrinking and its
            // velocity is negative - hence the sign flips going in and coming out.
            //
            // Position comes from the measured pose so the braking point is always re-planned
            // against reality, but velocity comes from our own previous command (see
            // m_lastCommandedSpeed) so the ramp can bootstrap from a standstill.
            TrapezoidProfile.State next = m_translationProfile.calculate(
                    timestep,
                    new TrapezoidProfile.State(brakingDistance, -m_lastCommandedSpeed),
                    kArrived);

            m_lastCommandedSpeed = Math.max(0, -next.velocity);

            vx = unitVector.getX() * m_lastCommandedSpeed;
            vy = unitVector.getY() * m_lastCommandedSpeed;
        } else {
            // Inside the arrival deadband: hold still and do not let a stale command carry over.
            m_lastCommandedSpeed = 0;
        }

        // RotationOffset (unlike TargetPoint) is NOT alliance-mirrored upstream, so this
        // request corrects it here: add 180 degrees on Red alliance so the heading target
        // mirrors the same way the translation target already does.
        Rotation2d allianceHeadingCorrection = isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero;

        // 4. Apply to the underlying FacingAngle request
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

    public FieldCentricAutoAlign withMaxAcceleration(double acceleration) {
        this.MaxAcceleration = acceleration;
        return this;
    }

    public FieldCentricAutoAlign withMaxAcceleration(LinearAcceleration acceleration) {
        this.MaxAcceleration = acceleration.in(MetersPerSecondPerSecond);
        return this;
    }

    public FieldCentricAutoAlign withLagCompensation(double seconds) {
        this.LagCompensation = seconds;
        return this;
    }

    public FieldCentricAutoAlign withLagCompensation(Time lag) {
        this.LagCompensation = lag.in(Seconds);
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