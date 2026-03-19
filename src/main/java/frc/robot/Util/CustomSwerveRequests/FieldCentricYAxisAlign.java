package frc.robot.Util.CustomSwerveRequests;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

public class FieldCentricYAxisAlign implements SwerveRequest{

    public double TravelVelocityY = 0;

    public double XCoordinate = 0;

     /**
     * The desired direction to face.
     * 0 Degrees is defined as in the direction of the X axis.
     * As a result, a TargetDirection of 90 degrees will point along
     * the Y axis, or to the left.
     */
    public Rotation2d TargetDirection = new Rotation2d();
    /**
     * The rotational rate feedforward to add to the output of the heading
     * controller, in radians per second. When using a motion profile for the
     * target direction, this can be set to the current velocity reference of
     * the profile.
     */
    public double TargetRateFeedforward = 0;

    /**
     * The allowable deadband of the request, in m/s.
     */
    public double Deadband = 0;
    /**
     * The rotational deadband of the request, in radians per second.
     */
    public double RotationalDeadband = 0;
    /**
     * The maximum absolute rotational rate to allow, in radians per second.
     * Setting this to 0 results in no cap to rotational rate.
     */
    public double MaxAbsRotationalRate = 0;
    /**
     * The center of rotation the robot should rotate around.
     * This is (0,0) by default, which will rotate around the center of the robot.
     */
    public Translation2d CenterOfRotation = new Translation2d();
    /**
     * The type of control request to use for the drive motor.
     */
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    /**
     * The type of control request to use for the steer motor.
     */
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.Position;
    /**
     * Whether to desaturate wheel speeds before applying.
     * For more information, see the documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     */
    public boolean DesaturateWheelSpeeds = true;

    /**
     * The perspective to use when determining which direction is forward.
     */
    public ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;

    /**
     * The PID controller used to maintain the desired heading.
     * Users can specify the PID gains to change how aggressively to maintain
     * heading.
     * <p>
     * This PID controller operates on heading radians and outputs a target
     * rotational rate in radians per second. Note that continuous input should
     * be enabled on the range [-pi, pi].
     */
    public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

    public PhoenixPIDController YAxisCorrectionController = new PhoenixPIDController(0, 0, 0);
    public Constraints profileConstraints = new Constraints(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), Constants.maxLinAccel.in(MetersPerSecondPerSecond));
    public ProfiledPIDController YAxisProfiledController = new ProfiledPIDController(0, 0, 0, profileConstraints);

    private final FieldCentricFacingAngle m_fieldCentricFacingAngle = new FieldCentricFacingAngle();

    public FieldCentricYAxisAlign(){
        HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }


    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        YAxisProfiledController.setConstraints(profileConstraints);
       double xPose = parameters.currentPose.getX();
       double allianceFlip = 1;
       if (DriverStation.getAlliance().isPresent()){
            allianceFlip = DriverStation.getAlliance().get().equals(Alliance.Red) ? -1 : 1;
       }
       //double vy = allianceFlip * YAxisCorrectionController.calculate(yPose, XCoordinate, parameters.timestamp);
        //double vy = allianceFlip * YAxisProfiledController.calculate(yPose, XCoordinate);
        //Pose2d targetPoint = FieldLayout.Trench.getNearestAllianceTrench(parameters.currentPose);
        double vx = allianceFlip * calcToPoint(xPose, XCoordinate);

       return m_fieldCentricFacingAngle
        .withCenterOfRotation(CenterOfRotation)
        .withDeadband(Deadband)
        .withDesaturateWheelSpeeds(DesaturateWheelSpeeds)
        .withDriveRequestType(DriveRequestType)
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
        .withHeadingPID(HeadingController.getP(), HeadingController.getI(), HeadingController.getD())
        .withMaxAbsRotationalRate(MaxAbsRotationalRate)
        .withRotationalDeadband(RotationalDeadband)
        .withSteerRequestType(SteerRequestType)
        .withTargetDirection(TargetDirection)
        .withTargetRateFeedforward(TargetRateFeedforward)
        .withVelocityX(vx)
        .withVelocityY(TravelVelocityY)
        .apply(parameters, modulesToApply);
    }

/**
     * Modifies the PID gains of the HeadingController parameter and returns itself.
     * <p>
     * Sets the proportional, integral, and differential coefficients used to maintain
     * the desired heading. Users can specify the PID gains to change how aggressively to
     * maintain heading.
     * <p>
     * This PID controller operates on heading radians and outputs a target
     * rotational rate in radians per second.
     *
     * @param kP The proportional coefficient; must be >= 0
     * @param kI The integral coefficient; must be >= 0
     * @param kD The differential coefficient; must be >= 0
     * @return this object
     */
    public FieldCentricYAxisAlign withHeadingPID(double kP, double kI, double kD)
    {
        this.HeadingController.setPID(kP, kI, kD);
        return this;
    }

    /**
     * Changes the velocity of the robot's travel along the circle.
     * @param newVelocity The desired velocity for the robot to travel at in Meters Per Second.
     * @return this object
     */
    public FieldCentricYAxisAlign withTravelVelocity(double newVelocity){
        this.TravelVelocityY = newVelocity;
        return this;
    }

    /**
     * Changes the velocity of the robot's travel along the circle.
     * 
     * @param newVelocity
     * @return
     */
    public FieldCentricYAxisAlign withTravelVelocity(LinearVelocity newVelocity){
        this.TravelVelocityY = newVelocity.in(MetersPerSecond);
        return this;
    }

    public FieldCentricYAxisAlign withYAxisCorrectionPID(double kP, double kI, double kD){
        //YAxisCorrectionController.setPID(kP, kI, kD);
        YAxisProfiledController.setPID(kP, kI, kD);
        return this;
    }

    public FieldCentricYAxisAlign withYAxisCoordinate(double coordinate){
        this.XCoordinate = coordinate;
        return this;
    }
    
    public FieldCentricYAxisAlign withYAxisCoordinate(Translation2d coordinate){
        this.XCoordinate = coordinate.getX();
        return this;
    }

/**
     * Modifies the TargetDirection parameter and returns itself.
     * <p>
     * The desired direction to face. 0 Degrees is defined as in the direction of
     * the X axis. As a result, a TargetDirection of 90 degrees will point along
     * the Y axis, or to the left.
     *
     * @param newTargetDirection Parameter to modify
     * @return this object
     */
    public FieldCentricYAxisAlign withTargetDirection(Rotation2d newTargetDirection) {
        this.TargetDirection = newTargetDirection;
        return this;
    }

    /**
     * Modifies the TargetRateFeedforward parameter and returns itself.
     * <p>
     * The rotational rate feedforward to add to the output of the heading
     * controller, in radians per second. When using a motion profile for the
     * target direction, this can be set to the current velocity reference of
     * the profile.
     *
     * @param newTargetRateFeedforward Parameter to modify
     * @return this object
     */
    public FieldCentricYAxisAlign withTargetRateFeedforward(double newTargetRateFeedforward) {
        this.TargetRateFeedforward = newTargetRateFeedforward;
        return this;
    }
    /**
     * Modifies the TargetRateFeedforward parameter and returns itself.
     * <p>
     * The rotational rate feedforward to add to the output of the heading
     * controller, in radians per second. When using a motion profile for the
     * target direction, this can be set to the current velocity reference of
     * the profile.
     *
     * @param newTargetRateFeedforward Parameter to modify
     * @return this object
     */
    public FieldCentricYAxisAlign withTargetRateFeedforward(AngularVelocity newTargetRateFeedforward) {
        this.TargetRateFeedforward = newTargetRateFeedforward.in(RadiansPerSecond);
        return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     * <p>
     * The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricYAxisAlign withDeadband(double newDeadband) {
        this.Deadband = newDeadband;
        return this;
    }

    /**
     * Modifies the Deadband parameter and returns itself.
     * <p>
     * The allowable deadband of the request, in m/s.
     *
     * @param newDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricYAxisAlign withDeadband(LinearVelocity newDeadband) {
        this.Deadband = newDeadband.in(MetersPerSecond);
        return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     * <p>
     * The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricYAxisAlign withRotationalDeadband(double newRotationalDeadband) {
        this.RotationalDeadband = newRotationalDeadband;
        return this;
    }

    /**
     * Modifies the RotationalDeadband parameter and returns itself.
     * <p>
     * The rotational deadband of the request, in radians per second.
     *
     * @param newRotationalDeadband Parameter to modify
     * @return this object
     */
    public FieldCentricYAxisAlign withRotationalDeadband(AngularVelocity newRotationalDeadband) {
        this.RotationalDeadband = newRotationalDeadband.in(RadiansPerSecond);
        return this;
    }

    /**
     * Modifies the MaxAbsRotationalRate parameter and returns itself.
     * <p>
     * The maximum absolute rotational rate to allow, in radians per second.
     * Setting this to 0 results in no cap to rotational rate.
     *
     * @param newMaxAbsRotationalRate Parameter to modify
     * @return this object
     */
    public FieldCentricYAxisAlign withMaxAbsRotationalRate(double newMaxAbsRotationalRate) {
        this.MaxAbsRotationalRate = newMaxAbsRotationalRate;
        return this;
    }

    /**
     * Modifies the MaxAbsRotationalRate parameter and returns itself.
     * <p>
     * The maximum absolute rotational rate to allow, in radians per second.
     * Setting this to 0 results in no cap to rotational rate.
     *
     * @param newMaxAbsRotationalRate Parameter to modify
     * @return this object
     */
    public FieldCentricYAxisAlign withMaxAbsRotationalRate(AngularVelocity newMaxAbsRotationalRate) {
        this.MaxAbsRotationalRate = newMaxAbsRotationalRate.in(RadiansPerSecond);
        return this;
    }

    /**
     * Modifies the CenterOfRotation parameter and returns itself.
     * <p>
     * The center of rotation the robot should rotate around. This is (0,0) by
     * default, which will rotate around the center of the robot.
     *
     * @param newCenterOfRotation Parameter to modify
     * @return this object
     */
    public FieldCentricYAxisAlign withCenterOfRotation(Translation2d newCenterOfRotation) {
        this.CenterOfRotation = newCenterOfRotation;
        return this;
    }

    /**
     * Modifies the DriveRequestType parameter and returns itself.
     * <p>
     * The type of control request to use for the drive motor.
     *
     * @param newDriveRequestType Parameter to modify
     * @return this object
     */
    public FieldCentricYAxisAlign withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
        this.DriveRequestType = newDriveRequestType;
        return this;
    }

    /**
     * Modifies the SteerRequestType parameter and returns itself.
     * <p>
     * The type of control request to use for the drive motor.
     *
     * @param newSteerRequestType Parameter to modify
     * @return this object
     */
    public FieldCentricYAxisAlign withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
        this.SteerRequestType = newSteerRequestType;
        return this;
    }

    /**
     * Modifies the DesaturateWheelSpeeds parameter and returns itself.
     * <p>
     * Whether to desaturate wheel speeds before applying. For more information, see
     * the documentation of {@link SwerveDriveKinematics#desaturateWheelSpeeds}.
     *
     * @param newDesaturateWheelSpeeds Parameter to modify
     * @return this object
     */
    public FieldCentricYAxisAlign withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
        this.DesaturateWheelSpeeds = newDesaturateWheelSpeeds;
        return this;
    }

    /**
     * Modifies the ForwardPerspective parameter and returns itself.
     * <p>
     * The perspective to use when determining which direction is forward.
     *
     * @param newForwardPerspective Parameter to modify
     * @return this object
     */
    public FieldCentricYAxisAlign withForwardPerspective(ForwardPerspectiveValue newForwardPerspective) {
        this.ForwardPerspective = newForwardPerspective;
        return this;
    }

    public FieldCentricYAxisAlign withUpdateTargetPose(Supplier<Pose2d> poseSupplier){
        if (poseSupplier != null){
            this.TargetDirection = poseSupplier.get().getRotation();
            this.XCoordinate = poseSupplier.get().getX();
        }
        return this;
    }

    public FieldCentricYAxisAlign withUpdateTargetTranslation(Supplier<Pose2d> poseSupplier){
        if (poseSupplier != null){
            this.XCoordinate = poseSupplier.get().getX();
        }
        return this;
    }

    public double calcToPoint(double curPoint, double tarPoint){

        // Calculate trapezoidal profile
        double maxDriveRate = Constants.maxLinVel.in(MetersPerSecond);

        //Gets the 2D distance between the target point and current point
        double linearError = tarPoint - curPoint;

        //Trapezoidal Profiling
        double targetSpeed = maxDriveRate * (linearError > 0 ? 1 : -1);
        double rampDownSpeed = linearError / Constants.linRampDownDist.in(Meters) * maxDriveRate;

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed)) targetSpeed = rampDownSpeed;
        
        return targetSpeed;
    }
}
