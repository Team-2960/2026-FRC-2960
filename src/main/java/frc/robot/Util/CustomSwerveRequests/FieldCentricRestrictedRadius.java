package frc.robot.Util.CustomSwerveRequests;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class FieldCentricRestrictedRadius implements SwerveRequest{
    /**
     * The desired velocity to travel along the circle created around the orbital point using the radius.
     * The travel velocity is eventually split into an X and Y Velocity to feed into the FieldCentric Request.
     */
    public double TravelVelocity = 0;

    public double distance = 0;

    /**
     * The desired radius of the circle.
     * Default value is 0 meters.
     * This value must be set in all requests.
     */
    public double MaxRadius = 0;

    public double MinRadius = 0;

    /**
     * The desired point for the robot to rotate around.
     * Default point is made by a new Translation2d() at (0, 0).
     */
    public Translation2d OrbitPoint = new Translation2d();
    
    /**
     * The offset applied to the rotation of the robot.
     */
    public Rotation2d RotationOffset = new Rotation2d();

    /**
     * The PID Controller used to keep the robot travelling along the circle.
     * Values must be given for the orbital motion to work correctly.
     */
    public PIDController RadiusCorrectionPID = new PIDController(0, 0, 0);

    public double RadiusTolerance = 0;

    public double RadiusVelocity = 0;

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

    private final FieldCentricFacingAngle m_fieldCentricFacingAngle = new FieldCentricFacingAngle();

    public FieldCentricRestrictedRadius(){
        HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }


    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        // 1. Account for Alliance Flipping
        // If Red, we flip the X coordinate (and Y if the field isn't symmetrical)

        Pose2d robotPose = parameters.currentPose;

        Translation2d relativeVector = OrbitPoint.minus(robotPose.getTranslation());
        double currentDistance = relativeVector.getNorm();

        //Value used for telemetry
        distance = currentDistance;
        
        // 2. Calculate the Unit Vectors
        // Unit vector pointing FROM robot TO target (Radial)
        Translation2d unitRadial = relativeVector.div(currentDistance);
        // Unit vector pointing tangent to the circle (Tangential)
        Translation2d unitTangential = new Translation2d(-unitRadial.getY(), unitRadial.getX());

        // 3. Radial Velocity (Correction to stay on the radius)
        // Positive kP moves us toward the circle if we are off
        double radialMag = -RadiusCorrectionPID.calculate(currentDistance, MaxRadius); // P-gain for distance correction

        if (currentDistance < MaxRadius){
            radialMag = RadiusVelocity;
        } else if (currentDistance >= MaxRadius && RadiusVelocity > 0){
            radialMag = RadiusVelocity;
        } 
        
        if (currentDistance < MinRadius){
            radialMag = -RadiusCorrectionPID.calculate(currentDistance, MinRadius);
            
            if (RadiusVelocity < 0){
                radialMag = RadiusVelocity;
            }
        }


        // 4. Combine Vectors
        double vx = (unitTangential.getX() * TravelVelocity) + (unitRadial.getX() * radialMag);
        double vy = (unitTangential.getY() * TravelVelocity) + (unitRadial.getY() * radialMag);


        
        // 5. Target Angle
        // The robot faces the point. 
        // Use targetPoint.minus(robotPose).getAngle()
        Rotation2d targetAngle = relativeVector.getAngle().plus(RotationOffset);

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
        .withTargetDirection(targetAngle)
        .withTargetRateFeedforward(TargetRateFeedforward)
        .withVelocityX(vx)
        .withVelocityY(vy)
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
        public FieldCentricRestrictedRadius withHeadingPID(double kP, double kI, double kD)
        {
            this.HeadingController.setPID(kP, kI, kD);
            return this;
        }

        /**
         * Changes the velocity of the robot's travel along the circle.
         * @param newVelocity The desired velocity for the robot to travel at in Meters Per Second.
         * @return this object
         */
        public FieldCentricRestrictedRadius withTravelVelocity(double newVelocity){
            this.TravelVelocity = newVelocity;
            return this;
        }

        /**
         * Changes the velocity of the robot's travel along the circle.
         * 
         * @param newVelocity
         * @return
         */
        public FieldCentricRestrictedRadius withTravelVelocity(LinearVelocity newVelocity){
            this.TravelVelocity = newVelocity.in(MetersPerSecond);
            return this;
        }

        public FieldCentricRestrictedRadius withMaxRadius(double radius){
            this.MaxRadius = radius;
            return this;
        }
        
        public FieldCentricRestrictedRadius withMaxRadius(Distance radius){
            this.MaxRadius = radius.in(Meters);
            return this;
        }

        public FieldCentricRestrictedRadius withMinRadius(double radius){
            this.MinRadius = radius;
            return this;
        }

        public FieldCentricRestrictedRadius withMinRadius(Distance radius){
            this.MinRadius = radius.in(Meters);
            return this;
        }

        public FieldCentricRestrictedRadius withOrbitPoint(Translation2d point){
            this.OrbitPoint = point;
            return this;
        }

        public FieldCentricRestrictedRadius withOrbitPoint(Distance x, Distance y){
            this.OrbitPoint = new Translation2d(x, y);
            return this;
        }

        public FieldCentricRestrictedRadius withOrbitPoint(double x, double y){
            this.OrbitPoint = new Translation2d(x, y);
            return this;
        }

        public FieldCentricRestrictedRadius withRotationalOffset(Rotation2d offset){
            this.RotationOffset = offset;
            return this;
        }

        public FieldCentricRestrictedRadius withRotationalOffset(Angle offset){
            this.RotationOffset = new Rotation2d(offset);
            return this;
        }

        public FieldCentricRestrictedRadius withRotationalOffset(double offset){
            this.RotationOffset = new Rotation2d(offset);
            return this;
        }

        public FieldCentricRestrictedRadius withRadiusCorrectionPID(double kP, double kI, double kD){
            RadiusCorrectionPID.setPID(kP, kI, kD);
            return this;
        }

        public FieldCentricRestrictedRadius withRadiusTolerance(Distance tolerance){
            this.RadiusTolerance = tolerance.in(Meters);
            return this;
        }

        public FieldCentricRestrictedRadius withRadiusTolerance(double tolerance){
            this.RadiusTolerance = tolerance;
            return this;
        }

        public FieldCentricRestrictedRadius withRadiusVelocity(double velocity){
            this.RadiusVelocity = velocity;
            return this;
        }

        public FieldCentricRestrictedRadius withRadiusVelocity(LinearVelocity velocity){
            this.RadiusVelocity = velocity.in(MetersPerSecond);
            return this;
        }

        public FieldCentricRestrictedRadius withUpdateTargetPose(Supplier<Pose2d> poseSupplier){
            if (poseSupplier.get() != null){
                this.OrbitPoint = poseSupplier.get().getTranslation();
            }
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
        public FieldCentricRestrictedRadius withTargetDirection(Rotation2d newTargetDirection) {
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
        public FieldCentricRestrictedRadius withTargetRateFeedforward(double newTargetRateFeedforward) {
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
        public FieldCentricRestrictedRadius withTargetRateFeedforward(AngularVelocity newTargetRateFeedforward) {
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
        public FieldCentricRestrictedRadius withDeadband(double newDeadband) {
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
        public FieldCentricRestrictedRadius withDeadband(LinearVelocity newDeadband) {
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
        public FieldCentricRestrictedRadius withRotationalDeadband(double newRotationalDeadband) {
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
        public FieldCentricRestrictedRadius withRotationalDeadband(AngularVelocity newRotationalDeadband) {
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
        public FieldCentricRestrictedRadius withMaxAbsRotationalRate(double newMaxAbsRotationalRate) {
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
        public FieldCentricRestrictedRadius withMaxAbsRotationalRate(AngularVelocity newMaxAbsRotationalRate) {
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
        public FieldCentricRestrictedRadius withCenterOfRotation(Translation2d newCenterOfRotation) {
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
        public FieldCentricRestrictedRadius withDriveRequestType(SwerveModule.DriveRequestType newDriveRequestType) {
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
        public FieldCentricRestrictedRadius withSteerRequestType(SwerveModule.SteerRequestType newSteerRequestType) {
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
        public FieldCentricRestrictedRadius withDesaturateWheelSpeeds(boolean newDesaturateWheelSpeeds) {
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
        public FieldCentricRestrictedRadius withForwardPerspective(ForwardPerspectiveValue newForwardPerspective) {
            this.ForwardPerspective = newForwardPerspective;
            return this;
        }
        

        public Distance getTargetDistance(){
            return Meters.of(distance);
        }
}
