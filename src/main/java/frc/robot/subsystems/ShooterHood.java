package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.EncoderConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.FieldLayout;

public class ShooterHood extends SubsystemBase {

    public class FeedbackControllerTuning implements Sendable{
        
        private double kP = 0;
        private double kI = 0;
        private double kD = 0;
        private double kS = 0;
        private double kV = 0;
        private double kA = 0;

        @Override
        public void initSendable(SendableBuilder builder){
            builder.addDoubleProperty("kP", () -> kP, (num) -> kP = num);
            builder.addDoubleProperty("kI", () -> kI, (num) -> kI = num);
            builder.addDoubleProperty("kD", () -> kD, (num) -> kD = num);
            builder.addDoubleProperty("kS", () -> kS, (num) -> kS = num);
            builder.addDoubleProperty("kV", () -> kV, (num) -> kV = num);
            builder.addDoubleProperty("kA", () -> kA, (num) -> kA = num);
        }

        public double getkP(){
            return kP;
        }

        public double getkI() {
            return kI;
        }

        public double getkD() {
            return kD;
        }

        public double getkS() {
            return kS;
        }

        public double getkV() {
            return kV;
        }

        public double getkA() {
            return kA;
        }
    }

    // Motor
    private final TalonFX motor;
    private final CANcoder encoder;

    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs();

    // Motor Control Requests
    private final VoltageOut voltCtrl = new VoltageOut(0.0);
    private final MotionMagicVelocityVoltage velCtrl = new MotionMagicVelocityVoltage(0)
        .withAcceleration(Degrees.of(60).per(Second).per(Second));
    private final MotionMagicVoltage posCtrl = new MotionMagicVoltage(0)
        .withSlot(0);

    private final MotionMagicTorqueCurrentFOC posCtrl2 = new MotionMagicTorqueCurrentFOC(0)
        .withSlot(1);
    private final CommandSwerveDrivetrain drivetrain;

    // SysId
    private final SysIdRoutine sysIdRoutime = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.25).per(Second),
                    Volts.of(2.0),
                    null,
                    (state) -> SignalLogger.writeString("Shooter Hood State", state.toString())),
            new SysIdRoutine.Mechanism(
                    this::setVoltage,
                    null,
                    this));

    private final FeedbackControllerTuning feedbackControllerTuning = new FeedbackControllerTuning();

    /**
     * Constructor
     * 
     * @param motorID
     */
    public ShooterHood(int motorId, int encoderId, CANBus bus, double gearRatio, CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        motor = new TalonFX(motorId, bus);
        encoder = new CANcoder(encoderId, bus);

        encoderConfig.withSensorDirection(SensorDirectionValue.Clockwise_Positive);

        motorConfig.CurrentLimits
            .withSupplyCurrentLimit(Constants.krakenX60CurrentLimit)
            .withSupplyCurrentLimitEnable(true);

        motorConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);

        motorConfig.Feedback
                //.withSensorToMechanismRatio(2.5)
                .withSensorToMechanismRatio(gearRatio)
                // .withRemoteCANcoder(encoder)
                //.withRotorToSensorRatio(gearRatio);
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

        motorConfig.Slot0
                .withKP(10.0)
                .withKI(0.0)
                .withKD(2.0)
                .withKS(0.3)
                .withKV(4)
                .withKA(0.0)
                .withKG(0.6);

        motorConfig.Slot1
            .withKP(2.0)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(12)
            .withKV(0)
            .withKA(0)
            .withKG(5)
            .withGravityType(GravityTypeValue.Arm_Cosine);

        motorConfig.Slot2
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0);

        motorConfig.MotionMagic
            .withMotionMagicCruiseVelocity(Degrees.of(120).per(Second))
            .withMotionMagicAcceleration(Degrees.of(240).per(Second).per(Second))
            .withMotionMagicJerk(Degrees.of(480).per(Second).per(Second).per(Second));
        
        motor.getConfigurator().apply(motorConfig);
        motor.getConfigurator().setPosition(Degrees.of(-40));
        encoder.getConfigurator().apply(encoderConfig);
    }

    /**
     * Gets the current voltage of the shooter hood
     * 
     * @return
     */
    @AutoLogOutput
    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
    }

    @AutoLogOutput
    public Current getCurrent(){
        return motor.getSupplyCurrent().getValue();
    }

    /**
     * Gets the current velocity of the motor
     * 
     * @return current velocity of the motor
     */
    @AutoLogOutput
    public AngularVelocity getVelocity() {
        return motor.getVelocity().getValue();
    }

    /**
     * Gets the current Position of the motor
     * 
     * @return current Position of the motor
     */
    @AutoLogOutput
    public Angle getPosition() {
        return motor.getPosition().getValue();
    }

    /**
     * Checks if the current velocity is within tolerance of the set point
     * 
     * @param tol measurement tolerance
     * @return true if in velocity control mode and within tolerance of the target.
     *         False otherwise
     */
    public boolean atVelocity(AngularVelocity tol) {
        return motor.getAppliedControl() == velCtrl &&
                MathUtil.isNear(
                        velCtrl.Velocity,
                        getVelocity().in(RotationsPerSecond),
                        tol.in(RotationsPerSecond));
    }

    /**
     * Checks if the current position is within tolerance of the set point
     * 
     * @param tol measurement tolerance
     * @return true if in position control mode and within tolerance of the target.
     *         False otherwise
     */
    public boolean atPosition(Angle tol) {
        return motor.getAppliedControl() == velCtrl &&
                MathUtil.isNear(
                        posCtrl.Position,
                        getPosition().in(Rotations),
                        tol.in(Rotations));
    }

    /**
     * Sets a target voltage to the motor
     * 
     * @param volts target voltage
     */
    public void setVoltage(Voltage volts) {
        motor.setControl(voltCtrl.withOutput(volts));
    }

    /**
     * Sets a target velocity to the motor
     * 
     * @param velocity target velocity
     */
    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(velCtrl.withVelocity(velocity));
    }

    /**
     * Sets a target velocity to the motor
     * 
     * @param velocity target velocity
     */
    public void setPosition(Angle angle) {
        //motor.setControl(posCtrl.withPosition(angle).withSlot(0));
        motor.setControl(posCtrl2.withPosition(angle).withSlot(1));
    }

    /**
     * Creates a new command to run the shooter hood at a set target voltage
     * 
     * @param target target voltage
     * @return new command to run the shooter hood at a set target voltage
     */
    public Command setVoltageCmd(Voltage target) {
        return this.runEnd(
                () -> setVoltage(target),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the shooter hood at a set target voltage
     * 
     * @param target target voltage
     * @return new command to run the shooter hood at a set target voltage
     */
    public Command setVoltageCmd(Supplier<Voltage> target) {
        return this.runEnd(
                () -> setVoltage(target.get()),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the shooter hood at a set target velocity
     * 
     * @param target target velocity
     * @return new command to run the shooter hood at a set target velocity
     */
    public Command setVelocityCmd(AngularVelocity target) {
        return this.runEnd(
                () -> setVelocity(target),
                () -> setVelocity(RotationsPerSecond.zero()));
    }

    /**
     * Creates a new command to run the shooter hood at a set target velocity
     * 
     * @param target target velocity
     * @return new command to run the shooter hood at a set target velocity
     */
    public Command setPositionCmd(Angle target) {
        return this.runEnd(
                () -> setPosition(target),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the shooter hood at a set target velocity
     * 
     * @param target target velocity
     * @return new command to run the shooter hood at a set target velocity
     */
    public Command setPositionCmd(Supplier<Angle> target) {
        return this.runEnd(
                () -> setPosition(target.get()),
                () -> setVoltage(Volts.zero()));
    }

    public Command setPositionTestCmd(Supplier<Angle> target) {
        return this.startRun(
            () -> motor.getConfigurator().refresh(motorConfig.Slot2), 
            () -> setPosition(target.get())
        )
        .finallyDo(() -> setVelocity(RotationsPerSecond.zero()));
    }


    /**
     * Creates a new command to set the shooter hood for shooting at the hub
     * 
     * @return new command to set the shooter hood for shooting at the hub
     */
    public Command hubShotCmd() {
        return setPositionCmd(this::calcHubShotAngle);
    }

    /**
     * Create a Quasistatic SysId command
     * 
     * @param direction direction of the command
     * @return Quasistatic SysId command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutime.quasistatic(direction);
    }

    /**
     * Create a Dynamic SysId command
     * 
     * @param direction direction of the command
     * @return Dynamic SysId command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutime.dynamic(direction);
    }

    public Command sysIdQuasistaticLimited(SysIdRoutine.Direction direction) {
        if (direction.equals(Direction.kReverse)){
            return sysIdRoutime.quasistatic(direction)
                .until(() -> getPosition().lte((Degrees.of(-42))));//TODO EDIT
        }else{
            return sysIdRoutime.quasistatic(direction)
                .until(() -> getPosition().gte((Degrees.of(-10))));//EDIT
        }
    }

    public Command sysIdDynamicLimited(SysIdRoutine.Direction direction) {
        if (direction.equals(Direction.kReverse)){
            return sysIdRoutime.dynamic(direction)
                .until(() -> getPosition().lte(Degrees.of(-42)));//EDIT
        }else{
            return sysIdRoutime.dynamic(direction)
                .until(() -> getPosition().gte(Degrees.of(-10)));//EDIT
        }
    }

    /**
     * Update shooter hood RPM on dashboard
     */
    @Override
    public void periodic() {
        // TODO Remove and use CTRE or AdvantageKit telemetry
        SmartDashboard.putNumber("Shooter Hood Angle", getPosition().in(Degrees));
        SmartDashboard.putNumber("Shooter Hood Angle RPM", getVelocity().in(Rotations.per(Minute)));
        SmartDashboard.putData("Tuning", feedbackControllerTuning);
        
        motorConfig.Slot2
            .withKP(feedbackControllerTuning.getkP())
            .withKI(feedbackControllerTuning.getkI())
            .withKD(feedbackControllerTuning.getkD())
            .withKS(feedbackControllerTuning.getkS())
            .withKV(feedbackControllerTuning.getkV())
            .withKA(feedbackControllerTuning.getkA());
    }

    private Angle calcHubShotAngle() {
        Distance hubDist = FieldLayout.Hub.getHubDist(drivetrain.getPose2d().getTranslation());

        // TODO Implement Formula for target shooter hood angle

        return Rotations.zero();
    }

    @AutoLogOutput
    public String getCommandString() {
        return this.getCurrentCommand() == null ? "null" : this.getCurrentCommand().getName();
    }
}
