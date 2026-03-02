package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class IntakeAngle extends SubsystemBase {

    // Motor
    private final TalonFX motor;
    private final CANcoder encoder;

    private final LaserCan leftLaserCan;
    private final LaserCan rightLaserCan;

    private final MutDistance leftHopperLevel = Millimeters.mutable(0);
    private final MutDistance rightHopperLevel = Millimeters.mutable(0);

    // Motor Control Requests
    private final VoltageOut voltCtrl = new VoltageOut(0.0);
    private final MotionMagicVelocityVoltage velCtrl = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage posCtrl = new MotionMagicVoltage(0);

    // SysId
    private final SysIdRoutine sysIdRoutime = new SysIdRoutine(
            new SysIdRoutine.Config(null,
                    Volts.of(4),
                    null,
                    (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                    this::setVoltage,
                    null,
                    this));

    /**
     * Constructor
     * 
     * @param motorID
     */
    public IntakeAngle(
            int motorId,
            int encoderId,
            int leftLaserCanID,
            int rightLaserCanID,
            CANBus bus,
            double gearRatio) {
        motor = new TalonFX(motorId, bus);
        encoder = new CANcoder(encoderId, bus);
        leftLaserCan = new LaserCan(leftLaserCanID);
        rightLaserCan = new LaserCan(rightLaserCanID);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);

        motorConfig.Feedback
                .withSensorToMechanismRatio(1)
                .withRemoteCANcoder(encoder)
                .withRotorToSensorRatio(gearRatio)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

        motorConfig.Slot0
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0);

        // TODO Set intake position limits
        motorConfig.SoftwareLimitSwitch
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(Degrees.of(90))
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Degrees.of(0));

        motor.getConfigurator().apply(motorConfig);
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
        motor.setControl(posCtrl.withPosition(angle));
    }

    /**
     * Gets the current voltage of the intake
     * 
     * @return
     */
    @AutoLogOutput
    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
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
     * Gets the hopper level from the left sensor
     * 
     * @return hopper level from the left sensor
     */
    @AutoLogOutput
    public Distance getLeftHopperLevel() {
        return leftHopperLevel.mut_replace(leftLaserCan.getMeasurement().distance_mm, Millimeters);
    }

    /**
     * Gets the hopper level from the right sensor
     * 
     * @return hopper level from the right sensor
     */
    @AutoLogOutput
    public Distance getRightHopperLevel() {
        return rightHopperLevel.mut_replace(rightLaserCan.getMeasurement().distance_mm, Millimeters);
    }

    /**
     * Gets the maximum hopper level value
     * 
     * @return maximum hopper level value
     */
    @AutoLogOutput
    public Distance getMaxHopperLevel() {
        var leftValue = getLeftHopperLevel();
        var rightValue = getRightHopperLevel();

        return leftValue.gt(rightValue) ? leftValue : rightValue;
    }

    /**
     * Check if the hopper is below a threshold
     * 
     * @param threshold low hopper threshold
     * @return True if the hopper is below a threshold
     */
    public boolean isHooperLow(Distance threshold) {
        return getMaxHopperLevel().gt(threshold);
    }

    /**
     * Retracts the intake at a set velocity when a condition is true
     * 
     * @param retract    True to retract the intake, false otherwise
     * @param retractVel Retract velocity
     */
    public void autoRetract(boolean retract, AngularVelocity retractVel) {
        if (retract) {
            setVelocity(retractVel);
        } else {
            setVelocity(RotationsPerSecond.zero());
        }
    }

    /**
     * Creates a new command to run the intake at a set target voltage
     * 
     * @param target target voltage
     * @return new command to run the intake at a set target voltage
     */
    public Command setVoltageCmd(Voltage target) {
        return this.runEnd(
                () -> setVoltage(target),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the intake at a set target velocity
     * 
     * @param target target velocity
     * @return new command to run the intake at a set target velocity
     */
    public Command setVelocityCmd(AngularVelocity target) {
        return this.runEnd(
                () -> setVelocity(target),
                () -> setVelocity(RotationsPerSecond.zero()));
    }

    /**
     * Creates a new command to run the intake at a set target velocity
     * 
     * @param target target velocity
     * @return new command to run the intake at a set target velocity
     */
    public Command setVelocityCmd(Angle target) {
        return this.runEnd(
                () -> setPosition(target),
                () -> setVelocity(RotationsPerSecond.zero()));
    }

    /**
     * Craetes a new command to automatically retract the intake when the hopper is
     * low
     * 
     * @param lowHopperThresh threshold for a low hopper
     * @param retractVel      velocity to retract the intake
     * @return new command to automatically retract the intake when the hopper is
     *         low
     */
    public Command autoRetractCmd(Distance lowHopperThresh, AngularVelocity retractVel) {
        return Commands.deadline(
                Commands.waitUntil(() -> motor.getPosition().getValue().gt(Constants.intakeAutoRetractLimit)),
                this.runEnd(() -> autoRetract(isHooperLow(lowHopperThresh), Constants.intakeAutoRetractVel),
                        () -> setVelocity(RotationsPerSecond.zero())));
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

    /**
     * Update intake RPM on dashboard
     */
    @Override
    public void periodic() {
        // TODO Remove and use CTRE or AdvantageKit telemetry
        // SmartDashboard.putNumber("Intake Angle", getPosition().in(Degrees));
        // SmartDashboard.putNumber("Intake Angle RPM",
        // getVelocity().in(Rotations.per(Minute)));
    }

    @AutoLogOutput
    public String getCommandString() {
        return this.getCurrentCommand() == null ? "null" : this.getCurrentCommand().getName();
    }
}
