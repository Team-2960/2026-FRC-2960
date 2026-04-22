package frc.lib2960.subsystems.mechanisms;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Value;

import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.lib2960.util.ControlTuner;
import frc.lib2960.util.OutputTuner;
import frc.robot.Constants;

public class FlywheelBase extends SubsystemBase {
    private final FlywheelConfigBase config;

    // Motors
    private final ArrayList<TalonFX> motors;
    private final TalonFX motorLeader;

    // Motor Control Requests
    private final DutyCycleOut dutyCycleCtrl = new DutyCycleOut(0).withEnableFOC(true);
    private final VoltageOut voltCtrl = new VoltageOut(0.0).withEnableFOC(true);
    private final TorqueCurrentFOC currentCtrl = new TorqueCurrentFOC(0);

    private final MotionMagicVelocityVoltage voltVelCtrl = new MotionMagicVelocityVoltage(0)
            .withAcceleration(Constants.shooterMaxAccel)
            .withEnableFOC(true);
    private final MotionMagicVelocityTorqueCurrentFOC torqueVelCtrl = new MotionMagicVelocityTorqueCurrentFOC(0)
            .withAcceleration(Constants.shooterMaxAccel)
            .withSlot(1);

    // Testing
    private final OutputTuner outputTuner = new OutputTuner();
    private final SendableChooser<Integer> testSlotChooser = new SendableChooser<>();

    // SysId
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(5).per(Second),
                    Volts.of(20),
                    Seconds.of(5),
                    (state) -> Logger.recordOutput("ShooterWheel", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setCurrent(Amps.of(volts.in(Volts))),
                    null,
                    this));

    /**
     * Constructor
     * 
     * @param motorLeaderID   CAN ID of the lead shooter motor
     * @param motorFollowerID CAN ID of the follower shooter motor
     * @param bus             CAN Bus the shooter motors are on
     * @param gearRatio       Gear ratio between the shooter and the motor
     */
    public FlywheelBase(FlywheelConfigBase config) {
        this.config = config;
        setName(config.name);

        motors = config.createTalonFXs();
        motorLeader = motors.get(0);

        // Configure Motors
        motorLeader.getConfigurator().apply(config.createTalonFXConfig());

        for (int i = 1; i < motors.size(); i++) {
            boolean isOpposed = config.motorGroupConfig.motorConfigs
                    .get(0).inverted != config.motorGroupConfig.motorConfigs.get(i).inverted;

            motors.get(i).setControl(
                    new Follower(
                            motorLeader.getDeviceID(),
                            isOpposed ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
        }

        // Test Controls
        SmartDashboard.putData(config.name + " Test Output Control", outputTuner);
        SmartDashboard.putData(config.name + " Run Test Voltage", run(() -> setVoltage(outputTuner.voltage)));
        SmartDashboard.putData(config.name + " Run Test Current", run(() -> setCurrent(outputTuner.current)));

        testSlotChooser.addOption("Slot 0", 0);
        testSlotChooser.addOption("Slot 1", 1);
        testSlotChooser.addOption("Slot 2", 2);
        SmartDashboard.putData(testSlotChooser);

        SmartDashboard.putData(config.name + " Run Test Voltage Velocity",
                run(() -> setVoltageVelocity(outputTuner.velocity, testSlotChooser.getSelected().intValue())));
        SmartDashboard.putData(config.name + " Run Test Torque-Current Velocity",
                run(() -> setTorqueCurrentVel(outputTuner.velocity, testSlotChooser.getSelected().intValue())));

        // Setup Control Tuner
        SmartDashboard.putData(
                config.name + " Slot 0 Tuner",
                new ControlTuner(config.motorGroupConfig.controlConfigs.get(0)));
        SmartDashboard.putData(
                config.name + " Slot 1 Tuner",
                new ControlTuner(config.motorGroupConfig.controlConfigs.get(1)));
        SmartDashboard.putData(
                config.name + " Slot 2 Tuner",
                new ControlTuner(config.motorGroupConfig.controlConfigs.get(2)));

        SmartDashboard.putData(
                config.name + " Apply Slot 0",
                Commands.runOnce(
                        () -> motorLeader.getConfigurator().apply(config.motorGroupConfig.applySlot0Config())));
        SmartDashboard.putData(
                config.name + " Apply Slot 1",
                Commands.runOnce(
                        () -> motorLeader.getConfigurator().apply(config.motorGroupConfig.applySlot1Config())));
        SmartDashboard.putData(
                config.name + " Apply Slot 2",
                Commands.runOnce(
                        () -> motorLeader.getConfigurator().apply(config.motorGroupConfig.applySlot2Config())));
    }

    /**
     * Gets the current voltage of the intake
     * 
     * @return
     */
    @AutoLogOutput
    public Voltage getVoltage() {
        return motorLeader.getMotorVoltage().getValue();
    }

    /**
     * Gets the current voltage of the intake
     * 
     * @return
     */
    @AutoLogOutput
    public Current getStatorCurrent() {
        return motorLeader.getStatorCurrent().getValue();
    }

    /**
     * Gets the current voltage of the intake
     * 
     * @return
     */
    @AutoLogOutput
    public Current getSupplyCurrent() {
        return motorLeader.getSupplyCurrent().getValue();
    }

    /**
     * Gets the current position of the motor
     * 
     * @return current position of the motor
     */
    @AutoLogOutput
    public Angle getPosition() {
        return motorLeader.getPosition().getValue();
    }

    /**
     * Gets the current velocity of the motor
     * 
     * @return current velocity of the motor
     */
    @AutoLogOutput
    public AngularVelocity getVelocity() {
        return motorLeader.getVelocity().getValue();
    }

    /**
     * Gets the current target velocity. Returns zero of Velocity Control is not
     * running.
     * 
     * @return Gets the current target velocity. Returns zero of Velocity Control is
     *         not running.
     */
    @AutoLogOutput
    public AngularVelocity getTargetVelocity() {
        AngularVelocity result = Rotations.per(Minute).zero();

        if (motorLeader.getAppliedControl() == voltVelCtrl)
            result = voltVelCtrl.getVelocityMeasure();
        if (motorLeader.getAppliedControl() == torqueVelCtrl)
            result = torqueVelCtrl.getVelocityMeasure();

        return result;
    }

    /**
     * Gets the current acceleration of the motor
     * 
     * @return current acceleration of the motor
     */
    @AutoLogOutput
    public AngularAcceleration getAcceleration() {
        return motorLeader.getAcceleration().getValue();
    }

    /**
     * Gets the velocity in RPM
     * 
     * @return velocity in RPM
     */
    @AutoLogOutput
    public double getRPM() {
        return getVelocity().in(Rotations.per(Minute));
    }

    /**
     * Checks if any velocity control is active
     * 
     * @return true if any velocity control is running, false otherwise
     */
    public boolean isVelocityCtrl() {
        return motorLeader.getAppliedControl() == voltVelCtrl ||
                motorLeader.getAppliedControl() == torqueVelCtrl;
    }

    /**
     * Checks if the current velocity is within tolerance of the set point
     * 
     * @param tol measurement tolerance
     * @return true if in torque control mode and within tolerance of the target.
     *         False otherwise
     */
    public boolean atVelocity(AngularVelocity tol) {
        boolean isNearVel = MathUtil.isNear(
                getTargetVelocity().in(RotationsPerSecond),
                getVelocity().in(RotationsPerSecond),
                tol.in(RotationsPerSecond));

        return isVelocityCtrl() && isNearVel;
    }

    /**
     * 
     * @param floorThreshold   The maximum angular velocity of the shooter threshold
     * @param ceilingThreshold The minimum angular velocity of the shooter threshold
     * @return returns true if shooter velocity is greater than floor and less than
     *         ceiling
     */
    public boolean atVelocity(AngularVelocity floorThreshold, AngularVelocity ceilingThreshold) {
        var target = getTargetVelocity();
        var current = getVelocity();

        return current.gt(target.minus(floorThreshold)) && current.lt(target.plus(ceilingThreshold));
    }

    /**
     * Sets the target motor voltage
     * 
     * @param value target motor voltage
     */
    public void setVoltage(Voltage value) {
        motorLeader.setControl(voltCtrl.withOutput(value));
    }

    /**
     * Sets the target motor current
     * 
     * @param value Sets the target motor current
     */
    public void setCurrent(Current value) {
        motorLeader.setControl(currentCtrl.withOutput(value));
    }

    /**
     * Sets the motor to a duty cycle output.
     * 
     * @param value duty
     */
    public void setDutyCycle(Dimensionless value) {
        motorLeader.setControl(dutyCycleCtrl.withOutput(value.in(Value)));
    }

    /**
     * Sets the target motor velocity using voltage control
     * 
     * @param velocity target motor velocity
     * @param slot     control parameter slot to use. Must be between 0 and 2
     *                 (inclusive)
     */
    public void setVoltageVelocity(AngularVelocity velocity, int slot) {
        motorLeader.setControl(voltVelCtrl.withVelocity(velocity).withSlot(slot));
    }

    /**
     * Sets the target motor velocity using torque current control
     * 
     * @param velocity target motor velocity
     * @param slot     control parameter slot to use. Must be between 0 and 2
     *                 (inclusive)
     */
    public void setTorqueCurrentVel(AngularVelocity velocity, int slot) {
        motorLeader.setControl(torqueVelCtrl.withVelocity(velocity).withSlot(slot));
    }

    /**
     * Creates a new command to run the mechanism at a set target voltage
     * 
     * @param value target voltage
     * @return new command to run the mechanism at a set target voltage
     */
    public Command setVoltageCmd(Voltage value) {
        return this.runEnd(
                () -> setVoltage(value),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the mechanism at a set target voltage
     * 
     * @param value target voltage supplier
     * @return new command to run the mechanism at a set target voltage
     */
    public Command setVoltageCmd(Supplier<Voltage> value) {
        return this.runEnd(
                () -> setVoltage(value.get()),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the mechanism at a set target current
     * 
     * @param value target current
     * @return new command to run the mechanism at a set target current
     */
    public Command setCurrentCmd(Current value) {
        return this.runEnd(
                () -> setCurrent(value),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the mechanism at a set target current
     * 
     * @param value target current supplier
     * @return new command to run the mechanism at a set target current
     */
    public Command setCurrentCmd(Supplier<Current> amps) {
        return this.runEnd(
                () -> setCurrent(amps.get()),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the mechanism at a set target velocity
     * 
     * @param volts target velocity
     * @param slot  control parameter slot to use. Must be between 0 and 2
     *              (inclusive)
     * @return new command to run the mechanism at a set target velocity
     */
    public Command setVelocityCmd(AngularVelocity velocity, int slot) {
        return this.runEnd(
                () -> setVoltageVelocity(velocity, slot),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the mechanism at a set target velocity
     * 
     * @param volts target velocity
     * @param slot  control parameter slot to use. Must be between 0 and 2
     *              (inclusive)
     * @return new command to run the mechanism at a set target velocity
     */
    public Command setVelocityCmd(Supplier<AngularVelocity> velocity, int slot) {
        return this.runEnd(
                () -> setVoltageVelocity(velocity.get(), slot),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the mechanism at a set target velocity
     * 
     * @param volts target velocity
     * @param slot  control parameter slot to use. Must be between 0 and 2
     *              (inclusive)
     * @return new command to run the mechanism at a set target velocity
     */
    public Command setTorqueVelocityCmd(AngularVelocity velocity, int slot) {
        return this.runEnd(
                () -> setTorqueCurrentVel(velocity, slot),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the mechanism at a set target velocity
     * 
     * @param volts target velocity
     * @param slot  control parameter slot to use. Must be between 0 and 2
     *              (inclusive)
     * @return new command to run the mechanism at a set target velocity
     */
    public Command setTorqueVelocityCmd(Supplier<AngularVelocity> velocity, int slot) {
        return this.runEnd(
                () -> setTorqueCurrentVel(velocity.get(), slot),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a command that sets the shooter output to zero volts
     * 
     * @return a command that sets the shooter output to zero volts
     */
    public Command stopCmd() {
        return setVoltageCmd(Volts.zero());
    }

    /**
     * Create a Quasistatic SysId command
     * 
     * @param direction direction of the command
     * @return Quasistatic SysId command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Create a Dynamic SysId command
     * 
     * @param direction direction of the command
     * @return Dynamic SysId command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    /**
     * Creates a full Sys ID sequence
     * 
     * @return full Sys ID sequence
     */
    public Command sysIDSequence() {
        return sysIdQuasistatic(Direction.kForward)
                .andThen(sysIdQuasistatic(Direction.kReverse))
                .andThen(sysIdDynamic(Direction.kForward))
                .andThen(sysIdDynamic(Direction.kReverse));
    }

    /**
     * Update shooter RPM on dashboard
     */
    @Override
    public void periodic() {
        // TODO Find better solution for this update
        SmartDashboard.putNumber(this.config.name + " RPM", getVelocity().in(Rotations.per(Minute)));
    }
}