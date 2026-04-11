package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.FieldLayout;

public class ShooterWheel extends SubsystemBase {

    /**
     * Class for controlling the shooter speed from the dashboard
     */
    public class ShooterWheelTest implements Sendable {

        private double velRPM = 0;
        private double kP = 0;
        private double kI = 0;
        private double kD = 0;
        private double kS = 0;
        private double kV = 0;
        private double kA = 0;

        @Override
        public void initSendable(SendableBuilder builder) {
            // TODO Auto-generated method stub
            builder.addDoubleProperty("Shooter Test Speed (RPM)", () -> velRPM, (val) -> velRPM = val);
            builder.addDoubleProperty("Shooter Test kP", () -> kP, (val) -> kP = val);
            builder.addDoubleProperty("Shooter Test kI", () -> kI, (val) -> kI = val);
            builder.addDoubleProperty("Shooter Test kD", () -> kD, (val) -> kD = val);
            builder.addDoubleProperty("Shooter Test kS", () -> kS, (val) -> kS = val);
            builder.addDoubleProperty("Shooter Test kV", () -> kV, (val) -> kV = val);
            builder.addDoubleProperty("Shooter Test kA", () -> kA, (val) -> kA = val);
        }

        /**
         * Gets a command to control the shooter from
         * 
         * @return
         */
        public Command getCommand() {
            return hubPhaseShotCmd(() -> Rotations.per(Minute).of(velRPM), Rotations.per(Minute).of(200),
                    Rotations.per(Minute).of(150));
        }

        public double getkP() {
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

    // Motors
    private final TalonFX motorLeader;
    private final TalonFX motorFollower;

    // Configure Motors
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // Motor Control Requests
    private final VoltageOut voltCtrl = new VoltageOut(0.0).withEnableFOC(true);
    private final MotionMagicVelocityVoltage velCtrl = new MotionMagicVelocityVoltage(0)
            .withAcceleration(Constants.shooterMaxAccel)
            .withEnableFOC(true)
            .withSlot(0);
    private final MotionMagicVelocityTorqueCurrentFOC torqueCtrl = new MotionMagicVelocityTorqueCurrentFOC(0)
            .withAcceleration(Constants.shooterMaxAccel)
            .withSlot(1);
    private final DutyCycleOut dutyCycleCtrl = new DutyCycleOut(0).withEnableFOC(true);

    private final TorqueCurrentFOC currentCtrl = new TorqueCurrentFOC(0);

    private final CommandSwerveDrivetrain drivetrain;

    private final ShooterWheelTest shooterWheelTest = new ShooterWheelTest();

    private Orchestra orchestra = new Orchestra();

    private AngularVelocity logTargetVelocity = RotationsPerSecond.of(0);

    // SysId

    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null,
                    Volts.of(4),
                    Seconds.of(5),
                    (state) -> SignalLogger.writeString("ShooterWheel", state.toString())),
            new SysIdRoutine.Mechanism(
                    this::setVoltage,
                    null,
                    this));

    private final SysIdRoutine sysIdRoutine2 = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(2), Seconds.of(5)),
            new SysIdRoutine.Mechanism(this::setVoltage, this::sysIDLogging, this));

    private final SysIdRoutine sysIdRoutine3 = new SysIdRoutine(
            new SysIdRoutine.Config(null,
                    Volts.of(0.5),
                    Seconds.of(5),
                    (state) -> SignalLogger.writeString("ShooterWheel", state.toString())),
            new SysIdRoutine.Mechanism(
                    this::setVoltage,
                    null,
                    this));

    private final SysIdRoutine sysIdRoutine4 = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(5).per(Second),
                    Volts.of(20),
                    Seconds.of(5),
                    (state) -> Logger.recordOutput("ShooterWheel", state.toString())),
            new SysIdRoutine.Mechanism(
                    (volts) -> setCurrent(Amps.of(volts.in(Volts))),
                    null,
                    this));

    private SysIdRoutine currentSysIdRoutine = sysIdRoutine4;

    /**
     * Constructor
     * 
     * @param motorLeaderID   CAN ID of the lead shooter motor
     * @param motorFollowerID CAN ID of the follower shooter motor
     * @param bus             CAN Bus the shooter motors are on
     * @param gearRatio       Gear ratio between the shooter and the motor
     */
    public ShooterWheel(int motorLeaderID, int motorFollowerID, CANBus bus, double gearRatio,
            CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Initialize Motors
        motorLeader = new TalonFX(motorLeaderID, bus);
        motorFollower = new TalonFX(motorFollowerID, bus);

        motorConfig.CurrentLimits
            .withSupplyCurrentLimit(Constants.krakenX60CurrentLimit)
            .withSupplyCurrentLimitEnable(true);

        motorConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Coast);

        motorConfig.Feedback
                .withSensorToMechanismRatio(gearRatio);

        motorConfig.MotionMagic.withMotionMagicJerk(Constants.shooterMaxAccel.in(RotationsPerSecondPerSecond) * 10);

        motorConfig.Slot0
                .withKP(0.005)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.28683)
                .withKV(0.11886)
                .withKA(0.0067811);

        // motorConfig.Slot1
        //         .withKP(12.0)
        //         .withKI(0.0)
        //         .withKD(0.0)
        //         .withKS(1.5)
        //         .withKV(0.8)
        //         .withKA(0);

        
        motorConfig.Slot1
                .withKP(12.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.03)
                .withKV(0.3)
                .withKA(0);

        motorConfig.Slot2
                .withKP(0)
                .withKI(0)
                .withKD(0)
                .withKS(0)
                .withKV(0)
                .withKA(0);

        motorLeader.getConfigurator().apply(motorConfig);

        motorFollower.getConfigurator().apply(motorConfig);
        motorFollower.setControl(new Follower(motorLeaderID, MotorAlignmentValue.Opposed));

        motorLeader.getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(true));
        motorFollower.getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(true));

        orchestra.addInstrument(motorLeader);
        orchestra.addInstrument(motorFollower);
        orchestra.loadMusic("washingMachine.chrp");
        // orchestra.play();

        // Setup Shooter Testing
        SmartDashboard.putData("Shooter Wheel Test", shooterWheelTest);
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
     * Gets the current velocity of the motor
     * 
     * @return current velocity of the motor
     */
    @AutoLogOutput
    public AngularVelocity getVelocity() {
        return motorLeader.getVelocity().getValue();
    }

    /**
     * \
     * @return Shooter's Target Velocity when using Torque Control.
     */
    @AutoLogOutput
    public AngularVelocity getTargetVelocity(){
        return torqueCtrl.getVelocityMeasure();
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

    @AutoLogOutput
    public Angle getPosition() {
        return motorLeader.getPosition().getValue();
    }

    @AutoLogOutput
    public double getRPM() {
        double rpm = motorLeader.getVelocity().getValue().in(Rotations.per(Minute));
        SignalLogger.writeDouble("ShooterWheel RPM", rpm);
        return rpm;
    }

    /**
     * Checks if the current velocity is within tolerance of the set point
     * 
     * @param tol measurement tolerance
     * @return true if in torque control mode and within tolerance of the target.
     *         False otherwise
     */
    public boolean atVelocity(AngularVelocity tol) {
        // SmartDashboard.putNumber("Shot Tolerance (RPS)", tol.in(RotationsPerSecond));
        // SmartDashboard.putNumber("Shot Velocity (RPS)",
        // getVelocity().in(RotationsPerSecond));
        // SmartDashboard.putNumber("Shot Target (RPS)", torqueCtrl.Velocity);

        boolean isNearVel = MathUtil.isNear(
                torqueCtrl.Velocity,
                getVelocity().in(RotationsPerSecond),
                tol.in(RotationsPerSecond));

        boolean isTorqueCtrl = motorLeader.getAppliedControl() == torqueCtrl;

        // SmartDashboard.putBoolean("Shot isTorqueCtrl", isTorqueCtrl);
        // SmartDashboard.putBoolean("Shot isNearVel", isNearVel);

        return isTorqueCtrl && isNearVel;
    }

    /**
     * 
     * @param floorThreshold   The maximum angular velocity of the shooter threhold
     * @param ceilingThreshold The minimum angular velocity of the shooter threhold
     * @return returns true if shooter velocity is greater than floor and less than
     *         ceiling
     */
    public boolean atVelocity(AngularVelocity floorThreshold, AngularVelocity ceilingThreshold) {
        return getVelocity().gt(RotationsPerSecond.of(torqueCtrl.Velocity).minus(floorThreshold)) 
            && getVelocity().lt(RotationsPerSecond.of(torqueCtrl.Velocity).plus(ceilingThreshold));
    }

    /**
     * Sets the target motor voltage
     * 
     * @param volts target motor voltage
     */
    public void setVoltage(Voltage volts) {
        motorLeader.setControl(voltCtrl.withOutput(volts));
    }

    public void setCurrent(Current amps) {
        motorLeader.setControl(currentCtrl.withOutput(amps));
    }

    /**
     * Sets the motor to a duty cycle output.
     * Useful for trying to get an output based on the possible max voltage output
     * (battery wise)
     * 
     * @param output
     */
    public void setDutyCyle(double output) {
        motorLeader.setControl(dutyCycleCtrl.withOutput(output));
    }

    /**
     * Sets the target motor velocity
     * 
     * @param velocity target motor velocity
     */
    public void setVelocity(AngularVelocity velocity) {
        motorLeader.setControl(velCtrl.withVelocity(velocity));
    }

    /**
     * Sets the target motor velocity
     * 
     * @param velocity target motor velocity
     */
    public void setTorqueCurrentVel(AngularVelocity velocity) {
        motorLeader.setControl(torqueCtrl.withVelocity(velocity));
    }

    public void setSysIdCurrent(Voltage voltage) {
        motorLeader.setControl(currentCtrl.withOutput(voltage.in(Volts)));
    }

    /**
     * Creates a new command to run the intake at a set target voltage
     * 
     * @param volts target voltage
     * @return new command to run the intake at a set target voltage
     */
    public Command setVoltageCmd(Voltage input) {
        return this.runEnd(
                () -> setVoltage(input),
                () -> setVoltage(Volts.zero()));
    }

    public Command setCurrentCmd(Current amps) {
        return this.runEnd(
                () -> setCurrent(amps),
                () -> setVoltage(Volts.zero()));
    }

    public Command setCurrentCmd(Supplier<Current> amps) {
        return this.runEnd(
                () -> setCurrent(amps.get()),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the intake at a set target velocity
     * 
     * @param volts target velocity
     * @return new command to run the intake at a set target velocity
     */
    public Command setVelocityCmd(AngularVelocity velocity) {
        return this.runEnd(
                () -> setVelocity(velocity),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the intake at a set target velocity
     * 
     * @param volts target velocity
     * @return new command to run the intake at a set target velocity
     */
    public Command setVelocityCmd(Supplier<AngularVelocity> velocity) {
        return this.runEnd(
                () -> setVelocity(velocity.get()),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the intake at a set target velocity
     * 
     * @param volts target velocity
     * @return new command to run the intake at a set target velocity
     */
    public Command setTorqueVelocityCmd(AngularVelocity velocity) {
        return this.runEnd(
                () -> setTorqueCurrentVel(velocity),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the intake at a set target velocity
     * 
     * @param volts target velocity
     * @return new command to run the intake at a set target velocity
     */
    public Command setTorqueVelocityCmd(Supplier<AngularVelocity> velocity) {
        return this.runEnd(
                () -> setTorqueCurrentVel(velocity.get()),
                () -> setVoltage(Volts.zero()));
    }

    public Command setTorqueVelocityTestCmd(Supplier<AngularVelocity> velocity) {
        return this.startRun(
                () -> motorLeader.getConfigurator().refresh(motorConfig.Slot2),
                () -> setTorqueCurrentVel(velocity.get()))
                .finallyDo(() -> setVoltage(Volts.zero()));
    }

    public Command idleVelocityCmd() {
        return setTorqueVelocityCmd((Constants.idleVelocity));
    }

    public Command passVelocityCmd() {
        return setTorqueVelocityCmd((Constants.passVelocity));
    }

    /**
     * Creates a new command to set the shooter for shooting at the hub
     * 
     * @return new command to set the shooter for shooting at the hub
     */
    public Command hubShotCmd() {
        return hubPhaseShotCmd(this::calcHubShotSpeed, Constants.shooterWheelFloorThreshold, Constants.shooterWheelCeilingThreshold);
    }

    /**
     * 
     * @param targetVel
     * @param floorThreshold
     * @param ceilingThreshold
     * @return
     */
    public Command hubPhaseShotCmd(Supplier<AngularVelocity> targetVel, AngularVelocity floorThreshold,
            AngularVelocity ceilingThreshold) {
        return this.runEnd(
                () -> {
                    AngularVelocity curVel = getVelocity();
                    AngularVelocity targetVelVal = targetVel.get();
                    if (targetVelVal.minus(floorThreshold).gt(curVel)) {
                        setDutyCyle(1);
                    } else if (targetVelVal.plus(ceilingThreshold).lt(curVel)) {
                        setVoltage(Volts.zero());
                    } else {
                        setTorqueCurrentVel(targetVel.get());
                    }
                },
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Create a Quasistatic SysId command
     * 
     * @param direction direction of the command
     * @return Quasistatic SysId command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return currentSysIdRoutine.quasistatic(direction);
    }

    /**
     * Create a Dynamic SysId command
     * 
     * @param direction direction of the command
     * @return Dynamic SysId command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return currentSysIdRoutine.dynamic(direction);
    }

    /**
     * Creates a new test command
     * 
     * @return
     */
    public Command getTestCommand() {
        return shooterWheelTest.getCommand();
    }

    /**
     * Update shooter RPM on dashboard
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getVelocity().in(Rotations.per(Minute)));

        if (DriverStation.isEnabled()) {
            orchestra.stop();
        }

        motorConfig.Slot2
                .withKP(shooterWheelTest.getkP())
                .withKI(shooterWheelTest.getkI())
                .withKD(shooterWheelTest.getkD())
                .withKS(shooterWheelTest.getkS())
                .withKV(shooterWheelTest.getkV())
                .withKA(shooterWheelTest.getkA());

        // motorLeader.getConfigurator().refresh(motorConfig);
    }

    /**
     * Calculates the angular velocity for shooting into the hub from the current
     * robot position
     * 
     * @return target angular velocity
     */
    private AngularVelocity calcHubShotSpeed() {
        Distance hubDist = FieldLayout.Hub.getHubDist(drivetrain.getPose2d().getTranslation());

        return Constants.shooterWheelTable.get(hubDist);
    }

    @AutoLogOutput
    public String getCommandString() {
        return this.getCurrentCommand() == null ? "null" : this.getCurrentCommand().getName();
    }

    // Logs System ID
    private void sysIDLogging(SysIdRoutineLog log) {
        log.motor("ShooterWheel")
                .voltage(getVoltage())
                .current(motorLeader.getStatorCurrent().getValue())
                .angularVelocity(getVelocity())
                .angularPosition(motorLeader.getPosition().getValue());
    }
}