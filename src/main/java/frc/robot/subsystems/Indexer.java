package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {

    // Motor
    private final TalonFX motor;

    // Motor Control Requests
    private final VoltageOut voltCtrl = new VoltageOut(0.0).withEnableFOC(true);
    private final MotionMagicVelocityVoltage velCtrl = new MotionMagicVelocityVoltage(0)
            .withAcceleration(Constants.indexerMaxAccel)
            .withEnableFOC(true)
            .withSlot(0);
    private final MotionMagicVelocityTorqueCurrentFOC torqueCtrl = new MotionMagicVelocityTorqueCurrentFOC(0)
            .withAcceleration(Constants.indexerMaxAccel)
            .withSlot(1);
            
    private final TorqueCurrentFOC currentCtrl = new TorqueCurrentFOC(0);

    private final CommandSwerveDrivetrain drivetrain;

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

    private SysIdRoutine currentSysIdRoutine = sysIdRoutine3;
    /**
     * Constructor
     * 
     * @param motorID
     */
    public Indexer(int motorId, CANBus bus, double gearRatio, CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        motor = new TalonFX(motorId, bus);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);

        motorConfig.Feedback
                .withSensorToMechanismRatio(gearRatio);

        motorConfig.MotionMagic.withMotionMagicJerk(Constants.indexerMaxAccel.in(RotationsPerSecondPerSecond) * 10);

        motorConfig.Slot0
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0);

        motorConfig.Slot1
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0);

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
     * Gets the current voltage of the indexer
     * 
     * @return
     */
    @AutoLogOutput
    public Voltage getVoltage() {
        return motor.getMotorVoltage().getValue();
    }

    public void setTorqueCurrentVel(AngularVelocity velocity) {
        motor.setControl(torqueCtrl.withVelocity(velocity));
    }

    /**
     * Gets the current voltage of the intake
     * 
     * @return
     */
    @AutoLogOutput
    public Current getStatorCurrent(){
        return motor.getStatorCurrent().getValue();
    }

    /**
     * Gets the current voltage of the intake
     * 
     * @return
     */
    @AutoLogOutput
    public Current getSupplyCurrent(){
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
     * Creates a new command to run the indexer at a set target voltage
     * 
     * @param volts target voltage
     * @return new command to run the indexer at a set target voltage
     */
    public Command setVoltageCmd(Voltage volts) {
        return this.runEnd(
                () -> setVoltage(volts),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the indexer at a set target velocity
     * 
     * @param volts target velocity
     * @return new command to run the indexer at a set target velocity
     */
    public Command setVelocityCmd(AngularVelocity velocity) {
        return this.runEnd(
                () -> setVelocity(velocity),
                () -> setVelocity(RotationsPerSecond.zero()));
    }

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

    public Command runShooterFeedCmd() {
        return setVoltageCmd(Constants.indexerFeedVolt);
    }

    public Command stopShooterFeedCmd() {
        return setVoltageCmd(Volts.zero());
    }


    public Command autoIndexCmd(BooleanSupplier enabled){
        return this.runEnd(() -> {
            if (enabled.getAsBoolean()) {
                setVoltage(Constants.indexerFeedVolt);
            }else{
                setVoltage(Volts.zero());
            }
        }, 
        () -> setVoltage(Volts.zero()));
    }

    /**
     * Create a Quasistatic SysId command
     * @param direction direction of the command
     * @return  Quasistatic SysId command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return currentSysIdRoutine.quasistatic(direction);
    }


    /**
     * Create a Dynamic SysId command
     * @param direction direction of the command
     * @return  Dynamic SysId command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return currentSysIdRoutine.dynamic(direction);
    }

    /**
     * Update indexer RPM on dashboard
     */
    @Override
    public void periodic() {
        // TODO Remove and use CTRE or AdvantageKit telemetry
        // SmartDashboard.putNumber("Indexer RPM", getVelocity().in(Rotations.per(Minute)));
    }

    @AutoLogOutput
    public String getCommandString() {
        return this.getCurrentCommand() == null ? "null" : this.getCurrentCommand().getName();
    }

    private void sysIDLogging(SysIdRoutineLog log) {
        log.motor("ShooterWheel")
                .voltage(getVoltage())
                .current(motor.getStatorCurrent().getValue())
                .angularVelocity(getVelocity())
                .angularPosition(motor.getPosition().getValue());
    }
}
