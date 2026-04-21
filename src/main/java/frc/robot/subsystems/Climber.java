package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.SignalLogger;
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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  
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

    private Orchestra orchestra = new Orchestra();

    // SysId
    private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.5).per(Second),
                    Volts.of(2),
                    null,
                    (state) -> SignalLogger.writeString("IntakeAngle State", state.toString())),
            new SysIdRoutine.Mechanism(
                    this::setVoltage,
                    null,
                    this));

    /**
     * Constructor
     * 
     * @param motorID
     */
    public Climber(int motorLeaderID, int motorFollowerID, CANBus bus, double gearRatio) {

         // Initialize Motors
        motorLeader = new TalonFX(motorLeaderID, bus);
        motorFollower = new TalonFX(motorFollowerID, bus);

        motorConfig.CurrentLimits
            .withSupplyCurrentLimit(Amps.of(80))
            .withSupplyCurrentLimitEnable(true);

        motorConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);

        motorConfig.Feedback
                .withSensorToMechanismRatio(gearRatio);

        motorConfig.Slot0
                .withKP(0)
                .withKI(0)
                .withKD(0)
                .withKS(0)
                .withKV(0)
                .withKA(0)
                .withKG(0);

        motorLeader.getConfigurator().apply(motorConfig);
        motorFollower.getConfigurator().apply(motorConfig);

        motorFollower.setControl(new Follower(motorLeaderID, MotorAlignmentValue.Opposed));

        motorLeader.getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(true));
        motorFollower.getConfigurator().apply(new AudioConfigs().withAllowMusicDurDisable(true));
    }

    /**
     * Sets a target voltage to the motor
     * 
     * @param volts target voltage
     */
    public void setVoltage(Voltage volts) {
        motorLeader.setControl(voltCtrl.withOutput(volts));
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
     * Gets the current velocity of the motor
     * 
     * @return current velocity of the motor
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
     * Creates a new command to run the intake at a set target voltage
     * 
     * @param volts target voltage
     * @return new command to run the intake at a set target voltage
     */
    public Command setVoltageCmd(Voltage volts) {
        return this.runEnd(
                () -> setVoltage(volts),
                () -> setVoltage(Volts.zero()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Create a Quasistatic SysId command
     * @param direction direction of the command
     * @return  Quasistatic SysId command
     */
    public Command sysIdQuasistaticLimited(SysIdRoutine.Direction direction) {
        if (direction.equals(Direction.kForward)){
            return sysIdRoutine.quasistatic(direction)
                .until(() -> getPosition().lte(Rotations.of(0))); // Tune Max Rotations
        }else{
            return sysIdRoutine.quasistatic(direction)
                .until(() -> getPosition().gte(Rotations.of(0))); // Tune Min Rotations
        }
    }

    /**
     * Create a Dynamic SysId command
     * @param direction direction of the command
     * @return  Dynamic SysId command
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    public Command sysIdDynamicLimited(SysIdRoutine.Direction direction) {
        if (direction.equals(Direction.kForward)){
            return sysIdRoutine.dynamic(direction)
                .until(() -> getPosition().lte(Rotations.of(0))); // Tune Max Rotations
        }else{
            return sysIdRoutine.dynamic(direction)
                .until(() -> getPosition().gte(Rotations.of(0))); // Tune Min Rotations
        }
    }

    /**
     * Update intake RPM on dashboard
     */
    @Override
    public void periodic() {
        // TODO Remove and use CTRE or AdvantageKit telemetry
        SmartDashboard.putNumber("Climber RPM", getVelocity().in(Rotations.per(Minute)));
    }

    @AutoLogOutput
    public String getCommandString() {
        return this.getCurrentCommand() == null ? "null" : this.getCurrentCommand().getName();
    }
}
