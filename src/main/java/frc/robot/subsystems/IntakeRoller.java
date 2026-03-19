package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;

public class IntakeRoller extends SubsystemBase {

    // Motor
    private final TalonFX motor;

    public class IntakeRollerTest implements Sendable {

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
            builder.addDoubleProperty("Intake Roller Test Speed (RPM)", () -> velRPM, (val) -> velRPM = val);
            builder.addDoubleProperty("Intake Roller Test kP", () -> kP, (val) -> kP = val);
            builder.addDoubleProperty("Intake Roller Test kI", () -> kI, (val) -> kI = val);
            builder.addDoubleProperty("Intake Roller Test kD", () -> kD, (val) -> kD = val);
            builder.addDoubleProperty("Intake Roller Test kS", () -> kS, (val) -> kS = val);
            builder.addDoubleProperty("Intake Roller Test kV", () -> kV, (val) -> kV = val);
            builder.addDoubleProperty("Intake Roller Test kA", () -> kA, (val) -> kA = val);
        }

        /**
         * Gets a command to control the intake roller from
         * 
         * @return
         */
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

    // Motor Control Requests
    private final VoltageOut voltCtrl = new VoltageOut(0.0);
    private final MotionMagicVelocityVoltage velCtrl = new MotionMagicVelocityVoltage(0)
            .withAcceleration(Rotations.per(Minute).per(Second).of(10000));
    private final IntakeRollerTest intakeRollerTest = new IntakeRollerTest();
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // SysId
    private final SysIdRoutine sysIdRoutime = new SysIdRoutine(
            new SysIdRoutine.Config(null,
                    Volts.of(4),
                    null,
                    (state) -> Logger.recordOutput("Intake Roller State", state.toString())),
            new SysIdRoutine.Mechanism(
                    this::setVoltage,
                    null,
                    this));

    /**
     * Constructor
     * 
     * @param motorID
     */
    public IntakeRoller(int motorId, CANBus bus, double gearRatio) {
        motor = new TalonFX(motorId, bus);

        motorConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        motorConfig.Feedback
                .withSensorToMechanismRatio(gearRatio);

        motorConfig.Slot0
                .withKP(0.16454)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.21962)
                .withKV(0.16367)
                .withKA(0.019017);

        motorConfig.Slot2
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

    @AutoLogOutput
    public Angle getPosition() {
        return motor.getPosition().getValue();
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

    /**
     * Creates a new command to run the intake at a set target velocity
     * 
     * @param volts target velocity
     * @return new command to run the intake at a set target velocity
     */
    public Command setVelocityCmd(AngularVelocity velocity) {
        return this.runEnd(
                () -> setVelocity(velocity),
                () -> setVelocity(RotationsPerSecond.zero()));
    }

    public Command intakeCmd() {
        return setVoltageCmd(Constants.intakeInVolt);
    }

    public Command ejectCommand() {
        return setVoltageCmd(Constants.intakeOutVolt);
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

    public Command sysIDSequence() {
        return Commands.sequence(
                sysIdQuasistatic(Direction.kReverse),
                sysIdQuasistatic(Direction.kForward),
                sysIdDynamic(Direction.kReverse),
                sysIdDynamic(Direction.kForward));
    }

    /**
     * Update intake RPM on dashboard
     */
    @Override
    public void periodic() {
        // TODO Remove and use CTRE or AdvantageKit telemetry
        // SmartDashboard.putNumber("Intake RPM",
        // getVelocity().in(Rotations.per(Minute)));

        // motorConfig.Slot2
        // .withKP(intakeRollerTest.getkP())
        // .withKI(intakeRollerTest.getkI())
        // .withKD(intakeRollerTest.getkD())
        // .withKS(intakeRollerTest.getkS())
        // .withKV(intakeRollerTest.getkV())
        // .withKA(intakeRollerTest.getkA());
        // motor.getConfigurator().refresh(motorConfig);
    }

    @AutoLogOutput
    public String getCommandString() {
        return this.getCurrentCommand() == null ? "null" : this.getCurrentCommand().getName();
    }
}
