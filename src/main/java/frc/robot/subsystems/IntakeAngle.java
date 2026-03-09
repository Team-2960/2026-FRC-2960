package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class IntakeAngle extends SubsystemBase {

    public class IntakeAngleTest implements Sendable {

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
            builder.addDoubleProperty("Intake Angle Test Speed (RPM)", ()->velRPM , (val) -> velRPM = val);
            builder.addDoubleProperty("Intake Angle Test kP", ()->kP , (val) -> kP = val);
            builder.addDoubleProperty("Intake Angle Test kI", ()->kI , (val) -> kI = val);
            builder.addDoubleProperty("Intake Angle Test kD", ()->kD , (val) -> kD = val);
            builder.addDoubleProperty("Intake Angle Test kS", ()->kS , (val) -> kS = val);
            builder.addDoubleProperty("Intake Angle Test kV", ()->kV , (val) -> kV = val);
            builder.addDoubleProperty("Intake Angle Test kA", ()->kA , (val) -> kA = val);
        }

        public double getkP(){
            return kP;
        }
        public double getkI(){
            return kI;
        }
        public double getkD(){
            return kD;
        }
        public double getkS(){
            return kS;
        }
        public double getkV(){
            return kV;
        }
        public double getkA(){
            return kA;
        }
        
    }

    // Motor
    private final TalonFX motor;
    private final CANcoder encoder;

    // Motor Control Requests
    private final VoltageOut voltCtrl = new VoltageOut(0.0);
    private final MotionMagicVelocityVoltage velCtrl = new MotionMagicVelocityVoltage(0).withAcceleration(RotationsPerSecondPerSecond.of(10));
    private final MotionMagicVoltage posCtrl = new MotionMagicVoltage(0);
    private final IntakeAngleTest intakeAngleTest = new IntakeAngleTest();
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

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
    public IntakeAngle(int motorId, int encoderId, CANBus bus, double gearRatio) {
        motor = new TalonFX(motorId, bus);
        encoder = new CANcoder(encoderId, bus);

        motorConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive);

        motorConfig.Feedback
                .withSensorToMechanismRatio(1)
                .withRemoteCANcoder(encoder)
                .withRotorToSensorRatio(gearRatio)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder);

        motorConfig.Slot0
                .withKP(1.8)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.21456)
                .withKV(4.7628)
                .withKA(0.18916)
                .withKG(0.32474);

        motorConfig.Slot2
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0)
                .withKG(0.0);

        motorConfig.MotionMagic
            .withMotionMagicCruiseVelocity(RotationsPerSecond.of(6))
            .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(12))
            .withMotionMagicJerk(120);


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
        motor.setControl(posCtrl.withPosition(angle)
            .withLimitReverseMotion(getPosition().lte(Degrees.of(-10)))
            .withLimitForwardMotion(getPosition().gte(Degrees.of(150))));
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

        public Command setVelocityTestCmd(Supplier<AngularVelocity> velocity){
        return this.startRun(
            () -> motor.getConfigurator().refresh(motorConfig.Slot2), 
            () -> setVelocity(velocity.get()))
            .finallyDo(() -> setVoltage(Volts.zero()));
    }

    /**
     * Creates a new command to run the intake at a set target velocity
     * 
     * @param target target velocity
     * @return new command to run the intake at a set target velocity
     */
    public Command setPositionCmd(Angle target) {
        return this.runEnd(
                () -> setPosition(target),
                () -> setVoltage(Volts.zero()));
    }

    /**
     * Create a Quasistatic SysId command
     * @param direction direction of the command
     * @return  Quasistatic SysId command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Create a Quasistatic SysId command
     * @param direction direction of the command
     * @return  Quasistatic SysId command
     */
    public Command sysIdQuasistaticLimited(SysIdRoutine.Direction direction, Angle min, Angle max) {
        if (direction.equals(Direction.kForward)){
            return sysIdRoutine.quasistatic(direction)
                .until(() -> getPosition().lte(min));
        }else{
            return sysIdRoutine.quasistatic(direction)
                .until(() -> getPosition().gte(max));
        }
    }


    /**
     * Create a Quasistatic SysId command
     * @param direction direction of the command
     * @return  Quasistatic SysId command
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction, Angle lowerLimit, Angle upperLimit) {
        return sysIdRoutime.quasistatic(direction);
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
                .until(() -> getPosition().lte(Degrees.of(0)));
        }else{
            return sysIdRoutine.dynamic(direction)
                .until(() -> getPosition().gte(Degrees.of(90)));
        }
    }

    /**
     * Update intake RPM on dashboard
     */
    @Override
    public void periodic() {
        // TODO Remove and use CTRE or AdvantageKit telemetry
        // SmartDashboard.putNumber("Intake Angle", getPosition().in(Degrees));
        // SmartDashboard.putNumber("Intake Angle RPM", getVelocity().in(Rotations.per(Minute)));
        SmartDashboard.putNumber("Intake Angle", getPosition().in(Degrees));
        SmartDashboard.putData("Intake Angle Tuning", intakeAngleTest);

        motorConfig.Slot2
        .withKP(intakeAngleTest.getkP())
        .withKI(intakeAngleTest.getkI())
        .withKD(intakeAngleTest.getkD())
        .withKS(intakeAngleTest.getkS())
        .withKV(intakeAngleTest.getkV())
        .withKA(intakeAngleTest.getkA());
    }

    @AutoLogOutput
    public String getCommandString() {
        return this.getCurrentCommand() == null ? "null" : this.getCurrentCommand().getName();
    }
}