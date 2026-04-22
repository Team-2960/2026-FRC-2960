package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib2960.subsystems.mechanisms.FlywheelBase;
import frc.lib2960.subsystems.mechanisms.FlywheelConfigBase;
import frc.lib2960.util.ControlConfig;
import frc.lib2960.util.MotorConfig;
import frc.lib2960.util.MotorGroupConfig;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.Util.ShotSpeedTable;

public class ShooterWheel extends FlywheelBase {
    // Motor Configuration
    private static final MotorGroupConfig shooterWheelMotorGroupConfig = new MotorGroupConfig()
            .addMotorConfig(
                    new MotorConfig(Constants.shooterMotorLeaderID)
                            .withCANBus(Constants.canivoreBus)
                            .withInverted(false))
            .addMotorConfig(
                    new MotorConfig(Constants.shooterMotorFollowerID)
                            .withCANBus(Constants.canivoreBus)
                            .withInverted(true))
            .withBrakeMode(true)
            .withMotorMechanismRatio(20.0 / 12.0)
            .withControlConfig(new ControlConfig()
                    .withKP(0.005)
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKS(0.28683)
                    .withKV(0.11886)
                    .withKA(0.0067811),
                    0)
            .withControlConfig(new ControlConfig()
                    .withKP(15)
                    .withKI(0.0)
                    .withKD(0.0)
                    .withKS(.03)
                    .withKV(0.3)
                    .withKA(0),
                    1);

    // Flywheel Configuration
    private static final FlywheelConfigBase shooterWheelConfig = new FlywheelConfigBase("Shooter Wheel")
            .withMotorGroupConfig(shooterWheelMotorGroupConfig)
            .withMaxAcceleration(Rotations.per(Minute).per(Second).of(6209));

    // Shooter Presets
    private static final int voltageCtrlSlot = 0;
    private static final int currentCtrlSlot = 1;
    private static final ShotSpeedTable hubShotTable = new ShotSpeedTable()
            .addEntry(Meters.of(1.71), Revolution.per(Minute).of(1550))
            .addEntry(Meters.of(2.21), Revolution.per(Minute).of(1650))
            .addEntry(Meters.of(2.50), Revolution.per(Minute).of(1725))
            .addEntry(Meters.of(2.8), Revolution.per(Minute).of(1775))
            .addEntry(Meters.of(3.1), Revolution.per(Minute).of(1850))
            .addEntry(Meters.of(3.4), Revolution.per(Minute).of(1950))
            .addEntry(Meters.of(3.7), Revolution.per(Minute).of(2100))
            .addEntry(Meters.of(4.0), Revolution.per(Minute).of(2300))
            .addEntry(Meters.of(4.3), Revolution.per(Minute).of(2450));

    private static final AngularVelocity passVelocity = Rotations.per(Minute).of(1500);
    private static final AngularVelocity idleVelocity = Rotations.per(Minute).of(1600);
    private static final AngularVelocity towerShotVelocity = Rotations.per(Minute).of(1900);

    /**
     * Constructor
     * 
     * @param config Flywheel Configuration
     */
    public ShooterWheel(FlywheelConfigBase config) {
        super(config);
    }

    /**
     * Creates a command to run shooter at the idle velocity
     * 
     * @return command to run shooter at the idle velocity
     */
    public Command idleVelocityCmd() {
        return setTorqueVelocityCmd(idleVelocity, currentCtrlSlot);
    }

    /**
     * Creates a command to run shooter at the pass velocity
     * 
     * @return command to run shooter at the pass velocity
     */
    public Command passVelocityCmd() {
        return setTorqueVelocityCmd(passVelocity, currentCtrlSlot);
    }

    /**
     * Creates a command to run shooter at the tower shot velocity
     * 
     * @return command to run shooter at the tower shot velocity
     */
    public Command towerShotVelocityCmd() {
        return setTorqueVelocityCmd(towerShotVelocity, currentCtrlSlot);
    }
    
    public Command hubShotCmd(Supplier<Pose2d> currentPose) {

    }

    

    /**
     * Calculates the angular velocity for shooting into the hub from the current
     * robot position
     * 
     * @return target angular velocity
     */
    private AngularVelocity calcHubShotSpeed(Pose2d currentPose) {
        Distance hubDist = FieldLayout.Hub.getHubDist(currentPose.getTranslation());

        return hubShotTable.get(hubDist);
    }
}
