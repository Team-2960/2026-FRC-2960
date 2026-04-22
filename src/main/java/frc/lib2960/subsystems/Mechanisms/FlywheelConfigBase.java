package frc.lib2960.subsystems.mechanisms;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib2960.util.MotorGroupConfig;

public class FlywheelConfigBase {
    public String name;
    public MotorGroupConfig motorGroupConfigs = new MotorGroupConfig();
    public AngularVelocity maxVel = RotationsPerSecond.zero();
    public AngularAcceleration maxAccel = RotationsPerSecondPerSecond.zero();
    
    /**
     * Constructor
     * 
     * @param name Name of the flywheel mechanism
     */
    public FlywheelConfigBase(String name) {
        this.name = name;
    }

    /**
     * Creates and configures the talonFX motors from this configuration. The first motor in the list of 
     * @return  list of talonFX motors
     * @throws RuntimeException if no motor configurations have been set
     */
    public ArrayList<TalonFX> createTalonFXs() {
        // TODO Add Flywheel specific configuration
        return motorGroupConfigs.createTalonFXs();
    }

    public TalonFXConfiguration createTalonFXConfig() {
        var config = motorGroupConfigs.createTalonFXConfig();

        // Zero out gravity feed forward values
        config.Slot0.withKG(0);
        config.Slot1.withKG(0);
        config.Slot2.withKG(0);

        return config;
    }
}
