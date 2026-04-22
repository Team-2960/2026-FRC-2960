package frc.lib2960.util;

import static edu.wpi.first.units.Units.Amps;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Current;

/**
 * Configuration for a group of motors
 */
public class MotorGroupConfig {
    /**
     * List of motor configurations for each motor in the mechanism. The first motor
     * will be set as the leader.
     */
    public ArrayList<MotorConfig> motorConfigs = new ArrayList<>();

    /** List of control slot values */
    public final ArrayList<ControlConfig> controlConfigs = new ArrayList<>(
            Arrays.asList(new ControlConfig(), new ControlConfig(), new ControlConfig()));

    /** Sets the motor's neutral mode to Brake when true */
    public boolean brakeMode = false;

    /** Gear ratio between the mechanism and the motors */
    public double motorMechRatio = 1;

    /** Sets the maximum supply current for all the motors in the group */
    public Optional<Current> supplyCurrentLimit = Optional.empty();

    /** Sets the maximum stator current for all the motors in the group */
    public Optional<Current> statorCurrentLimit = Optional.empty();

    /**
     * Sets the motor config list
     * 
     * @param configs list of motor configs
     * @return reference to this object
     */
    public MotorGroupConfig withMotorConfigs(ArrayList<MotorConfig> configs) {
        this.motorConfigs = configs;
        return this;
    }

    /**
     * Sets the motor config list
     * 
     * @param configs list of motor configs
     * @return reference to this object
     */
    public MotorGroupConfig withMotorConfigs(MotorConfig... configs) {
        this.motorConfigs = new ArrayList<>(Arrays.asList(configs));
        return this;
    }

    /**
     * Adds the motor config to the list
     * 
     * @param configs motor configs to add
     * @return reference to this object
     */
    public MotorGroupConfig addMotorConfig(MotorConfig config) {
        this.motorConfigs.add(config);
        return this;
    }

    /**
     * Sets a Control Configuration slot
     * 
     * @param config control configuration to set
     * @param slot   slot to set the control configuration to
     * @return reference to this object
     * @throws RuntimeException if slot is not between 0 and 2 inclusive
     */
    public MotorGroupConfig withControlConfig(ControlConfig config, int slot) {
        // TODO Generalize to allow more than 3 control slots
        if (slot >= 0 && slot < 3) {
            controlConfigs.set(slot, config);
        } else {
            throw new RuntimeException("Invalid Slot Selected of " + slot + " used for Control Config");
        }

        return this;
    }

    /**
     * Sets the motor's neutral mode to Brake when true
     * 
     * @param brakeMode brakeMode value
     * @return reference to this object
     */
    public MotorGroupConfig withBrakeMode(boolean brakeMode) {
        this.brakeMode = brakeMode;
        return this;
    }

    /**
     * Sets the gear ratio between the motor and the mechanism
     * 
     * @param ratio gear ratio
     * @return reference to this object
     */
    public MotorGroupConfig withMotorMechanismRatio(double ratio) {
        this.motorMechRatio = ratio;
        return this;
    }

    public MotorGroupConfig withSupplyCurrentLimit(Current limit) {
        this.supplyCurrentLimit = Optional.of(limit);
        return this;
    }

    public MotorGroupConfig clearSupplyCurrentLimit() {
        this.supplyCurrentLimit = Optional.empty();
        return this;
    }

    public MotorGroupConfig withStatorCurrentLimit(Current limit) {
        this.statorCurrentLimit = Optional.of(limit);
        return this;
    }

    public MotorGroupConfig clearStatorCurrentLimit() {
        this.statorCurrentLimit = Optional.empty();
        return this;
    }

    /**
     * Creates and configures the talonFX motors from this configuration. The first
     * motor in the list of
     * 
     * @return list of talonFX motors
     * @throws RuntimeException if no motor configurations have been set
     */
    public ArrayList<TalonFX> createTalonFXs() {
        // Ensure at least 1 motor config is present
        if (motorConfigs.size() < 1)
            throw new RuntimeException("No motors configured");

        // Create motors
        var motors = new ArrayList<TalonFX>(motorConfigs.size());

        for (var motorConfig : motorConfigs)
            motors.add(motorConfig.createTalonFX());

        return motors;
    }

    /**
     * Creates a TalonFX motor config for the lead motor
     * 
     * @return motor config for the lead motor
     */
    public TalonFXConfiguration createTalonFXConfig() {
        // Configure Leader Motor
        var config = new TalonFXConfiguration();

        // Set Control Value Slots
        config.withSlot0(applySlot0Config());
        config.withSlot1(applySlot1Config());
        config.withSlot2(applySlot2Config());

        // Set Brake Mode
        config.MotorOutput
                .withNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);

        // Set Feedback ratio
        config.Feedback
                .withSensorToMechanismRatio(motorMechRatio);

        // Set Current Limits
        if (supplyCurrentLimit.isPresent()) {
            config.CurrentLimits
                    .withSupplyCurrentLimit(supplyCurrentLimit.get().in(Amps))
                    .withSupplyCurrentLimitEnable(true);
        }

        if (statorCurrentLimit.isPresent()) {
            config.CurrentLimits
                    .withStatorCurrentLimit(statorCurrentLimit.get().in(Amps))
                    .withStatorCurrentLimitEnable(true);
        }

        return config;
    }

    /**
     * Creates a new Slot 0 config from the slot 0 settings
     * 
     * @return new slot 0 config
     */
    public Slot0Configs applySlot0Config() {
        var ctrlCfg = controlConfigs.get(0);
        var slotConfig = new Slot0Configs()
                .withKP(ctrlCfg.kP)
                .withKI(ctrlCfg.kI)
                .withKD(ctrlCfg.kD)
                .withKS(ctrlCfg.kS)
                .withKV(ctrlCfg.kV)
                .withKG(ctrlCfg.kG)
                .withKA(ctrlCfg.kA);

        return slotConfig;
    }

    /**
     * Creates a new Slot 1 config from the slot 1 settings
     * 
     * @return new slot 1 config
     */
    public Slot1Configs applySlot1Config() {
        var ctrlCfg = controlConfigs.get(1);
        var slotConfig = new Slot1Configs()
                .withKP(ctrlCfg.kP)
                .withKI(ctrlCfg.kI)
                .withKD(ctrlCfg.kD)
                .withKS(ctrlCfg.kS)
                .withKV(ctrlCfg.kV)
                .withKG(ctrlCfg.kG)
                .withKA(ctrlCfg.kA);

        return slotConfig;
    }

    /**
     * Creates a new Slot 2 config from the slot 2 settings
     * 
     * @return new slot 2 config
     */
    public Slot2Configs applySlot2Config() {
        var ctrlCfg = controlConfigs.get(2);
        var slotConfig = new Slot2Configs()
                .withKP(ctrlCfg.kP)
                .withKI(ctrlCfg.kI)
                .withKD(ctrlCfg.kD)
                .withKS(ctrlCfg.kS)
                .withKV(ctrlCfg.kV)
                .withKG(ctrlCfg.kG)
                .withKA(ctrlCfg.kA);

        return slotConfig;
    }
}
