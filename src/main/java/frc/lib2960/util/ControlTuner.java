package frc.lib2960.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Provides Driver Station interface for updating control config
 */
public class ControlTuner implements Sendable {
    /** Control Config for the tuner */
    public final ControlConfig config;

    /**
     * Constructor
     * 
     * @param config Control config for the tuner
     */
    public ControlTuner(ControlConfig config) {
        this.config = config;
    }

    /**
     * Constructor. The controls config will be set to default.
     */
    public ControlTuner() {
        this(new ControlConfig());
    }

    /**
     * Initialize the sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("kP", config::getKP, config::withKP);
        builder.addDoubleProperty("kI", config::getKI, config::withKI);
        builder.addDoubleProperty("kD", config::getKD, config::withKD);
        builder.addDoubleProperty("kS", config::getKS, config::withKS);
        builder.addDoubleProperty("kV", config::getKV, config::withKV);
        builder.addDoubleProperty("kG", config::getKG, config::withKG);
        builder.addDoubleProperty("kA", config::getKA, config::withKA);
    }
}
