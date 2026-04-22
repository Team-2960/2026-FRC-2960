package frc.lib2960.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * Details the configuration for a motor group
 */
public class MotorConfig {
    /** Motor ID */
    public final int id;

    /** Motor CAN Bus */
    public CANBus bus = new CANBus();

    /**
     * Sets if the motor should be inverted. Counter-Clockwise is the non-inverted
     * positive direction.
     */
    public boolean inverted = false;

    /**
     * Constructor.
     * 
     * @param id CAN ID of the motor
     */
    public MotorConfig(int id) {
        this.id = id;
    }

    /**
     * Sets the motor CAN Bus
     * 
     * @param bus motor CAN bus
     * @return reference to this object
     */
    public MotorConfig withCANBus(CANBus bus) {
        this.bus = bus;
        return this;
    }

    /**
     * Sets if the motor should be inverted. Counter-Clockwise is the non-inverted
     * positive direction.
     * 
     * @param inverted inverted value
     * @return reference to this object
     */
    public MotorConfig withInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    /**
     * Creates a talon FX from the configured values
     * 
     * @return new TalonFX
     */
    public TalonFX createTalonFX() {
        return new TalonFX(id, bus);
    }
}
