package frc.lib2960.util;

public class SensorConfig {
    /** CAN ID for the sensor */
    public final int id;

    /** Gear ratio between the sensor and the mechanism */
    public double sensorMechRatio = 1.0;


    /**
     * Constructor
     * @param id
     */
    public SensorConfig(int id) {
        this.id = id;
    }

    /**
     * Sets the sensor to mechanism gear ratio
     * @param ratio
     * @return reference to this object
     */
    public SensorConfig withSensorMechRatio(double ratio) {
        this.sensorMechRatio = ratio;
        return this;
    }
}
