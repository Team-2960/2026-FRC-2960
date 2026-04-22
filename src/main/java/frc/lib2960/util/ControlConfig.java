package frc.lib2960.util;

/**
 * Stores the PID and Feed forward values for a controller
 */
public class ControlConfig {
    /** Proportional Gain */
    public double kP = 0;

    /** Integral Gain */
    public double kI = 0;

    /** Differential Gain */
    public double kD = 0;

    /** Static Feed Forward Gain */
    public double kS = 0;

    /** Velocity Feed Forward Gain */
    public double kV = 0;

    /** Gravity Feed Forward Gain */
    public double kG = 0;

    /** Acceleration Feed Forward Gain */
    public double kA = 0;

    /**
     * Sets the P gain value
     * 
     * @param value value to set
     * @return reference to this object
     */
    public ControlConfig withKP(double value) {
        this.kP = value;
        return this;
    }

    /**
     * Sets the I gain value
     * 
     * @param value value to set
     * @return reference to this object
     */
    public ControlConfig withKI(double value) {
        this.kI = value;
        return this;
    }

    /**
     * Sets the D gain value
     * 
     * @param value value to set
     * @return reference to this object
     */
    public ControlConfig withKD(double value) {
        this.kD = value;
        return this;
    }

    /**
     * Sets the S gain value
     * 
     * @param value value to set
     * @return reference to this object
     */
    public ControlConfig withKS(double value) {
        this.kS = value;
        return this;
    }

    /**
     * Sets the V gain value
     * 
     * @param value value to set
     * @return reference to this object
     */
    public ControlConfig withKV(double value) {
        this.kV = value;
        return this;
    }

    /**
     * Sets the G gain value
     * 
     * @param value value to set
     * @return reference to this object
     */
    public ControlConfig withKG(double value) {
        this.kG = value;
        return this;
    }

    /**
     * Sets the A gain value
     * 
     * @param value value to set
     * @return reference to this object
     */
    public ControlConfig withKA(double value) {
        this.kA = value;
        return this;
    }

    /**
     * Gets the P gain value
     * 
     * @param value value to set
     */
    public double getKP() {
        return kP;
    }

    /**
     * Gets the I gain value
     * 
     * @param value value to set
     */
    public double getKI() {
        return kI;
    }

    /**
     * Gets the D gain value
     * 
     * @param value value to set
     */
    public double getKD() {
        return kD;
    }

    /**
     * Gets the S gain value
     * 
     * @param value value to set
     */
    public double getKS() {
        return kS;
    }

    /**
     * Gets the V gain value
     * 
     * @param value value to set
     */
    public double getKV() {
        return kV;
    }

    /**
     * Gets the G gain value
     * 
     * @param value value to set
     */
    public double getKG() {
        return kG;
    }

    /**
     * Gets the A gain value
     * 
     * @param value value to set
     */
    public double getKA() {
        return kA;
    }
}
