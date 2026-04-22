package frc.lib2960.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Sendable class for setting output values for a subsystem motor
 */
public class OutputTuner implements Sendable {

    /** Target Voltage */
    public MutVoltage voltage = Volts.mutable(0);

    /** Target Current */
    public MutCurrent current = Amps.mutable(0);

    /** Target Velocity */
    public MutAngularVelocity velocity = Rotations.per(Minute).mutable(0);

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
                "Voltage (V)",
                () -> voltage.in(Volts),
                (val) -> voltage.mut_replace(val, Volts));
        builder.addDoubleProperty(
                "Current (A)",
                () -> current.in(Amps),
                (val) -> current.mut_replace(val, Amps));
        builder.addDoubleProperty(
                "Velocity (RPM)",
                () -> velocity.in(Rotations.per(Minute)),
                (val) -> velocity.mut_replace(val, Rotations.per(Minute)));
    }
}
