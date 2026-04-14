package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FieldLayout;

public class ShooterManagement {

    private final CommandSwerveDrivetrain drivetrain;
    private final Indexer indexer;
    private final ShooterWheel shooterWheel;
    // private final ShooterHood shooterHood;
    private final IntakeRoller intakeRoller;
    private final IntakeAngle intakeAngle;

    /**
     * Constructor
     * 
     * @param indexer      indexer subsystem instance
     * @param shooterWheel shooter wheel subsystem instance
     * @param shooterHood  shooter hood subsystem instance
     */
    public ShooterManagement(CommandSwerveDrivetrain drivetrain, Indexer indexer, ShooterWheel shooterWheel,
            IntakeRoller intakeRoller, IntakeAngle intakeAngle) {
        this.drivetrain = drivetrain;
        this.indexer = indexer;
        this.shooterWheel = shooterWheel;
        // this.shooterHood = shooterHood;
        this.intakeRoller = intakeRoller;
        this.intakeAngle = intakeAngle;
    }

    public Command hubNoIntakeShotCmd() {
        return Commands.parallel(
                shooterWheel.hubShotCmd(),
                indexer.autoIndexCmd(() -> isShooterReady()));
    }

    public Command hubAutoShotCmd() {
        return Commands.parallel(
                shooterWheel.hubShotCmd(),
                intakeRoller.intakeInCmd(),
                intakeAngle.rushOscillate(),
                indexer.autoIndexCmd(() -> isShooterReady()));
    }
    
    public Command passAutoShotCmd() {
        return Commands.parallel(
                shooterWheel.passShotCmd(),
                intakeRoller.intakeInCmd(),
                intakeAngle.rushOscillate(),
                indexer.autoIndexCmd(() -> isShooterReady()));
    }

    public Command hubIndexAutoShotCmd() {
        return Commands.parallel(
                shooterWheel.hubShotCmd(),
                intakeRoller.intakeInCmd(),
                intakeAngle.rushOscillate(),
                indexer.indexReverseCmd().withTimeout(0.1).andThen(indexer.autoContinuousIndexCmd(() -> isShooterReady())));
    }

    public Command passIndexAutoShotCmd() {
        return Commands.parallel(
                shooterWheel.passShotCmd(),
                intakeRoller.intakeInCmd(),
                intakeAngle.rushOscillate(),
                indexer.autoContinuousIndexCmd(() -> isShooterReady()));
    }

    public Command hubNoIntakeIndexAutoShotCmd() {
        return Commands.parallel(
                shooterWheel.hubShotCmd(),
                intakeRoller.intakeInCmd(),
                indexer.autoContinuousIndexCmd(() -> isShooterReady()));
    }

    public Command hubAutoHighShotCmd() {
        return Commands.parallel(
                shooterWheel.hubShotCmd(),
                intakeRoller.intakeInCmd(),
                intakeAngle.highOscillate(),
                indexer.autoIndexCmd(() -> isShooterReady(Constants.shooterWheelFloorThreshold,
                        Constants.shooterWheelCeilingThreshold)));
    }

    public Command hubBangBangShotCmd() {
        return Commands.parallel(
                shooterWheel.hubShotCmd(),
                intakeRoller.intakeInCmd(),
                intakeAngle.clamShellOscilate(),
                indexer.autoContinuousIndexCmd(() -> isShooterReady()));
    }

    public Command hubSequentialShotCmd() {
        return Commands.parallel(
                shooterWheel.hubShotCmd(),
                intakeRoller.intakeInCmd(),
                this.intakeSequentialCmd(),
                indexer.autoContinuousIndexCmd(() -> isShooterReady()));
    }

    public Command intakeSequentialCmd() {
        return Commands.sequence(
                this.intakeOscilateRaceCmd(),
                intakeAngle.clamShellOscilate());
    }

    public Command intakeOscilateRaceCmd() {
        return intakeAngle.rushOscillate().withTimeout(1.5);
    }

    public Command intakeHighOscilateRaceCmd() {
        return intakeAngle.highOscillate().withTimeout(1);
    }

    public Command passShotCmd() {
        return Commands.parallel(
                shooterWheel.passVelocityCmd(),
                intakeRoller.intakeInCmd(),
                intakeAngle.lowOscillate(),
                indexer.autoIndexCmd(() -> isShooterReady()));
    }

    public Command IdleShotPrepCmd() {
        return Commands.parallel(
                shooterWheel.idleVelocityCmd());
    }

    public Command setVelocityShotCmd(Supplier<AngularVelocity> targetVel, AngularVelocity floorThreshold,
            AngularVelocity ceilingThreshold) {
        return Commands.parallel(
                shooterWheel.hubPhaseShotCmd(targetVel, floorThreshold, ceilingThreshold),
                intakeRoller.intakeInCmd(),
                intakeAngle.lowOscillate(),
                indexer.autoIndexCmd(() -> isShooterReady(floorThreshold, ceilingThreshold)));
    }

    /**
     * Checks if the shooter is ready to shoot
     * 
     * @return true if the shooter is ready to shoot, false otherwise
     */
    private boolean isShooterReady() {
        return shooterWheel.atVelocity(Constants.shooterWheelTol);
        // && shooterHood.atPosition(Constants.shooterHoodTol);
    }

    private boolean isShooterReady(AngularVelocity floorThreshold, AngularVelocity ceilingThreshold) {
        return shooterWheel.atVelocity(floorThreshold, ceilingThreshold);
    }

    /**
     * Checks if the robot is aligned with the target
     * 
     * @return true if the robot is aligned with the target, false otherwise.
     */
    private boolean isRobotAligned() {
        return drivetrain.atTargetAngle(Constants.shotAngleTol);
    }
}