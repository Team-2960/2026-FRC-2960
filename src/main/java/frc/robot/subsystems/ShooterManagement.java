package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class ShooterManagement {

    private final CommandSwerveDrivetrain drivetrain;
    private final Indexer indexer;
    private final IntakeAngle intakeAngle;
    private final IntakeRoller intakeRoller;
    private final ShooterWheel shooterWheel;
    private final ShooterHood shooterHood;

    /**
     * Constructor
     * 
     * @param indexer      indexer subsystem instance
     * @param shooterWheel shooter wheel subsystem instance
     * @param shooterHood  shooter hood subsystem instance
     */
    public ShooterManagement(CommandSwerveDrivetrain drivetrain, IntakeAngle intakeAngle, IntakeRoller intakeRoller,
            Indexer indexer, ShooterWheel shooterWheel,
            ShooterHood shooterHood) {
        this.drivetrain = drivetrain;
        this.indexer = indexer;
        this.intakeAngle = intakeAngle;
        this.intakeRoller = intakeRoller;
        this.shooterWheel = shooterWheel;
        this.shooterHood = shooterHood;
    }

    /**
     * Creates a command to set the shooter wheel and hood to be ready to shoot at
     * the hub
     * 
     * @return command to set the shooter and hood to be ready to shoot at the hub
     */
    public Command hubShotPrepCmd() {
        return Commands.parallel(
                shooterWheel.hubShotCmd(),
                shooterHood.hubShotCmd());
    }

    /**
     * Creates a command to set the robot orientation, shooter wheel and shooter
     * hood to be ready to shoot at the hub
     * 
     * @param xVel target x velocity
     * @param yVel target y velocity
     * 
     * @return command to set the robot orientation, shooter wheel and shooter hood
     *         to be ready to shoot at the hub
     */
    public Command hubAlignCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel) {
        return Commands.parallel(
                shooterWheel.hubShotCmd(),
                shooterHood.hubShotCmd(),
                drivetrain.lookAtHubCmd(xVel, yVel, Constants.shooterOrientation));
    }

    /**
     * Creates a command to align the shooter to the hub, set the shooter wheel and
     * hood to shoot at the hub, and feed game pieces when ready.
     * 
     * @param xVel target x velocity
     * @param yVel target y velocity
     * @return command to align the shooter to the hub, set the shooter wheel and
     *         hood to shoot at the hub, and feed game pieces when ready.
     */
    public Command hubShotCmd() {
        return Commands.parallel(
                shooterWheel.hubShotCmd(),
                // shooterHood.hubShotCmd(),
                indexer.autoIndexCmd(this::isShooterReady));
    }

    /**
     * Creates a command to align the shooter to the hub, set the shooter wheel and
     * hood to shoot at the hub, and feed game pieces when ready.
     * 
     * @param xVel target x velocity
     * @param yVel target y velocity
     * @return command to align the shooter to the hub, set the shooter wheel and
     *         hood to shoot at the hub, and feed game pieces when ready.
     */
    public Command hubAlignedShotCmd(
            Supplier<LinearVelocity> xVel,
            Supplier<LinearVelocity> yVel) {
        return Commands.parallel(
                shooterWheel.hubShotCmd(),
                // shooterHood.hubShotCmd(),
                intakeAngle.lowOscillate(),
                intakeRoller.intakeCmd(),
                drivetrain.lookAtHubCmd(xVel, yVel, Constants.shooterOrientation),
                indexer.autoIndexCmd(() -> isShooterReady() && isRobotAligned()));
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
    
    /**
     * Checks if the robot is aligned with the target
     * 
     * @return true if the robot is aligned with the target, false otherwise.
     */
    private boolean isRobotAligned() {
        return drivetrain.atTargetAngle(Constants.shotAngleTol);
    }
}