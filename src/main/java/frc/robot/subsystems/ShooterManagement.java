package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.FieldLayout;

public class ShooterManagement {

    private final CommandSwerveDrivetrain drivetrain;
    private final Indexer indexer;
    private final ShooterWheel shooterWheel;
    private final ShooterHood shooterHood;

    /**
     * Constructor
     * 
     * @param indexer      indexer subsystem instance
     * @param shooterWheel shooter wheel subsystem instance
     * @param shooterHood  shooter hood subsystem instance
     */
    public ShooterManagement(CommandSwerveDrivetrain drivetrain, Indexer indexer, ShooterWheel shooterWheel,
            ShooterHood shooterHood) {
        this.drivetrain = drivetrain;
        this.indexer = indexer;
        this.shooterWheel = shooterWheel;
        this.shooterHood = shooterHood;
    }

    private AngularVelocity calcHubShotSpeed() {
        Distance hubDist = FieldLayout.Hub.getHubDist(drivetrain.getPose2d().getTranslation());

        return Constants.shooterWheelTable.get(hubDist);
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
                shooterHood.hubShotCmd(),
                drivetrain.lookAtHubCmd(xVel, yVel, Constants.shooterOrientation),
                indexer.autoIndexCmd(() -> isShooterReady() && isRobotAligned()));
    }

    public Command hubNoHoodShotCmd(AngularVelocity velTolerance, Voltage indexerVolt){
        return Commands.parallel(
            shooterWheel.hubShotCmd(),
            Commands.either(indexer.setVoltageCmd(indexerVolt), 
            indexer.setVoltageCmd(Volts.zero()), 
                () -> shooterWheel.getVelocity().gte(calcHubShotSpeed().minus(velTolerance)) && calcHubShotSpeed().plus(velTolerance).gt(shooterWheel.getVelocity()))
        );
    }

    public Command phaseShotIndexerCtrlCmd(Supplier<AngularVelocity> targetVel, AngularVelocity floorThreshold,
            AngularVelocity ceilingThreshold) {
        return Commands.parallel(
                shooterWheel.hubPhaseShotCmd(targetVel, floorThreshold, ceilingThreshold),
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