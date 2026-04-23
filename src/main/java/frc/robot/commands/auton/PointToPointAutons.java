package frc.robot.commands.auton;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldLayout;
import frc.robot.Util.WaypointFactory;
import frc.robot.Util.WaypointFactory.AllianceFlip;
import frc.robot.Util.WaypointFactory.Waypoint;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeAngle;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.ShooterManagement;
import frc.robot.subsystems.ShooterWheel;

public final class PointToPointAutons {

    private final CommandSwerveDrivetrain drivetrain;
    private final Indexer indexer;
    private final IntakeAngle intakeAngle;
    private final IntakeRoller intakeRoller;
    private final ShooterManagement shooterManagement;
    private final ShooterWheel shooterWheel;

    private final SendableChooser<Command> autonChooser = new SendableChooser<>();
    
    public PointToPointAutons(CommandSwerveDrivetrain drivetrain, Indexer indexer, IntakeAngle intakeAngle, 
        IntakeRoller intakeRoller, ShooterManagement shooterManagement, ShooterWheel shooterWheel){
        
        this.drivetrain = drivetrain;
        this.indexer = indexer;
        this.intakeAngle = intakeAngle;
        this.intakeRoller = intakeRoller;
        this.shooterManagement = shooterManagement;
        this.shooterWheel = shooterWheel;
        
        autonChooser.setDefaultOption("null", Commands.none());
        autonChooser.addOption("Test Auton", getTestAuton());
        autonChooser.addOption("Right Trench 2 Cycle", getRightTrench2Cycle());
        autonChooser.addOption("Left Trench 2 Cycle", getLeftTrench2Cycle());
    }

    public static final class AutonWaypoints{
        //Trench
        public static final Waypoint rightTrenchAutonStart = WaypointFactory.of(new Pose2d(FieldLayout.Trench.blueTrenchRight, Rotation2d.fromDegrees(90)), AllianceFlip.ALLIANCE);
        public static final Waypoint rightNeutralIntakePrep = WaypointFactory.of(new Pose2d(7.8, FieldLayout.Trench.blueTrenchRight.getY(), Rotation2d.fromDegrees(90)), AllianceFlip.ALLIANCE);
        public static final Waypoint rightNeutralIntakeFinished = WaypointFactory.of(new Pose2d(7.8, 3.3, Rotation2d.fromDegrees(90)), AllianceFlip.ALLIANCE);
        public static final Waypoint rightBumpCrossPrep = WaypointFactory.of(new Pose2d(6.0, FieldLayout.Bump.blueBumpRight.getY(), Rotation2d.fromDegrees(180)), AllianceFlip.ALLIANCE);
        public static final Waypoint rightBumpCrossFinished = rightBumpCrossPrep.translated(-3.5, 0).withRotation(Rotation2d.fromDegrees(210));
        public static final Waypoint rightTrenchNeutralPrep = rightTrenchAutonStart.translated(-2, 0).withRotation(Rotation2d.fromDegrees(90));
        public static final Waypoint rightTrenchAlliancePrep = rightTrenchAutonStart.translated(2, 0).withRotation(Rotation2d.fromRadians(-90));

        //Snake Intake Path Points
        public static final Waypoint snakePoint1 = WaypointFactory.of(new Pose2d(6.4, 3.6, Rotation2d.fromDegrees(-110)), AllianceFlip.ALLIANCE);
        public static final Waypoint snakePoint2 = rightBumpCrossPrep;
        

        public static final Waypoint rightBumpAutonStart = WaypointFactory.of(
            FieldLayout.Bump.blueBumpRight.minus(
                new Translation2d(FieldLayout.Bump.bumpWidth.div(2).plus(Constants.robotWithBumpersLength), 
                    Inches.zero())), 
            Rotation2d.kZero, 
            AllianceFlip.ALLIANCE);

        public static final Waypoint centerHubAutonStart = WaypointFactory.of(
            FieldLayout.Hub.blueHubCenterFront.minus(
                new Translation2d(
                    Constants.robotWithBumpersLength.div(2), Inches.of(0))), 
            Rotation2d.kZero, 
            AllianceFlip.ALLIANCE);

        public static final Pose2d[] rightAutonPoints = {
            rightTrenchAutonStart.get(),
            rightNeutralIntakePrep.get(),
            rightNeutralIntakeFinished.get(),
            rightBumpCrossPrep.get(),
            rightBumpCrossFinished.get(),
            rightTrenchNeutralPrep.get()
        };
    }

    public static class WaypointSet {
        private final List<Waypoint> waypoints = new ArrayList<>();

        public Waypoint add(Waypoint w) {
            waypoints.add(w);
            return w; // returns the waypoint so it can be used inline
        }

        public List<Pose2d> getResolved() {
            return waypoints.stream().map(Waypoint::get).toList();
        }

        public Pose2d[] getResolvedArray() {
            return waypoints.stream().map(Waypoint::get).toArray(Pose2d[]::new);
        }
    }

    public final SendableChooser<Command> getAutonChooser(){
        return autonChooser;
    }

    public final Command eventMarkerGoTo(Waypoint target, double percent, Distance tolerance,Command event){
        return drivetrain.runOnce(() -> drivetrain.initialPoseHelper(drivetrain.getPose2d()))
        .andThen(
            new ParallelDeadlineGroup(
                drivetrain.goToPointCmd(target, tolerance),
                new SequentialCommandGroup(
                    Commands.waitUntil(() -> {
                        Pose2d targetPose = target.get();
                        double totalDist = PhotonUtils.getDistanceToPose(drivetrain.getInitialPose(), targetPose);
                        double remainingDist = PhotonUtils.getDistanceToPose(drivetrain.getPose2d(), targetPose);
                        double coveredDist = totalDist - remainingDist;
                        return coveredDist >= totalDist * percent;
                    }),
                    event
                )
            )
        );
    }
 
    public Command getTestAuton(){
        return new SequentialCommandGroup(
            drivetrain.getResetPoseCmd(AutonWaypoints.rightTrenchAutonStart.withRotation(FieldLayout.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero)),
            drivetrain.goToPointCmd(AutonWaypoints.rightNeutralIntakePrep, Meters.of(0.2)),
            drivetrain.yAxisAlignDistanceCmd(() -> MetersPerSecond.of(3), AutonWaypoints.rightNeutralIntakeFinished, Meters.of(0.2)),
            drivetrain.goToPointCmd(AutonWaypoints.rightBumpCrossPrep, Meters.of(1)),
            drivetrain.goToPointCmd(AutonWaypoints.rightBumpCrossFinished, Meters.of(0.2)),
            hubOrbitRangeCmd()
        );
    }

    private Command getTrench2CycleImpl(boolean mirror) {
        WaypointSet points = new WaypointSet();

        Waypoint trenchStart       = points.add(w(AutonWaypoints.rightTrenchAutonStart, mirror));
        Waypoint neutralIntakePrep = points.add(w(AutonWaypoints.rightNeutralIntakePrep, mirror));
        Waypoint neutralIntakeFin  = points.add(w(AutonWaypoints.rightNeutralIntakeFinished, mirror));
        Waypoint bumpCrossPrep     = points.add(w(AutonWaypoints.rightBumpCrossPrep, mirror));
        Waypoint bumpCrossFin      = points.add(w(AutonWaypoints.rightBumpCrossFinished, mirror));
        Waypoint trenchPrep        = points.add(w(AutonWaypoints.rightTrenchNeutralPrep, mirror));

        double yVel = mirror ? -3 : 3;

        return new SequentialCommandGroup(
            drivetrain.getResetPoseCmd(trenchStart.withRotation(
                FieldLayout.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero)),

            drivetrain.drawAutonPathCmd(points),

            // First pass
            eventMarkerGoTo(neutralIntakePrep, 0.2, Meters.of(0.2),
                new ParallelCommandGroup(intakeAngle.extendCmd(), intakeRoller.intakeInCmd())),
            new ParallelDeadlineGroup(
                drivetrain.yAxisAlignDistanceCmd(() -> MetersPerSecond.of(yVel), neutralIntakeFin, Meters.of(0.2)),
                intakeAngle.extendCmd(), intakeRoller.intakeInCmd()),
            new ParallelDeadlineGroup(drivetrain.goToPointCruiseCmd(bumpCrossPrep, MetersPerSecond.of(5), Meters.of(0.1))),
            new ParallelDeadlineGroup(drivetrain.goToPointCruiseCmd(bumpCrossFin, MetersPerSecond.of(4), Meters.of(0.2))),

            // Shoot 1
            getShootRoutine(),

            // Second pass
            new ParallelDeadlineGroup(
                drivetrain.goToPointCruiseCmd(trenchPrep.translated(0, 0.2), MetersPerSecond.of(4), Meters.of(0.1)),
                intakeAngle.retractCmd()),
            eventMarkerGoTo(neutralIntakePrep.translated(0, 0.2), 0.2, Meters.of(0.2),
                new ParallelCommandGroup(intakeAngle.extendCmd(), intakeRoller.intakeInCmd())),
            new ParallelDeadlineGroup(
                drivetrain.yAxisAlignDistanceCmd(() -> MetersPerSecond.of(yVel), neutralIntakeFin, Meters.of(0.2)),
                intakeAngle.extendCmd(), intakeRoller.intakeInCmd()),
            new ParallelDeadlineGroup(drivetrain.goToPointCruiseCmd(bumpCrossPrep, MetersPerSecond.of(5), Meters.of(0.1))),
            new ParallelDeadlineGroup(drivetrain.goToPointCruiseCmd(bumpCrossFin, MetersPerSecond.of(4), Meters.of(0.2))),

            // Shoot 2
            getShootRoutine()
        );
    }

    // Tiny alias to keep the impl method readable
    private Waypoint w(Waypoint waypoint, boolean mirror) {
        return mirror ? mirrored(waypoint) : waypoint;
    }

    /**
     * Applies an additional flip policy on top of a waypoint's existing policy.
     * Used to mirror entire autons without redefining every waypoint.
     */
    private Waypoint mirrored(Waypoint w) {
        return switch (w.getFlipPolicy()) {
            case NONE     -> w.withFlipPolicy(AllianceFlip.X_AXIS);
            case ALLIANCE -> w.withFlipPolicy(AllianceFlip.BOTH);
            case X_AXIS   -> w.withFlipPolicy(AllianceFlip.NONE);
            case BOTH     -> w.withFlipPolicy(AllianceFlip.ALLIANCE);
        };
    }

    public Command getRightTrench2Cycle() { 
        return getTrench2CycleImpl(false); 
    }
    
    public Command getLeftTrench2Cycle() {
        return getTrench2CycleImpl(true);
    }

    public Command hubOrbitRangeCmd() {
        return drivetrain.hubOrbitRestrictedRadiusCommand(() -> MetersPerSecond.zero(), () -> MetersPerSecond.zero(),
                Rotation2d.fromDegrees(180),
                Inches.of(147), Meters.of(1.75));
    }

    public Command getShootRoutine(){
        return hubOrbitRangeCmd().alongWith(shooterManagement.hubIndexAutoShotCmd()).withTimeout(5);
    }

    public Command getTrenchNeutralZone(boolean mirror){
        WaypointSet points = new WaypointSet();

        Waypoint neutralIntakePrep = points.add(w(AutonWaypoints.rightNeutralIntakePrep, mirror));
        Waypoint neutralIntakeFin  = points.add(w(AutonWaypoints.rightNeutralIntakeFinished, mirror));

        double yVel = mirror ? -3 : 3;

        return new SequentialCommandGroup(
            // First pass
            eventMarkerGoTo(neutralIntakePrep, 0.2, Meters.of(0.2),
                new ParallelCommandGroup(intakeAngle.extendCmd(), intakeRoller.intakeInCmd())),
            new ParallelDeadlineGroup(
                drivetrain.yAxisAlignDistanceCmd(() -> MetersPerSecond.of(yVel), neutralIntakeFin, Meters.of(0.2)),
                intakeAngle.extendCmd(), intakeRoller.intakeInCmd())
        );
    }

    public Command getBumpNeutralZone(boolean mirror){
        
        Waypoint neutralIntakePrep = w(AutonWaypoints.rightNeutralIntakePrep, mirror);
        Waypoint neutralIntakeFin  = w(AutonWaypoints.rightNeutralIntakeFinished, mirror);
        Waypoint bumpCrossPrep = w(AutonWaypoints.rightBumpCrossPrep.withRotation(Rotation2d.kZero), mirror);

        double yVel = mirror ? -3 : 3;

        return new SequentialCommandGroup(
            drivetrain.goToPointCruiseCmd(bumpCrossPrep, MetersPerSecond.of(4), Meters.of(0.3)),
            // First pass
            eventMarkerGoTo(neutralIntakePrep, 0.2, Meters.of(0.2),
                new ParallelCommandGroup(intakeAngle.extendCmd(), intakeRoller.intakeInCmd())),
            new ParallelDeadlineGroup(
                drivetrain.yAxisAlignDistanceCmd(() -> MetersPerSecond.of(yVel), neutralIntakeFin, Meters.of(0.2)),
                intakeAngle.extendCmd(), intakeRoller.intakeInCmd())
        );
    }

    public Command getReturnBumperRoutine(boolean mirror){
        Waypoint bumpCrossPrep     = w(AutonWaypoints.rightBumpCrossPrep, mirror);
        Waypoint bumpCrossFin      =  w(AutonWaypoints.rightBumpCrossFinished, mirror);
        
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(drivetrain.goToPointCruiseCmd(bumpCrossPrep, MetersPerSecond.of(4), Meters.of(0.1))),
            new ParallelDeadlineGroup(drivetrain.goToPointCruiseCmd(bumpCrossFin, MetersPerSecond.of(2), Meters.of(0.2)))
        );
    }

    public Command getReturnTrenchRoutine(boolean mirror){
        Waypoint trenchCrossPrep = w(AutonWaypoints.rightTrenchAlliancePrep.withRotation(Rotation2d.fromDegrees(-90)), mirror);
        Waypoint trenchCrossFinished = w(AutonWaypoints.rightTrenchNeutralPrep.withRotation(Rotation2d.fromDegrees(-90)), mirror);

        return new SequentialCommandGroup(
            drivetrain.goToPointCmd(trenchCrossPrep, Meters.of(0.1)),
            drivetrain.goToPointCruiseCmd(trenchCrossFinished, MetersPerSecond.of(4), Meters.of(0.2))
        );
    }
    
    public Command getSnakeIntakePath(boolean mirror){
        Waypoint snakePointOne = w(AutonWaypoints.snakePoint1, mirror);
        Waypoint snakePointTwo = w(AutonWaypoints.snakePoint2, mirror);

        AngularVelocity rotationalVel = RotationsPerSecond.of(0.4);
        rotationalVel = mirror ? rotationalVel.times(-1) : rotationalVel;

        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                drivetrain.goToPointRotationCruiseCmd(snakePointOne, MetersPerSecond.of(2), rotationalVel, Meters.of(0.2), Rotation2d.fromDegrees(2)),
                intakeRoller.intakeInCmd()
            ),

            new ParallelDeadlineGroup(
                drivetrain.goToPointRotationCruiseCmd(snakePointTwo, MetersPerSecond.of(2), rotationalVel, Meters.of(0.2), Rotation2d.fromDegrees(2)),                
                intakeRoller.intakeInCmd()
            )
        );
    }

    public class AutonBuilder{

        private final SendableChooser<Boolean> isMirroredChooser = new SendableChooser<>();
        private final SendableChooser<StartType> startTypeChooser = new SendableChooser<>();
        private final SendableChooser<NeutralZoneType> neutralZoneChooser = new SendableChooser<>();
        private final SendableChooser<IntakePathType> intakePathChooser = new SendableChooser<>();
        private final SendableChooser<ReturnType> returnTypeChooser = new SendableChooser<>();

        public AutonBuilder(){
            startTypeChooser.addOption("Trench", StartType.TRENCH);
            startTypeChooser.addOption("Bump", StartType.BUMP);
            startTypeChooser.addOption("Hub", StartType.HUB);
            startTypeChooser.setDefaultOption("None", StartType.NONE);

            neutralZoneChooser.addOption("Trench", NeutralZoneType.TRENCH);
            neutralZoneChooser.addOption("Bump", NeutralZoneType.BUMP);
            neutralZoneChooser.setDefaultOption("None", NeutralZoneType.NONE);

            intakePathChooser.addOption("Snake", IntakePathType.SNAKE);
            intakePathChooser.setDefaultOption("None", IntakePathType.NONE);

            returnTypeChooser.addOption("Trench", ReturnType.TRENCH);
            returnTypeChooser.addOption("Bump", ReturnType.BUMP);
            returnTypeChooser.setDefaultOption("None", ReturnType.NONE);

            isMirroredChooser.addOption("True", true);
            isMirroredChooser.setDefaultOption("False", false);

            SmartDashboard.putData("Start Type P2PC", startTypeChooser);
            SmartDashboard.putData("Neutral Zone P2PC", neutralZoneChooser);
            SmartDashboard.putData("Intake Path P2PC", intakePathChooser);
            SmartDashboard.putData("Return Type P2PC", returnTypeChooser);
            SmartDashboard.putData("Is Mirrored P2PC", isMirroredChooser);
        }

        private enum StartType{
            NONE,
            TRENCH,
            BUMP,
            HUB
        }

        private enum NeutralZoneType{
            NONE,
            TRENCH,
            BUMP
        }

        private enum IntakePathType{
            NONE,
            SNAKE        
        }

        private enum ReturnType{
            NONE,
            TRENCH,
            BUMP
        }

        public Command getStartCommand(){
            
            switch (startTypeChooser.getSelected()) {
                case TRENCH:
                    return drivetrain.getResetPoseCmd(w(AutonWaypoints.rightTrenchAutonStart, isMirroredChooser.getSelected()));
                
                case BUMP:
                    return drivetrain.getResetPoseCmd(w(AutonWaypoints.rightBumpAutonStart, isMirroredChooser.getSelected()));

                case HUB:
                    return drivetrain.getResetPoseCmd(w(AutonWaypoints.centerHubAutonStart, isMirroredChooser.getSelected()));
                
                default:
                    return drivetrain.getResetPoseCmd(Pose2d.kZero);
            }
        }

        public Command getNeutralZoneCommand(){
            switch (neutralZoneChooser.getSelected()) {
                case TRENCH:
                    return getTrenchNeutralZone(isMirroredChooser.getSelected());

                case BUMP:
                    return getBumpNeutralZone(isMirroredChooser.getSelected());

                default:
                    return Commands.none();
            }
        }

        public Command getIntakePathCommand(){
            switch (intakePathChooser.getSelected()) {
                case SNAKE:
                    return getSnakeIntakePath(isMirroredChooser.getSelected());
            
                default:
                    return Commands.none();
            }
        }

        public Command getReturnCommand(){
            switch (returnTypeChooser.getSelected()) {
                case TRENCH: 
                    return getReturnTrenchRoutine(isMirroredChooser.getSelected());
                case BUMP:
                    return getReturnBumperRoutine(isMirroredChooser.getSelected());

                default:
                    return Commands.none();
            }
        }


        public Command getAuton(){
            return new SequentialCommandGroup(
                getStartCommand(),
                getNeutralZoneCommand(),
                getIntakePathCommand(),
                getReturnCommand(),
                getShootRoutine()
            );
        }
        public SendableChooser<StartType> getStartTypeChooser(){
            return startTypeChooser;
        }

        public SendableChooser<NeutralZoneType> getNeutralZoneChooser(){
            return neutralZoneChooser;
        }
        
        public SendableChooser<IntakePathType> getIntakePathChooser(){
            return intakePathChooser;
        }

        public SendableChooser<ReturnType> getReturnTypeChooser(){
            return returnTypeChooser;
        }
    }
}
