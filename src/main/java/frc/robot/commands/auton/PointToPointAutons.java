package frc.robot.commands.auton;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
        public static final Waypoint rightTrenchAutonStart = WaypointFactory.of(new Pose2d(FieldLayout.Trench.blueTrenchRight, Rotation2d.kZero), AllianceFlip.ALLIANCE);
        public static final Waypoint rightNeutralIntakePrep = WaypointFactory.of(new Pose2d(7.8, FieldLayout.Trench.blueTrenchRight.getY(), Rotation2d.fromDegrees(90)), AllianceFlip.ALLIANCE);
        public static final Waypoint rightNeutralIntakeFinished = WaypointFactory.of(new Pose2d(7.8, 3.3, Rotation2d.fromDegrees(90)), AllianceFlip.ALLIANCE);
        public static final Waypoint rightBumpCrossPrep = WaypointFactory.of(new Pose2d(6.0, FieldLayout.Bump.blueBumpRight.getY(), Rotation2d.fromDegrees(180)), AllianceFlip.ALLIANCE);
        public static final Waypoint rightBumpCrossFinished = rightBumpCrossPrep.translated(-3.5, 0).withRotation(Rotation2d.fromDegrees(210));
        public static final Waypoint rightTrenchPrep = rightTrenchAutonStart.translated(-2, 0).withRotation(Rotation2d.fromDegrees(90));

        public static final Pose2d[] rightAutonPoints = {
            rightTrenchAutonStart.get(),
            rightNeutralIntakePrep.get(),
            rightNeutralIntakeFinished.get(),
            rightBumpCrossPrep.get(),
            rightBumpCrossFinished.get(),
            rightTrenchPrep.get()
        };
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

    public Command getRightTrench2Cycle(){
        return new SequentialCommandGroup(
            drivetrain.getResetPoseCmd(AutonWaypoints.rightTrenchAutonStart.withRotation(FieldLayout.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero)),

            //First Pass
            eventMarkerGoTo(AutonWaypoints.rightNeutralIntakePrep, 0.2, Meters.of(0.2), 
                new ParallelCommandGroup(
                    intakeAngle.extendCmd(),
                    intakeRoller.intakeInCmd()
                )
            ),
            new ParallelDeadlineGroup(
                drivetrain.yAxisAlignDistanceCmd(() -> MetersPerSecond.of(3), AutonWaypoints.rightNeutralIntakeFinished, Meters.of(0.2)),
                intakeAngle.extendCmd(),
                intakeRoller.intakeInCmd()
            ),
            new ParallelDeadlineGroup(
                drivetrain.goToPointCruiseCmd(AutonWaypoints.rightBumpCrossPrep, MetersPerSecond.of(5), Meters.of(0.1))
            ),
            new ParallelDeadlineGroup(
                drivetrain.goToPointCruiseCmd(AutonWaypoints.rightBumpCrossFinished, MetersPerSecond.of(4), Meters.of(0.2))
            ),
            
            //Shoot 1
            hubOrbitRangeCmd().alongWith(
                shooterManagement.hubAutoShotCmd()
            ).withTimeout(5),
            
            //Second Pass
            new ParallelDeadlineGroup(
                drivetrain.goToPointCruiseCmd(AutonWaypoints.rightTrenchPrep.translated(0, 0.2), MetersPerSecond.of(4), Meters.of(0.1)),
                intakeAngle.retractCmd()
            ),

            eventMarkerGoTo(AutonWaypoints.rightNeutralIntakePrep.translated(0, 0.2), 0.2, Meters.of(0.2), 
                new ParallelCommandGroup(
                    intakeAngle.extendCmd(),
                    intakeRoller.intakeInCmd()
                )
            ),
            new ParallelDeadlineGroup(
                drivetrain.yAxisAlignDistanceCmd(() -> MetersPerSecond.of(3), AutonWaypoints.rightNeutralIntakeFinished, Meters.of(0.2)),
                intakeAngle.extendCmd(),
                intakeRoller.intakeInCmd()
            ),
            new ParallelDeadlineGroup(
                drivetrain.goToPointCruiseCmd(AutonWaypoints.rightBumpCrossPrep, MetersPerSecond.of(5), Meters.of(0.1))
            ),
            new ParallelDeadlineGroup(
                drivetrain.goToPointCruiseCmd(AutonWaypoints.rightBumpCrossFinished, MetersPerSecond.of(4), Meters.of(0.2))
            ),
            
            //Shoot 2
            hubOrbitRangeCmd().alongWith(
                shooterManagement.hubAutoShotCmd()
            ).withTimeout(5)
        );
    }

    public Command getLeftTrench2Cycle(){
        return new SequentialCommandGroup(
            drivetrain.getResetPoseCmd(AutonWaypoints.rightTrenchAutonStart.withFlipPolicy(AllianceFlip.BOTH).withRotation(FieldLayout.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero)),

            //First Pass
            eventMarkerGoTo(AutonWaypoints.rightNeutralIntakePrep.withFlipPolicy(AllianceFlip.BOTH), 0.2, Meters.of(0.2), 
                new ParallelCommandGroup(
                    intakeAngle.extendCmd(),
                    intakeRoller.intakeInCmd()
                )
            ),
            new ParallelDeadlineGroup(
                drivetrain.yAxisAlignDistanceCmd(() -> MetersPerSecond.of(-3), AutonWaypoints.rightNeutralIntakeFinished.withFlipPolicy(AllianceFlip.BOTH), Meters.of(0.2)),
                intakeAngle.extendCmd(),
                intakeRoller.intakeInCmd()
            ),
            new ParallelDeadlineGroup(
                drivetrain.goToPointCruiseCmd(AutonWaypoints.rightBumpCrossPrep.withFlipPolicy(AllianceFlip.BOTH), MetersPerSecond.of(5), Meters.of(0.1))
            ),
            new ParallelDeadlineGroup(
                drivetrain.goToPointCruiseCmd(AutonWaypoints.rightBumpCrossFinished.withFlipPolicy(AllianceFlip.BOTH), MetersPerSecond.of(4), Meters.of(0.2))
            ),
            
            //Shoot 1
            hubOrbitRangeCmd().alongWith(
                shooterManagement.hubAutoShotCmd()
            ).withTimeout(5),
            
            //Second Pass
            new ParallelDeadlineGroup(
                drivetrain.goToPointCruiseCmd(AutonWaypoints.rightTrenchPrep.withFlipPolicy(AllianceFlip.BOTH).translated(0, 0.2), MetersPerSecond.of(4), Meters.of(0.1)),
                intakeAngle.retractCmd()
            ),

            eventMarkerGoTo(AutonWaypoints.rightNeutralIntakePrep.withFlipPolicy(AllianceFlip.BOTH).translated(0, 0.2), 0.2, Meters.of(0.2), 
                new ParallelCommandGroup(
                    intakeAngle.extendCmd(),
                    intakeRoller.intakeInCmd()
                )
            ),
            new ParallelDeadlineGroup(
                drivetrain.yAxisAlignDistanceCmd(() -> MetersPerSecond.of(-3), AutonWaypoints.rightNeutralIntakeFinished.withFlipPolicy(AllianceFlip.BOTH), Meters.of(0.2)),
                intakeAngle.extendCmd(),
                intakeRoller.intakeInCmd()
            ),
            new ParallelDeadlineGroup(
                drivetrain.goToPointCruiseCmd(AutonWaypoints.rightBumpCrossPrep.withFlipPolicy(AllianceFlip.BOTH), MetersPerSecond.of(5), Meters.of(0.1))
            ),
            new ParallelDeadlineGroup(
                drivetrain.goToPointCruiseCmd(AutonWaypoints.rightBumpCrossFinished.withFlipPolicy(AllianceFlip.BOTH), MetersPerSecond.of(4), Meters.of(0.2))
            ),
            
            //Shoot 2
            hubOrbitRangeCmd().alongWith(
                shooterManagement.hubAutoShotCmd()
            ).withTimeout(5)
        );
    }

    public Command hubOrbitRangeCmd() {
        return drivetrain.hubOrbitRestrictedRadiusCommand(() -> MetersPerSecond.zero(), () -> MetersPerSecond.zero(),
                Rotation2d.fromDegrees(180),
                Inches.of(147), Meters.of(1.75));
    }
}
