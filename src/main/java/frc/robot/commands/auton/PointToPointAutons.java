package frc.robot.commands.auton;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    
    public PointToPointAutons(CommandSwerveDrivetrain drivetrain, Indexer indexer, IntakeAngle intakeAngle, 
        IntakeRoller intakeRoller, ShooterManagement shooterManagement, ShooterWheel shooterWheel){
        
        this.drivetrain = drivetrain;
        this.indexer = indexer;
        this.intakeAngle = intakeAngle;
        this.intakeRoller = intakeRoller;
        this.shooterManagement = shooterManagement;
        this.shooterWheel = shooterWheel;
    }

    public static final class AutonWaypoints{
        //Trench
        public static final Waypoint rightTrenchAutonStart = WaypointFactory.of(new Pose2d(FieldLayout.Trench.blueTrenchRight, Rotation2d.kZero), AllianceFlip.ALLIANCE);
        public static final Waypoint rightNeutralIntakePrep = WaypointFactory.of(new Pose2d(7.8, FieldLayout.Trench.blueTrenchRight.getY(), Rotation2d.fromDegrees(90)), AllianceFlip.ALLIANCE);
        public static final Waypoint rightNeutralIntakeFinished = WaypointFactory.of(new Pose2d(7.8, 3.3, Rotation2d.fromDegrees(90)), AllianceFlip.ALLIANCE);
        public static final Waypoint rightBumpCrossPrep = WaypointFactory.of(new Pose2d(6.0, FieldLayout.Bump.blueBumpRight.getY(), Rotation2d.fromDegrees(180)), AllianceFlip.ALLIANCE);
        public static final Waypoint rightBumpCrossFinished = WaypointFactory.of(new Pose2d(3.0, FieldLayout.Bump.blueBumpRight.getY(), Rotation2d.fromDegrees(210)), AllianceFlip.ALLIANCE);
    }

    public final Command eventMarkerGoTo(Waypoint target, double percent, Command event){
        return drivetrain.runOnce(() -> drivetrain.initialPoseHelper(drivetrain.getPose2d()))
        .andThen(
            new ParallelDeadlineGroup(
                drivetrain.goToPointCmd(target.get()),
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
            drivetrain.getResetPoseCmd(AutonWaypoints.rightTrenchAutonStart.withRotation(FieldLayout.isRedAlliance() ? Rotation2d.k180deg : Rotation2d.kZero).get()),
            drivetrain.goToPointCmd(AutonWaypoints.rightNeutralIntakePrep.get(), Meters.of(0.2)),
            drivetrain.yAxisAlignDistanceCmd(() -> MetersPerSecond.of(3), AutonWaypoints.rightNeutralIntakeFinished.get(), Meters.of(0.2)),
            drivetrain.goToPointCmd(AutonWaypoints.rightBumpCrossPrep.get(), Meters.of(1)),
            drivetrain.goToPointCmd(AutonWaypoints.rightBumpCrossFinished.get(), Meters.of(0.2)),
            hubOrbitRangeCmd()
        );
    }

    public Command hubOrbitRangeCmd() {
        return drivetrain.hubOrbitRestrictedRadiusCommand(() -> MetersPerSecond.zero(), () -> MetersPerSecond.zero(),
                Rotation2d.fromDegrees(180),
                Inches.of(147), Meters.of(1.75));
    }



}
