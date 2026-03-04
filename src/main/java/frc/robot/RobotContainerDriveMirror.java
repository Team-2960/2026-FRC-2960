// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CommandSelector;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagPipeline;
import frc.robot.subsystems.CameraSim;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainerDriveMirror {

    /* Setting up bindings for necessary control of the swerve drive platform */

    private final Telemetry logger = new Telemetry(Constants.maxLinVel.in(MetersPerSecond));

    private final CommandXboxController driverCtrl = new CommandXboxController(0);


    // Physical Subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Pathplanner
    SendableChooser<Command> autoChooser;

    // Mutable Units
    private MutLinearVelocity xVel = MetersPerSecond.mutable(0);
    private MutLinearVelocity yVel = MetersPerSecond.mutable(0);
    private MutAngularVelocity rVel = RotationsPerSecond.mutable(0);

    // Triggers
    private Trigger testMode = new Trigger(DriverStation::isTest);

    // // Cameras
    private final AprilTagPipeline leftCamera = new AprilTagPipeline(
            drivetrain,
            Constants.leftCameraSettings,
            "LeftCamera",
            "LeftCamera");
    private final AprilTagPipeline rightCamera = new AprilTagPipeline(
            drivetrain,
            Constants.rightCameraSettings,
            "RightCamera",
            "RightCamera");

    @SuppressWarnings("unused")
    private final CameraSim cameraSim = new CameraSim(drivetrain, leftCamera, rightCamera);


    // Standard Suppliers
    private Supplier<LinearVelocity> fullXVelCtrl = () -> xVel.mut_replace(Constants.maxLinVel)
            .mut_times(MathUtil.applyDeadband(-driverCtrl.getHID().getLeftY(), 0.02));
    private Supplier<LinearVelocity> fullYVelCtrl = () -> yVel.mut_replace(Constants.maxLinVel)
            .mut_times(MathUtil.applyDeadband(-driverCtrl.getHID().getLeftX(), 0.02));
    private Supplier<AngularVelocity> fullRVelCtrl = () -> rVel.mut_replace(Constants.maxAngVel)
            .mut_times(MathUtil.applyDeadband(-driverCtrl.getHID().getRightX(), 0.02));

    private Supplier<LinearVelocity> slowXVelCtrl = () -> xVel.mut_replace(Constants.slowdownLinVel)
            .mut_times(-driverCtrl.getHID().getLeftY());
    private Supplier<LinearVelocity> slowYVelCtrl = () -> yVel.mut_replace(Constants.slowdownLinVel)
            .mut_times(-driverCtrl.getHID().getLeftX());
    private Supplier<AngularVelocity> slowRVelCtrl = () -> rVel.mut_replace(Constants.slowdownAngVel)
            .mut_times(-driverCtrl.getHID().getRightX());

    // Test Command Lists
    private final CommandSelector drivetrainTestCmds = new CommandSelector()
        .addEntry("SysID Translation Quasistatic Forward", drivetrain.sysIdTranslationQuasistatic(Direction.kForward))
        .addEntry("SysID Translation Quasistatic Reverse", drivetrain.sysIdTranslationQuasistatic(Direction.kReverse))
        .addEntry("SysID Translation Dynamic Forward", drivetrain.sysIdTranslationDynamic(Direction.kForward))
        .addEntry("SysID Translation Dynamic Forward", drivetrain.sysIdTranslationDynamic(Direction.kReverse))
        .addEntry("SysID Steer Quasistatic Forward", drivetrain.sysIdSteerQuasistatic(Direction.kForward))
        .addEntry("SysID Steer Quasistatic Reverse", drivetrain.sysIdSteerQuasistatic(Direction.kReverse))
        .addEntry("SysID Steer Dynamic Forward", drivetrain.sysIdSteerDynamic(Direction.kForward))
        .addEntry("SysID Steer Dynamic Forward", drivetrain.sysIdSteerDynamic(Direction.kReverse))
        .addEntry("SysID Rotation Quasistatic Forward", drivetrain.sysIdRotationQuasistatic(Direction.kForward))
        .addEntry("SysID Rotation Quasistatic Reverse", drivetrain.sysIdRotationQuasistatic(Direction.kReverse))
        .addEntry("SysID Rotation Dynamic Forward", drivetrain.sysIdRotationDynamic(Direction.kForward))
        .addEntry("SysID Rotation Dynamic Forward", drivetrain.sysIdRotationDynamic(Direction.kReverse))
        .sendToDashboard("Drivetrain Test Commands");

    /**
     * Constructor
     */
    public RobotContainerDriveMirror() {
        // Init PathPlanner
        initPathPlanner();

        // Configure Robot Bindings
        configureBindings();

        // Initialize drivetrain telemetry
        drivetrain.registerTelemetry(logger::telemeterize);

    }

    /**
     * Initializes PathPlanner
     */
    private void initPathPlanner() {
        // Initialize Auton chooser
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auton Chooser", autoChooser);
    }

    /**
     * Configures robot bindings
     */
    private void configureBindings() {
        drivetrainBindings();
    }

    /**
     * Configures drivetrain bindings
     */
    private void drivetrainBindings() {
        // Set default drivetrain command
        drivetrain.setDefaultCommand(
                drivetrain.getDriveCmd(
                        fullXVelCtrl,
                        fullYVelCtrl,
                        fullRVelCtrl));

        // Slow Drive Command
        driverCtrl.rightBumper().whileTrue(
                drivetrain.getDriveCmd(
                        slowXVelCtrl,
                        slowYVelCtrl,
                        slowRVelCtrl));

        // Track Goal
        driverCtrl.leftBumper().whileTrue(
                drivetrain.lookAtPointCmd(
                        fullXVelCtrl,
                        fullYVelCtrl,
                        FieldLayout.getHubCenter(),
                        Rotation2d.fromDegrees(180)));

        driverCtrl.a().whileTrue(
                drivetrain.hubOrbitCommand(fullYVelCtrl, Rotation2d.fromDegrees(180), Inches.of(92)));

        driverCtrl.b().whileTrue(
                drivetrain.hubOrbitRestrictedRadiusCommand(fullYVelCtrl, fullXVelCtrl, Rotation2d.fromDegrees(180),
                        Inches.of(147), Meters.of(1.75)));

        driverCtrl.x().whileTrue(
                drivetrain.travelSetSpeedCmd(() -> MetersPerSecond.zero(), () -> MetersPerSecond.of(2),
                        Rotation2d.fromDegrees(90)));

        driverCtrl.y().whileTrue(
                drivetrain.trenchAlignCmd(fullXVelCtrl, Rotation2d.kZero, FieldLayout.blueTrenchRight)
        );

        // driverCtrl.y().whileTrue(
        //         drivetrain.trenchPathAlignCmd(FieldLayout.blueTrenchRight)
        // );

        driverCtrl.leftTrigger(.1).whileTrue(
                drivetrain.towerAlignCommand(fullYVelCtrl, Rotation2d.fromDegrees(180),new Translation2d(Inches.of(-11.25) ,Inches.of(-40)))
        );

          driverCtrl.rightTrigger(.1).whileTrue(
                drivetrain.towerAlignCommand(fullYVelCtrl, Rotation2d.fromDegrees(0), new Translation2d(Inches.of(2.15) ,Inches.of(40)))
        );

        // Pose Reset
        driverCtrl.pov(0).onTrue(drivetrain.runOnce(
                () -> drivetrain.resetPose(
                        new Pose2d(
                                FieldLayout.getHubCenterFront(),
                                Rotation2d.fromDegrees(FieldLayout.getForwardAngle().in(Degrees) + 180)))));

        // Idle motors when disabled
        RobotModeTriggers.disabled().whileTrue(drivetrain.idleCmd());

        // Drivetrain test Controls
        testMode.and(driverCtrl.a()).whileTrue(drivetrainTestCmds.runCommandCmd());
    }

    /**
     * Retrieves the selected auton
     * 
     * @return the selected auton
     */
    public Command getAutonomousCommand() {
        SmartDashboard.putString("Current Command", autoChooser.getSelected().getName());
        return autoChooser.getSelected();
    }
}
