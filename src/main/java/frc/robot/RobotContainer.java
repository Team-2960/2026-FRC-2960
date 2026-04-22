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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib2960.subsystems.mechanisms.FlywheelConfigBase;
import frc.robot.commands.CommandSelector;
import frc.robot.commands.auton.PointToPointAutons;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CameraSim;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeRoller;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.ShooterManagement;
import frc.robot.subsystems.ShooterWheel;

public class RobotContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */

    private final Telemetry logger = new Telemetry(Constants.maxLinVel.in(MetersPerSecond));

    private final CommandXboxController driverCtrl = new CommandXboxController(0);
    private final CommandXboxController operatorCtrl = new CommandXboxController(1);

    // Physical Subsystems
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final IntakeAngle intakeAngle = new IntakeAngle(Constants.intakeAngleID, Constants.intakeAngleEncoderID,
            TunerConstants.kCANBus, Constants.intakeAngleGearRatio);
    private final IntakeRoller intakeRoller = new IntakeRoller(Constants.intakeMotorID, TunerConstants.kCANBus,
            Constants.intakeRollerGearRatio);
    private final Indexer indexer = new Indexer(Constants.indexMotorID, TunerConstants.kCANBus, Constants.indexerGearRatio, drivetrain);
    private final ShooterWheel shooterWheel = new ShooterWheel(Constants.shooterWheelConfig);
    private final ShooterManagement shooterMngt = new ShooterManagement(drivetrain, indexer, shooterWheel, intakeRoller, intakeAngle);

    private final PointToPointAutons pointToPointAutons = new PointToPointAutons(drivetrain, indexer, intakeAngle, intakeRoller, shooterMngt, shooterWheel);
    // private final Climber climber = new Climber(0, 0, TunerConstants.kCANBus, 0);

    // Pathplanner
    SendableChooser<Command> autoChooser;

    //P2P Auton
    SendableChooser<Command> p2pAutoChooser;

    // Mutable Units
    private MutLinearVelocity xVel = MetersPerSecond.mutable(0);
    private MutLinearVelocity yVel = MetersPerSecond.mutable(0);
    private MutAngularVelocity rVel = RotationsPerSecond.mutable(0);

    // Triggers
    private Trigger testMode = new Trigger(DriverStation::isTest);

    // Cameras
    private final AprilTagPipeline sideCamera = new AprilTagPipeline(
        drivetrain, 
        Constants.sideCameraSettings, 
        "SideCamera", 
        "SideCamera");

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
            .mut_times(MathUtil.applyDeadband(-driverCtrl.getHID().getLeftY(), 0.0));
    private Supplier<LinearVelocity> fullYVelCtrl = () -> yVel.mut_replace(Constants.maxLinVel)
            .mut_times(MathUtil.applyDeadband(-driverCtrl.getHID().getLeftX(), 0.0));
    private Supplier<AngularVelocity> fullRVelCtrl = () -> rVel.mut_replace(Constants.maxAngVel)
            .mut_times(MathUtil.applyDeadband(-driverCtrl.getHID().getRightX(), 0.0));

    private Supplier<LinearVelocity> slowXVelCtrl = () -> xVel.mut_replace(Constants.slowdownLinVel)
            .mut_times(-driverCtrl.getHID().getLeftY());
    private Supplier<LinearVelocity> slowYVelCtrl = () -> yVel.mut_replace(Constants.slowdownLinVel)
            .mut_times(-driverCtrl.getHID().getLeftX());
    private Supplier<AngularVelocity> slowRVelCtrl = () -> rVel.mut_replace(Constants.slowdownAngVel)
            .mut_times(-driverCtrl.getHID().getRightX());

    /**
     * Constructor
     */
    public RobotContainer() {
        // Init PathPlanner
        initPathPlanner();

        // Configure Robot Bindings
        configureBindings();

        // Initialize drivetrain telemetry
        if (Robot.isSimulation()) {
            drivetrain.registerTelemetry(
                    (telemetryFunction) -> logger.telemeterize(telemetryFunction, true));
        } else {
            drivetrain.registerTelemetry(
                    (telemetryFunction) -> logger.telemeterize(telemetryFunction, false));
        }
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    /**
     * Initializes PathPlanner
     */
    private void initPathPlanner() {
        // Initialize Auton chooser
        NamedCommands.registerCommand("IntakeRollerIN Command", intakeRoller.intakeInCmd());
        NamedCommands.registerCommand("IntakePivOut Command", intakeAngle.extendCmd());
        NamedCommands.registerCommand("IntakePivIn Command", intakeAngle.retractCmd());
        NamedCommands.registerCommand("IntakePivShake Command", intakeAngle.lowOscillate());
        NamedCommands.registerCommand("ClimberDeploy Command", intakeAngle.setPositionCmd(Degrees.of(140)));
        NamedCommands.registerCommand("ClimberRetract Command", intakeAngle.setPositionCmd(Degrees.of(140)));
        NamedCommands.registerCommand("Hub Orbit Command", hubOrbitRangeCmd());
        NamedCommands.registerCommand("Auto Aim Shake Command", hubShakeCmd());
        NamedCommands.registerCommand("Auto Aim Command", hubOrbitRangeCmd());
        NamedCommands.registerCommand("ShooterWheel Command", shooterMngt.hubIndexAutoShotCmd());
        NamedCommands.registerCommand("Sequential Shot Command", shooterMngt.hubSequentialShotCmd());
        NamedCommands.registerCommand("IndexerBackwards Command", indexer.setVoltageCmd(Volts.of(-6)));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auton Chooser", autoChooser);

        p2pAutoChooser = pointToPointAutons.getAutonChooser();
        SmartDashboard.putData("P2P Auton Chooser", p2pAutoChooser);
    }

    /**
     * Configures robot bindings
     */
    private void configureBindings() {
        drivetrainBindings();
        intakeBindings();
        shooterBindings();
    }

    private void intakeBindings() {
        Command intakeAngleSysId = intakeAngle
                .sysIdQuasistaticLimited(Direction.kReverse, Degrees.of(0), Degrees.of(120))
                .andThen(intakeAngle.sysIdQuasistaticLimited(Direction.kForward, Degrees.of(0),
                        Degrees.of(90)))
                .andThen(intakeAngle.sysIdDynamicLimited(Direction.kReverse))
                .andThen(intakeAngle.sysIdDynamicLimited(Direction.kForward));

        Command intakeRollerSysId = intakeRoller.sysIdQuasistatic(Direction.kReverse)
                .andThen(intakeRoller.sysIdQuasistatic(Direction.kForward))
                .andThen(intakeRoller.sysIdDynamic(Direction.kReverse))
                .andThen(intakeRoller.sysIdDynamic(Direction.kForward));

        // OPERATOR

        operatorCtrl.leftBumper().whileTrue(intakeRoller.intakeInCmd());

        operatorCtrl.start().whileTrue(intakeRoller.intakeOutCmd());

        operatorCtrl.povUp().whileTrue(intakeAngle.setVoltageCmd(Volts.of(3)));

        operatorCtrl.povDown().whileTrue(intakeAngle.setVoltageCmd(Volts.of(-3)));

        operatorCtrl.b().onTrue(intakeAngle.extendCmd());

        operatorCtrl.x().onTrue(intakeAngle.retractCmd());

        operatorCtrl.y().whileTrue(shooterMngt.hubNoIntakeIndexAutoShotCmd());

        operatorCtrl.povRight().whileTrue(indexer.indexForwardCmd());

        operatorCtrl.povLeft().whileTrue(indexer.indexReverseCmd());

        // DRIVER

        driverCtrl.leftTrigger().onTrue(intakeAngle.setPositionCmd(Constants.intakeOutAngle));
    }

    private void shooterBindings() {
        Command shooterWheelSysId = shooterWheel.sysIdQuasistatic(Direction.kForward)
                .andThen(shooterWheel.sysIdQuasistatic(Direction.kReverse))
                .andThen(shooterWheel.sysIdDynamic(Direction.kForward))
                .andThen(shooterWheel.sysIdDynamic(Direction.kReverse));

        // OPERATOR

        operatorCtrl.rightTrigger(.1).whileTrue(shooterMngt.hubIndexAutoShotCmd());

        operatorCtrl.a().whileTrue(shooterMngt.passIndexAutoShotCmd());
        
        operatorCtrl.rightBumper().whileTrue(shooterMngt.hubBangBangShotCmd());

        operatorCtrl.leftTrigger(.1).whileTrue(shooterMngt.IdleShotPrepCmd());

        operatorCtrl.axisMagnitudeGreaterThan(1, 0.01).whileTrue(shooterWheel.setCurrentCmd(() -> Amps.of(operatorCtrl.getLeftY() * 10)));

        operatorCtrl.back().whileTrue(shooterMngt.hubNoIntakeIndexAutoShotCmd());

        // DRIVER

        driverCtrl.rightTrigger(.1).whileTrue(shooterMngt.hubAutoShotCmd());

        driverCtrl.rightBumper().whileTrue(shooterMngt.passShotCmd());

    }

    /**
     * Configures drivetrain bindings
     */
    private void drivetrainBindings() {
        // Set default drivetrain command
        drivetrain.setDefaultCommand(
                drivetrain.getDriveCmd(
                        () -> driverCtrl.getHID().getLeftBumperButton() ? fullXVelCtrl.get() : slowXVelCtrl.get(),
                        () -> driverCtrl.getHID().getLeftBumperButton() ? fullYVelCtrl.get() : slowYVelCtrl.get(),
                        () -> driverCtrl.getHID().getLeftBumperButton() ? fullRVelCtrl.get() : slowRVelCtrl.get()));

        driverCtrl.a().whileTrue(autoAimCmd());

        driverCtrl.y().whileTrue(
                Commands.runOnce(
                        () -> drivetrain.initialRotationHelper(
                                FieldLayout.getInwardAngle(drivetrain.getPose2d())),
                        drivetrain)
                        .andThen(drivetrain.hubBackAlignCmd(() -> driverCtrl.getHID().getLeftBumperButton() ? fullYVelCtrl.get() : slowYVelCtrl.get())));

        driverCtrl.b().whileTrue(
                drivetrain.trenchAlignCmd(() -> driverCtrl.getHID().getLeftBumperButton() ? fullXVelCtrl.get() : slowXVelCtrl.get()));

        
        driverCtrl.x().whileTrue(
                drivetrain.trenchAngleAlignCmd(() -> driverCtrl.getHID().getLeftBumperButton() ? fullXVelCtrl.get() : slowXVelCtrl.get(), Rotation2d.fromDegrees(-20)));

        // Pose Reset
        driverCtrl.pov(0).onTrue(drivetrain.runOnce(
                () -> drivetrain.resetPose(
                        new Pose2d(
                                FieldLayout.Hub.getHubCenterFront(),
                                Rotation2d.fromDegrees(FieldLayout.getForwardAngle()
                                        .in(Degrees) + 180)))));

        // Idle motors when disabled
        RobotModeTriggers.disabled().whileTrue(drivetrain.idleCmd());

        // Drivetrain test Controls
        testMode.and(driverCtrl.a()).whileTrue(drivetrainTestCmds.runCommandCmd());
    }

    public Command hubOrbitRangeCmd() {
        return drivetrain.hubOrbitRestrictedRadiusCommand(slowYVelCtrl, slowXVelCtrl,
                Rotation2d.fromDegrees(180),
                Inches.of(160), Meters.of(1.75));
    }

    public Command hubOrbitCmd(){
        return drivetrain.hubOrbitCommand(slowYVelCtrl, Rotation2d.fromDegrees(180), Meters.of(2.25));
        // return drivetrain.hubOrbitShakeCommand(slowYVelCtrl, Rotation2d.k180deg, Rotation2d.fromDegrees(4), 
        //         Meters.of(2.25), Meters.of(0.05), Rotation2d.fromDegrees(2));
    }

    public Command hubShakeCmd(){
        return drivetrain.hubOrbitShakeCommand(slowYVelCtrl, Rotation2d.k180deg, Rotation2d.fromDegrees(4), 
                Meters.of(2.25), Meters.of(0.05), Rotation2d.fromDegrees(2));
    }

    public Command passOrbitCmd() {
        return drivetrain.passOrbitRestrictedRadiusCommand(() -> slowYVelCtrl.get().times(-1),
                () -> slowXVelCtrl.get().times(-1), Rotation2d.fromDegrees(180),
                FieldLayout.fieldCenterX, FieldLayout.fieldCenterX.minus(Inches.of(130)));
    }

    public Command autoAimCmd(){
        return Commands.either(
                hubOrbitRangeCmd(), 
                passOrbitCmd(), 
                () -> FieldLayout.inAllianceZone(drivetrain::getPose2d)
        );
    }

    /**
     * Retrieves the selected auton
     * 
     * @return the selected auton
     */
    public Command getAutonomousCommand() {
        SmartDashboard.putString("Current Auton", autoChooser.getSelected().getName());
        return autoChooser.getSelected();
    }

    public Command getP2PAutononomousCmd(){
        SmartDashboard.putString("Current P2P Auton", p2pAutoChooser.getSelected().getName());
        return p2pAutoChooser.getSelected();
    }
    
}
