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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CommandSelector;
import frc.robot.commands.auton.PointToPointAutons;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagPipeline;
import frc.robot.subsystems.CameraSim;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IntakeAngle;
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
    private final ShooterWheel shooterWheel = new ShooterWheel(Constants.shooterMotorLeaderID,
            Constants.shooterMotorFollowerID, TunerConstants.kCANBus, Constants.shooterWheelGearRatio, drivetrain);
    private final ShooterHood shooterHood = new ShooterHood(Constants.shooterHoodMotor,
            Constants.shooterHoodEncoderID, TunerConstants.kCANBus, Constants.shooterHoodGearRatio, drivetrain);
    private final ShooterManagement shooterMngt = new ShooterManagement(drivetrain, indexer, shooterWheel, shooterHood, intakeRoller, intakeAngle);

    private final PointToPointAutons pointToPointAutons = new PointToPointAutons(drivetrain, indexer, intakeAngle, intakeRoller, shooterMngt, shooterWheel);
    // private final Climber climber = new Climber(0, 0, TunerConstants.kCANBus, 0);

    // Pathplanner
    SendableChooser<Command> autoChooser;

    // Mutable Units
    private MutLinearVelocity xVel = MetersPerSecond.mutable(0);
    private MutLinearVelocity yVel = MetersPerSecond.mutable(0);
    private MutAngularVelocity rVel = RotationsPerSecond.mutable(0);

    // Triggers
    private Trigger testMode = new Trigger(DriverStation::isTest);

    // Cameras
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

    // Test Command Lists
    private final CommandSelector drivetrainTestCmds = new CommandSelector()
            .addEntry("SysID Translation Quasistatic Forward",
                    drivetrain.sysIdTranslationQuasistatic(Direction.kForward))
            .addEntry("SysID Translation Quasistatic Reverse",
                    drivetrain.sysIdTranslationQuasistatic(Direction.kReverse))
            .addEntry("SysID Translation Dynamic Forward",
                    drivetrain.sysIdTranslationDynamic(Direction.kForward))
            .addEntry("SysID Translation Dynamic Forward",
                    drivetrain.sysIdTranslationDynamic(Direction.kReverse))
            .addEntry("SysID Steer Quasistatic Forward",
                    drivetrain.sysIdSteerQuasistatic(Direction.kForward))
            .addEntry("SysID Steer Quasistatic Reverse",
                    drivetrain.sysIdSteerQuasistatic(Direction.kReverse))
            .addEntry("SysID Steer Dynamic Forward", drivetrain.sysIdSteerDynamic(Direction.kForward))
            .addEntry("SysID Steer Dynamic Forward", drivetrain.sysIdSteerDynamic(Direction.kReverse))
            .addEntry("SysID Rotation Quasistatic Forward",
                    drivetrain.sysIdRotationQuasistatic(Direction.kForward))
            .addEntry("SysID Rotation Quasistatic Reverse",
                    drivetrain.sysIdRotationQuasistatic(Direction.kReverse))
            .addEntry("SysID Rotation Dynamic Forward", drivetrain.sysIdRotationDynamic(Direction.kForward))
            .addEntry("SysID Rotation Dynamic Forward", drivetrain.sysIdRotationDynamic(Direction.kReverse))
            .sendToDashboard("Drivetrain Test Commands");

    private final CommandSelector shooterTestCmds = new CommandSelector()
            .addEntry("SysID Quasistatic Forward", shooterWheel.sysIdQuasistatic(Direction.kForward))
            .addEntry("SysID Quasistatic Reverse", shooterWheel.sysIdQuasistatic(Direction.kReverse))
            .addEntry("SysID Dynamic Forward", shooterWheel.sysIdDynamic(Direction.kForward))
            .addEntry("SysID Dynamic Reverse", shooterWheel.sysIdDynamic(Direction.kReverse))
            .addEntry("SysID Full", Commands.sequence(
                    shooterWheel.sysIdQuasistatic(Direction.kForward),
                    shooterWheel.sysIdQuasistatic(Direction.kReverse),
                    shooterWheel.sysIdDynamic(Direction.kForward),
                    shooterWheel.sysIdDynamic(Direction.kReverse)))
            .addEntry("Shooter Speed Test", shooterWheel.getTestCommand())
            .sendToDashboard("Shooter Test Commands");

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
        NamedCommands.registerCommand("LeftTowerAlign Command", drivetrain.towerAlignCommand(fullYVelCtrl,
                Rotation2d.fromDegrees(180), new Translation2d(Inches.of(-11.25), Inches.of(-40))));
        NamedCommands.registerCommand("RightTowerAlign Command", drivetrain.towerAlignCommand(fullYVelCtrl,
                Rotation2d.fromDegrees(0), new Translation2d(Inches.of(2.15), Inches.of(40))));
        NamedCommands.registerCommand("Hub Orbit Command", hubOrbitRangeCmd());
        NamedCommands.registerCommand("ShooterWheel Command", shooterMngt.hubAutoShotCmd());

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auton Chooser", autoChooser);
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

        // operatorCtrl.povUp().whileTrue(intakeAngle.
        //         setBangBangOscilateLimitCmd(RotationsPerSecond.of(0.2), 
        //                 Degrees.of(0), 
        //                 Degrees.of(110))
        //         );

        operatorCtrl.start().whileTrue(intakeRoller.intakeOutCmd());

        operatorCtrl.povUp().whileTrue(intakeAngle.setVoltageCmd(Volts.of(2)));

        operatorCtrl.povDown().whileTrue(intakeAngle.setVoltageCmd(Volts.of(-2)));

        operatorCtrl.b().onTrue(intakeAngle.extendCmd());

        operatorCtrl.x().onTrue(intakeAngle.retractCmd());

        operatorCtrl.y().onTrue(intakeAngle.highOscillate());

        operatorCtrl.a().onTrue(intakeAngle.lowOscillate());

        operatorCtrl.povRight().whileTrue(indexer.indexForwardCmd());

        operatorCtrl.povLeft().whileTrue(indexer.indexReverseCmd());

        // DRIVER

        driverCtrl.leftTrigger().onTrue(intakeAngle.setPositionCmd(Constants.intakeOutAngle));

        // TEST CONTROLS

        // operatorCtrl.axisGreaterThan(0, 0.1)
        // .whileTrue(intakeAngle.setOscilateProgressionTestCmd(Degrees.of(20),
        // Seconds.of(0.5),() -> operatorCtrl.getRawAxis(0)));
        // operatorCtrl.rightTrigger(.1).whileTrue(indexer.setVoltageCmd(Volts.of(6)));
    }

    private void shooterBindings() {
        Command shooterHoodSysId = shooterHood.sysIdQuasistaticLimited(Direction.kForward)
                .andThen(shooterHood.sysIdQuasistaticLimited(Direction.kReverse))
                .andThen(shooterHood.sysIdDynamicLimited(Direction.kForward))
                .andThen(shooterHood.sysIdDynamicLimited(Direction.kReverse));

        Command shooterWheelSysId = shooterWheel.sysIdQuasistatic(Direction.kForward)
                .andThen(shooterWheel.sysIdQuasistatic(Direction.kReverse))
                .andThen(shooterWheel.sysIdDynamic(Direction.kForward))
                .andThen(shooterWheel.sysIdDynamic(Direction.kReverse));

        // OPERATOR

        // operatorCtrl.rightBumper().whileTrue(shooterWheel.hubShotCmd());

        // operatorCtrl.rightBumper().whileTrue(shooterMngt.hubBangBangShotCmd());

        operatorCtrl.rightTrigger(.1).whileTrue(shooterMngt.hubAutoShotCmd());
        
        operatorCtrl.rightBumper().whileTrue(shooterMngt.hubSequentialShotCmd());

        // operatorCtrl.rightBumper().whileTrue(intakeAngle.setBangBangOscilateLimitCmd(RotationsPerSecond.of(.2), Degrees.of(10), Degrees.of(80)));

        operatorCtrl.leftTrigger(.1).whileTrue(shooterMngt.IdleShotPrepCmd());

        operatorCtrl.back().whileTrue(shooterMngt.passShotCmd());

        // DRIVER

        driverCtrl.rightTrigger(.1).whileTrue(shooterMngt.hubAutoShotCmd());

        driverCtrl.rightBumper().whileTrue(shooterMngt.passShotCmd());

        // TEST CONTROLS

        // operatorCtrl.back().onTrue(shooterHoodSysId);

        // operatorCtrl.rightTrigger(0.1).whileTrue(shooterWheel.setVoltageCmd(Volts.of(2)));
        // operatorCtrl.povLeft().onTrue(shooterWheel.setCurrentCmd(() -> Amps.of(60 *
        // operatorCtrl.getLeftY())));
        // operatorCtrl.povLeft().onTrue(shooterHood.setVoltageCmd(() -> Volts.of(12 *
        // operatorCtrl.getLeftY())));
        // operatorCtrl.povRight().onTrue(shooterHood.setPositionCmd(Degrees.of(-20)));
        // operatorCtrl.povLeft().onTrue(shooterHood.setPositionCmd(Degrees.of(-40)));
        // operatorCtrl.povDown().whileTrue(shooterHood.setVoltageCmd(() -> Volts.of(12
        // * operatorCtrl.getRightY())));
        // operatorCtrl.povLeft().whileTrue(shooterHood.setVoltageCmd(Volts.of(2)));
        // operatorCtrl.povRight().whileTrue(shooterHood.setVoltageCmd(Volts.of(-2)));
        // //operatorCtrl.rightBumper().whileTrue(shooter.setTorqueVelocityTestCmd(() ->
        // RotationsPerSecond.of(60)));
        // operatorCtrl.rightTrigger(0.1).whileTrue(shooterMngt.hubShotCmd());
        // operatorCtrl.a().whileTrue(indexer.setVoltageCmd(Volts.of(-12)));
        // operatorCtrl.b().whileTrue(indexer.setVoltageCmd(Volts.of(12)));

    }

    /**
     * Configures drivetrain bindings
     */
    private void drivetrainBindings() {
        // Set default drivetrain command
        drivetrain.setDefaultCommand(
                drivetrain.getDriveCmd(
                        slowXVelCtrl,
                        slowYVelCtrl,
                        slowRVelCtrl));

        // Fast Drive Command
        driverCtrl.leftBumper().whileTrue(
                drivetrain.getDriveCmd(
                        fullXVelCtrl,
                        fullYVelCtrl,
                        fullRVelCtrl));

        // Track Goal
        // driverCtrl.leftBumper().whileTrue(
        //         drivetrain.lookAtPointCmd(
        //                 fullXVelCtrl,
        //                 fullYVelCtrl,
        //                 FieldLayout.Hub.getHubCenter(),
        //                 Rotation2d.fromDegrees(180)));

        // driverCtrl.a().whileTrue(
        //         drivetrain.hubOrbitCommand(fullYVelCtrl, Rotation2d.fromDegrees(180), Inches.of(92)));

        driverCtrl.a().whileTrue(autoAimCmd());

        // driverCtrl.x().whileTrue(passOrbitCmd());
        // driverCtrl.x().whileTrue(
        // drivetrain.travelSetSpeedCmd(() -> MetersPerSecond.zero(), () ->
        // MetersPerSecond.of(2),
        // Rotation2d.fromDegrees(90)));

        // Translation2d offsetBackHub = FieldLayout.Hub.getHubCenterBack(new
        // Translation2d(Inches.of(35.0/2.0 + 10), Meters.zero()));
        // driverCtrl.x().whileTrue(
        // drivetrain.yAxisAlignCmd(
        // fullYVelCtrl, Rotation2d.fromDegrees(90),
        // offsetBackHub));

        driverCtrl.y().whileTrue(
                Commands.runOnce(
                        () -> drivetrain.initialRotationHelper(
                                FieldLayout.getInwardAngle(drivetrain.getPose2d())),
                        drivetrain)
                        .andThen(drivetrain.hubBackAlignCmd(slowYVelCtrl)));

        // driverCtrl.y().whileTrue(
        // drivetrain.xAxisAlignCmd(fullXVelCtrl, Rotation2d.fromDegrees(90),
        // FieldLayout.Trench.blueTrenchRight)
        // );

        driverCtrl.b().whileTrue(
                drivetrain.trenchAlignCmd(slowXVelCtrl));

        // driverCtrl.leftTrigger(.1).whileTrue(
        // drivetrain.towerAlignCommand(fullYVelCtrl, Rotation2d.fromDegrees(180),new
        // Translation2d(Inches.of(-11.25) ,Inches.of(-40)))
        // );

        // driverCtrl.rightTrigger(.1).whileTrue(
        // drivetrain.towerAlignCommand(fullYVelCtrl, Rotation2d.fromDegrees(0), new
        // Translation2d(Inches.of(2.15) ,Inches.of(40)))
        // );

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
                Inches.of(147), Meters.of(1.75));
    }

    public Command hubOrbitCmd(){
        return drivetrain.hubOrbitCommand(slowYVelCtrl, Rotation2d.fromDegrees(180), Meters.of(2.25));
    }

    public Command passOrbitCmd() {
        return drivetrain.passOrbitRestrictedRadiusCommand(() -> slowYVelCtrl.get().times(-1),
                () -> slowXVelCtrl.get().times(-1), Rotation2d.fromDegrees(180),
                FieldLayout.fieldCenterX, FieldLayout.fieldCenterX.minus(Inches.of(130)));
    }

    public Command autoAimCmd(){
        return Commands.either(
                hubOrbitCmd(), 
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
        SmartDashboard.putString("Current Command", autoChooser.getSelected().getName());
        return autoChooser.getSelected();
    }

    public Command getP2PAutononomousCmd(){
        return pointToPointAutons.getTestAuto1();
    }
    
}
