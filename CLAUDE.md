# FRC Team 2960 — Robot Debugging Agent

## Stack
- WPILib 2026 / GradleRIO · Java 17
- AdvantageKit (LoggedRobot, @AutoLogOutput, WPILOGWriter/WPILOGReader)
- CTRE Phoenix 6: TalonFX (Kraken X60), Pigeon 2.0, CANivore
- PathPlanner + custom P2P/P2PC autons
- Swerve drivetrain (CommandSwerveDrivetrain from CTRE)

## Dependency install (once)
```
pip install robotpy-wpiutil
```

## Key build commands
```
.\gradlew.bat build                   # compile check
.\gradlew.bat test                    # unit tests
.\gradlew.bat simulateJava            # run simulation (triggers AKit replay when in sim)
```

## AKit log replay workflow
Robot logs are written to USB at `/U/logs/` on the RoboRIO.
Copy the desired `.wpilog` file into the `logs/` folder of this project.

To replay:
1. Place the `.wpilog` file in `logs/`
2. Run `.\gradlew.bat simulateJava`
3. When prompted, select the log file in AdvantageScope
4. The replay output is written as `logs/<original-name>_sim.wpilog`

## Closed-loop debugging loop protocol
Follow these steps in order. Do not skip steps or declare done early.

### Step 1 — Read the bug description
Check `bugs/` for a markdown file describing the observed behavior.
If none exists, ask the user to describe the problem before proceeding.

### Step 2 — Catalog the log
```
python tools/parse_wpilog.py logs/<file>.wpilog --catalog
```
Identify signal paths relevant to the problem. AKit signals live under `/RealOutputs/`.

### Step 3 — Find the anomaly
```
python tools/parse_wpilog.py logs/<file>.wpilog --anomalies \
    --signals ShooterWheel Indexer IntakeAngle IntakeRoller CommandSwerveDrivetrain \
    --start <T-10> --end <T+30>
```

### Step 4 — Extract before-fix CSV
```
python tools/parse_wpilog.py logs/<file>.wpilog \
    --signals <relevant subsystems> \
    --start <start> --end <end> \
    --out /tmp/before.csv
```

### Step 5 — Write your plan
Before touching any code, write your hypothesis and plan to `tasks/fix-plan.md`:
```
## Hypothesis
<what you think is happening and why>

## Plan
1. <step 1>
2. <step 2>

## Files to change
- src/main/java/frc/robot/...
```

### Step 6 — Implement the fix
Edit only the files listed in your plan. Build after every change:
```
.\gradlew.bat build
```
Fix all compiler errors before proceeding.

### Step 7 — Run log replay
```
.\gradlew.bat simulateJava
```
Select `logs/<file>.wpilog` when prompted. Replay output: `logs/<file>_sim.wpilog`.

### Step 8 — Extract after-fix CSV
```
python tools/parse_wpilog.py logs/<file>_sim.wpilog \
    --signals <same as step 4> \
    --start <start> --end <end> \
    --out /tmp/after.csv
```

### Step 9 — Compare
```
python tools/compare_signals.py \
    --before /tmp/before.csv \
    --after /tmp/after.csv \
    --target-signals <anomalous signals> \
    --regression-signals CommandSwerveDrivetrain \
    --output-json /tmp/result.json
```
Exit code 0 = PASS → commit to Testing_Area and close the bug.
Exit code 1 = FAIL → update `tasks/fix-plan.md` with new findings and return to Step 5.

**Never declare a task done until compare_signals.py exits 0.**

---

## Codebase structure
```
src/main/java/frc/robot/
  Robot.java                   — LoggedRobot entry point; AKit setup + replay
  RobotContainer.java          — Subsystem wiring + button bindings
  Constants.java               — PID gains, thresholds, lookup tables
  FieldLayout.java             — Field geometry, hub/feed positions
  subsystems/
    CommandSwerveDrivetrain    — CTRE swerve, pose estimator, PathPlanner integration
    ShooterWheel.java          — TalonFX shooter (MotionMagicVelocityTorqueCurrentFOC)
    ShooterManagement.java     — Coordinates shooter + indexer + intake for shot sequences
    Indexer.java               — Ball indexing
    IntakeAngle.java           — Intake angle control
    IntakeRoller.java          — Intake roller
    Climber.java               — Climber
    AprilTagPipeline.java      — Vision (PhotonVision)
    MatchPeriodTracking.java   — Match phase state machine
  commands/
    auton/PointToPointAutons   — P2PC autonomous routines
    CommandSelector.java
  Util/
    CustomSwerveRequests/      — FieldCentricGoToPoint, CircularOrbit, etc.
    ShotSpeedTable.java        — Distance → shooter RPM interpolation
    WaypointFactory.java
```

## Key AKit signal paths
All subsystem outputs log under `/RealOutputs/<SubsystemName>/<MethodName>`.

| Filter keyword            | Signals covered                                          |
|---------------------------|----------------------------------------------------------|
| ShooterWheel              | Velocity, RPM, TargetVelocity, Voltage, StatorCurrent, SupplyCurrent |
| Indexer                   | Indexer motor state                                      |
| IntakeAngle               | Intake angle position/velocity                           |
| IntakeRoller              | Intake roller state                                      |
| CommandSwerveDrivetrain   | Pose, module states, gyro                                |
| SystemStats               | BatteryVoltage, CANBusUtilization, MemoryUsedMB          |

## Subsystem conventions
- All TalonFX motors use Phoenix 6 API (not v5 / WPI wrappers)
- Shooter uses `MotionMagicVelocityTorqueCurrentFOC` (Slot 1) for steady-state, duty-cycle for spinup
- `@AutoLogOutput` on public methods → auto-logged by AKit annotation processor
- `Logger.recordOutput(key, value)` for manual logging points
- SmartDashboard is display-only — never use it for control-flow decisions

## Branch rules
- Always commit to Testing_Area only
- Never touch or merge into main
- Never run `.\gradlew.bat deploy` unless the user explicitly asks to deploy to the robot
- Never edit `TunerConstants.java` unless specifically asked (auto-generated by Phoenix Tuner X)
