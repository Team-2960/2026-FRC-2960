## Task
Current limit optimization — reduce brownouts observed across 7 MITR2 matches.

## Findings from log analysis (7 matches, 1048.8s enabled time)
- **5 of 7 matches had brownouts** (battery below 6.75V); worst: 6.01V in q27
- **Indexer**: stator peak 166A, p99 113A, p95 85A — 65% of visible supply budget
  - Currently: supply 80A only, NO stator limit
- **ShooterWheel**: stator peak 158A, p99 119A, p95 101A
  - Currently: supply 80A only, NO stator limit
- **IntakeRoller**: current limits are COMMENTED OUT — no protection at all
- **IntakeAngle, ShooterHood, Climber**: supply 80A only, no stator limit
- **Drivetrain (TunerConstants)**: supply 60A + stator 60A — do not touch

## CTRE guidance
- Stator limits restrict torque and indirectly reduce supply draw
- Supply limits protect the battery/breakers
- Stator limits are the primary tool for preventing brownouts

## Plan
1. Add per-subsystem stator + supply limit constants to Constants.java
2. Apply stator limit to ShooterWheel (both motors)
3. Apply stator limit to Indexer
4. Re-enable and add stator limit to IntakeRoller
5. Apply stator limit to IntakeAngle
6. Apply stator limit to ShooterHood
7. Apply stator limit to Climber
8. Build

## Recommended values
| Subsystem    | Stator Limit | Supply Limit | Basis |
|-------------|-------------|-------------|-------|
| ShooterWheel | 135A | 80A | p99 × 1.10, rounded up to 5 |
| Indexer      | 125A | 80A | p99 × 1.10, rounded up to 5 |
| IntakeRoller | 40A  | 40A | CTRE intake example; no data |
| IntakeAngle  | 80A  | 60A | Position servo; conservative |
| ShooterHood  | 40A  | 40A | Small position servo |
| Climber      | 80A  | 80A | Needs to lift robot |

## Files to change
- src/main/java/frc/robot/Constants.java
- src/main/java/frc/robot/subsystems/ShooterWheel.java
- src/main/java/frc/robot/subsystems/Indexer.java
- src/main/java/frc/robot/subsystems/IntakeRoller.java
- src/main/java/frc/robot/subsystems/IntakeAngle.java
- src/main/java/frc/robot/subsystems/ShooterHood.java
- src/main/java/frc/robot/subsystems/Climber.java
