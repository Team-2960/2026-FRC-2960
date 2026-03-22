package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import frc.robot.FieldLayout;

public class MatchPeriodTracking {
    public enum MatchPeriod {
        UNDEFINED,
        AUTO,
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        END_GAME
    }

    private static final double autonEnd = 140;
    private static final double transitionEnd = 130;
    private static final double shift1End = 105;
    private static final double shift2End = 80;
    private static final double shift3End = 55;
    private static final double shift4End = 30;
    private static final double endGameEnd = 0;

    /**
     * Determines the current match period
     * 
     * @return current match period
     */
    @AutoLogOutput (key = "Current Match Period")
    public static MatchPeriod getPeriod() {
        MatchPeriod period = MatchPeriod.UNDEFINED;

        if (DriverStation.isAutonomous()) {
            period = MatchPeriod.AUTO;
        } else if (DriverStation.isTeleop()) {
            double matchTime = DriverStation.getMatchTime();

            if (matchTime > transitionEnd) {
                period = MatchPeriod.TRANSITION;
            } else if (matchTime > shift1End) {
                period = MatchPeriod.SHIFT_1;
            } else if (matchTime > shift2End) {
                period = MatchPeriod.SHIFT_2;
            } else if (matchTime > shift3End) {
                period = MatchPeriod.SHIFT_3;
            } else if (matchTime > shift4End) {
                period = MatchPeriod.SHIFT_4;
            } else if (matchTime > endGameEnd) {
                period = MatchPeriod.END_GAME;
            }
        }

        return period;
    }

    /**
     * Gets the current match time
     * 
     * @return current match time in seconds
     */
    @AutoLogOutput (key = "Match Time")
    public static  double getMatchTime() {
        double matchTime = DriverStation.getMatchTime();

        if (!DriverStation.isAutonomous() && DriverStation.getMatchType() == MatchType.None) {
            matchTime = autonEnd - matchTime;
        }

        return matchTime;
    }

    /**
     * Determines the time remaining until the end of a selected match period
     * 
     * @param period period to check
     * @return number of seconds until the end of the selected match period
     */
    @AutoLogOutput (key = "Time to End of Period: {period}")
    public static double timeToEndOfPeriod(MatchPeriod period) {
        switch (period) {
            case AUTO:
                return DriverStation.getMatchTime();
            case TRANSITION:
                return DriverStation.getMatchTime() - transitionEnd;
            case SHIFT_1:
                return DriverStation.getMatchTime() - shift1End;
            case SHIFT_2:
                return DriverStation.getMatchTime() - shift2End;
            case SHIFT_3:
                return DriverStation.getMatchTime() - shift3End;
            case SHIFT_4:
                return DriverStation.getMatchTime() - shift4End;
            case END_GAME:
                return DriverStation.getMatchTime() - endGameEnd;
            default:
                return 0;
        }
    }

    public static double getPhaseTime(){
        return timeToEndOfPeriod(getPeriod());
    }
    /**
     * Check if the current alliance won auton
     * 
     * @return True if current alliance won auton, false if it did not. Not set if
     *         input is invalid or not teleop period
     */
    @AutoLogOutput (key = "Alliance Won Auton")
    public static Boolean allianceWonAuton() {
        String autonStr = DriverStation.getGameSpecificMessage();
        Optional<Boolean> result = Optional.empty();

        if (DriverStation.isTeleopEnabled() && autonStr.length() > 0)  {
            char autonChar = autonStr.charAt(0);
            boolean isRed = FieldLayout.isRedAlliance();

            result = Optional.of(isRed && autonChar == 'A' || !isRed && autonChar == 'B');
        }

        

        if (result.isEmpty()){
            return false;
        }else{
            return result.get();
        }

    }


    /**
     * Checks if the hub is active
     * @return  true if the hub is active, false if it is not or if the robot is an invalid state
     */
    @AutoLogOutput (key = "Is Hub Active")
    public static boolean isHubActive() {
        var wonAuton = allianceWonAuton();
        switch(getPeriod()) {
            case SHIFT_1:
            case SHIFT_3:
                return !wonAuton;
            case SHIFT_2:
            case SHIFT_4:
                return wonAuton;
            default:
                return true;
        }
    }

    // TODO: Add methods to check when it is safe to shoot
}
