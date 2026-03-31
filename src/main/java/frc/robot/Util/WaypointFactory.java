package frc.robot.Util;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.FieldLayout;

/**
 * Factory for creating field-relative waypoints that automatically resolve to the
 * correct Pose2d at runtime based on alliance color and/or field axis mirroring.
 *
 * <p>All waypoints are defined once on the BLUE side. The {@link AllianceFlip} policy
 * on each waypoint controls how it is transformed when the robot is on the Red alliance
 * or when a Y-axis mirror is needed.
 *
 * <p>Usage:
 * <pre>
 *   // Resolves to blue or red side automatically
 *   Waypoint start = WaypointFactory.of(FieldLayout.Trench.blueTrenchRight, AllianceFlip.ALLIANCE);
 *
 *   // Resolve at runtime and pass to drivetrain
 *   drivetrain.goToPointCmd(start);          // if drivetrain accepts Supplier<Pose2d>
 *   drivetrain.goToPointCmd(start.get());    // resolve immediately if needed
 * </pre>
 */
public final class WaypointFactory {

    private WaypointFactory() {}

    // -------------------------------------------------------------------------
    // Flip policy
    // -------------------------------------------------------------------------

    /**
     * Determines how a blue-side Pose2d is transformed for other field configurations.
     *
     * <p>Field coordinate convention (WPILib standard):
     * <ul>
     *   <li>+X points from blue wall toward red wall (length axis)
     *   <li>+Y points from the right side of the blue driver station toward the left
     *   <li>Rotation follows right-hand rule (CCW positive when viewed from above)
     * </ul>
     *
     * <p>Mirror types:
     * <ul>
     *   <li>{@link #NONE}     – No transformation. Use for field-center-symmetric points.
     *   <li>{@link #ALLIANCE} – Mirror across the field's X midline (flip Y and heading).
     *                           Used for alliance-specific positions (trench, hub, tower).
     *   <li>{@link #X_AXIS}  – Mirror across the field's Y midline (flip X and heading).
     *                           Used for center-line notes that are symmetric left/right
     *                           but need to stay on the same alliance side.
     *   <li>{@link #BOTH}    – Apply both mirrors (180° rotation about field center).
     *                           Used for points that need full field symmetry.
     * </ul>
     */
    public enum AllianceFlip {
        /** No transformation applied regardless of alliance. */
        NONE,

        /**
         * Rotates 180° around the field center (alliance boundary flip).
         * Blue-right becomes red-left, etc. Use for most alliance-specific waypoints.
         */
        ALLIANCE,

        /**
         * Reflects over the field's X axis (the field's long center line).
         * Flips Y position top-to-bottom while staying on the same alliance side.
         * Use this to mirror an entire auton from one side of the field to the other.
         */
        X_AXIS,

        /**
         * Applies {@link #X_AXIS} first, then {@link #ALLIANCE}.
         * Use when you want to run a mirrored auton on the opposite alliance.
         */
        BOTH
    }

    // -------------------------------------------------------------------------
    // Waypoint type
    // -------------------------------------------------------------------------

    /**
     * A lazily-resolved field waypoint. Calling {@link #get()} applies the configured
     * {@link AllianceFlip} policy against the current DriverStation alliance and returns
     * the correct {@link Pose2d}.
     *
     * <p>The underlying {@link Pose2d} is always stored in blue-alliance coordinates.
     */
    public static final class Waypoint implements Supplier<Pose2d> {

        private final Pose2d bluePose;
        private final AllianceFlip flip;

        private Waypoint(Pose2d bluePose, AllianceFlip flip) {
            this.bluePose = bluePose;
            this.flip = flip;
        }

        /**
         * Resolves this waypoint to a {@link Pose2d} for the current alliance.
         *
         * @return the transformed pose, or the original blue pose if alliance is not set
         */
        @Override
        public Pose2d get() {
            boolean redAlliance = FieldLayout.isRedAlliance();

            return switch (flip) {
                case NONE     -> bluePose;
                case ALLIANCE -> redAlliance ? flipAlliance(bluePose) : bluePose;
                case X_AXIS   -> flipXAxis(bluePose);
                case BOTH     -> redAlliance ? flipAlliance(flipXAxis(bluePose)) : flipXAxis(bluePose);
            };
        }

        /** Returns the raw blue-side pose, regardless of alliance. */
        public Pose2d getBluePose() {
            return bluePose;
        }

        /** Returns the flip policy applied by this waypoint. */
        public AllianceFlip getFlipPolicy() {
            return flip;
        }

        /**
         * Returns a new Waypoint with the same pose but a new flip policy.
         * 
         * @param flip the new flip policy to apply.
         * @return the new Waypoint with the applied flip policy.
         */
        public Waypoint withFlipPolicy(AllianceFlip flip){
            return new Waypoint(this.getBluePose(), flip);
        }

        /**
         * Returns a new Waypoint with the same position but a different heading.
         *
         * @param rotation the new blue-side heading (will be flipped with the pose)
         */
        public Waypoint withRotation(Rotation2d rotation) {
            return new Waypoint(new Pose2d(bluePose.getTranslation(), rotation), flip);
        }

        /**
         * Returns a new Waypoint translated by the given offset (in blue-alliance coordinates).
         *
         * @param dx X offset in meters
         * @param dy Y offset in meters
         */
        public Waypoint translated(double dx, double dy) {
            Translation2d shifted = bluePose.getTranslation().plus(new Translation2d(dx, dy));
            return new Waypoint(new Pose2d(shifted, bluePose.getRotation()), flip);
        }

        /**
         * Returns a new Waypoint translated by the given offset (in blue-alliance coordinates).
         *
         * @param dx X offset distance
         * @param dy Y offset distance
         */
        public Waypoint translated(Distance dx, Distance dy) {
            Translation2d shifted = bluePose.getTranslation().plus(new Translation2d(dx, dy));
            return new Waypoint(new Pose2d(shifted, bluePose.getRotation()), flip);
        }
    }

    // -------------------------------------------------------------------------
    // Factory methods
    // -------------------------------------------------------------------------

    /**
     * Creates a waypoint from a {@link Pose2d} with the given flip policy.
     *
     * @param bluePose pose defined in blue-alliance coordinates
     * @param flip     how to transform the pose for other alliance/axis configurations
     */
    public static Waypoint of(Pose2d bluePose, AllianceFlip flip) {
        return new Waypoint(bluePose, flip);
    }

    /**
     * Creates a waypoint from a {@link Translation2d} and heading, with the given flip policy.
     *
     * @param translation position in blue-alliance coordinates
     * @param rotation    heading in blue-alliance coordinates
     * @param flip        how to transform the pose for other alliance/axis configurations
     */
    public static Waypoint of(Translation2d translation, Rotation2d rotation, AllianceFlip flip) {
        return new Waypoint(new Pose2d(translation, rotation), flip);
    }

    /**
     * Creates a waypoint with {@link AllianceFlip#NONE} — position is absolute and
     * never transformed.
     */
    public static Waypoint fixed(Pose2d pose) {
        return new Waypoint(pose, AllianceFlip.NONE);
    }

    /**
     * Creates an alliance-flipped waypoint from a {@link Translation2d}.
     * Shorthand for the most common case.
     *
     * @param translation blue-alliance position
     * @param rotation    blue-alliance heading
     */
    public static Waypoint alliance(Translation2d translation, Rotation2d rotation) {
        return of(translation, rotation, AllianceFlip.ALLIANCE);
    }

    // -------------------------------------------------------------------------
    // Flip math
    // -------------------------------------------------------------------------

    private static final Translation2d FIELD_CENTER = new Translation2d(
        FieldLayout.fieldLength.div(2),
        FieldLayout.fieldWidth.div(2)
    );

    /**
     * Rotates the pose 180° around the field center.
     * Correctly handles both position and heading for full alliance symmetry.
     */
    private static Pose2d flipAlliance(Pose2d pose) {
        Translation2d translation2d = pose.getTranslation().rotateAround(FIELD_CENTER, Rotation2d.k180deg);
        return new Pose2d(translation2d, pose.getRotation());
    }

    /**
     * Reflects the pose over the field's long center line (the X axis).
     *
     * <p>Translation: Y is mirrored across the field's Y midpoint (fieldWidth - y),
     * X is unchanged.
     *
     * <p>Rotation: only the sin component is negated, which reflects the heading
     * across the X axis without rotating it. For example, 45° becomes 135°, and
     * 270° becomes 90°. This is intentionally NOT a 180° rotation — a reflection
     * cannot be expressed as a rotation.
     *
     * <p>Example: a pose at (3, 2) on a 16x8 field becomes (3, 6).
     * A heading of 45° becomes 135°.
     */
    private static Pose2d flipXAxis(Pose2d pose) {
        double fieldWidthMeters = FieldLayout.fieldWidth.in(Meters);
 
        // Mirror Y across the field's center line, X stays the same
        double newY = fieldWidthMeters - pose.getY();
 
        // Reflect heading across the X axis: negate sin (Y component), keep cos (X component)
        Rotation2d newRot = new Rotation2d(
             pose.getRotation().getCos(),
            -pose.getRotation().getSin()
        );
 
        return new Pose2d(pose.getX(), newY, newRot);
    }
}
