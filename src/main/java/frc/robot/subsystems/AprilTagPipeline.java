package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldLayout;
import frc.robot.Util.AprilTagPipelineSettings;

public class AprilTagPipeline extends SubsystemBase {

    // --- Vision measurement rejection thresholds (tune on-robot) ---
    /** How far a pose may sit outside the field bounds before it is rejected. */
    private static final double FIELD_MARGIN_METERS = 0.5;
    /** A grounded robot's reported height must be near zero. */
    private static final double MAX_Z_METERS = 0.5;
    /** A grounded robot's reported pitch/roll must be near zero. */
    private static final double MAX_TILT_RADIANS = 0.5;
    /** While enabled, reject single-frame teleports larger than this (likely a bad solve). */
    private static final double MAX_POSE_JUMP_METERS = 1.5;
    /**
     * If no vision fix has been accepted for this long, bypass the jump gate so the robot can
     * recover from a bad pose (e.g. a code restart mid-match) instead of locking vision out.
     */
    private static final double VISION_RECOVERY_TIMEOUT_SECONDS = 1.0;

    private final CommandSwerveDrivetrain drive;

    private final AprilTagPipelineSettings settings;
    /** < Pipeline Settings */
    private final PhotonCamera camera;
    @SuppressWarnings("unused")
    private final String cameraName;
    /** < Camera object */
    private final PhotonPoseEstimator pose_est;
    /** < Pose Estimator */
    /** Field tag layout, loaded once and reused (loading it is expensive). */
    private final AprilTagFieldLayout fieldLayout;

    private final double maxDistance;
    private final double ambiguity_threshold;

    private Matrix<N3, N1> curStdDevs;

    private boolean aprilTagSeen;
    /** FPGA timestamp of the most recently accepted vision measurement. */
    private double lastAcceptedTimestamp = 0;

    // Advantage Scope
    private final StructArrayPublisher<Pose3d> as_aprilTags;
    private final StructPublisher<Pose3d> as_cameraPose; // Camera Pose Relative to Robot on the Field
    private final StructPublisher<Pose2d> as_estimatedCameraPose;

    private Pose3d[] aprilTagList;

    private Pose2d last_pose; // Do NOT Use for any estimates

    // Camera Simulation
    private final SimCameraProperties cameraProp;
    private final PhotonCameraSim cameraSim;

    /**
     * Constructor
     *
     * @param settings Pipeline settings
     */
    public AprilTagPipeline(CommandSwerveDrivetrain drive, AprilTagPipelineSettings settings, String cameraName, String name) {
        this.cameraName = cameraName;
        this.drive = drive;
        this.settings = settings;

        camera = new PhotonCamera(cameraName);

        // Load the field layout exactly once and hand it to the estimator. Reusing this
        // object avoids re-parsing the layout JSON on every frame (a major loop-overrun source).
        fieldLayout = AprilTagFieldLayout.loadField(settings.field_layout);
        // The estimation strategy is chosen per-call (estimateCoprocMultiTagPose / fallback),
        // so the strategy is no longer passed to the constructor.
        pose_est = new PhotonPoseEstimator(fieldLayout, settings.robot_to_camera);

        last_pose = Pose2d.kZero; // For AdvantageScope DO NOT USE FOR ESTIMATION
        maxDistance = settings.max_dist;
        ambiguity_threshold = settings.ambiguity_threshold;

        // Advantage Scope
        as_aprilTags = NetworkTableInstance.getDefault()
                .getStructArrayTopic(cameraName, Pose3d.struct).publish();
        as_cameraPose = NetworkTableInstance.getDefault()
                .getStructTopic(cameraName + " pose", Pose3d.struct).publish();
        as_estimatedCameraPose = NetworkTableInstance.getDefault()
                .getStructTopic(cameraName + " Estimated Pose", Pose2d.struct).publish();

        // Vision Simulation. The simulated AprilTags themselves are added by CameraSim from the
        // field layout (2026 -> 36h11), so no TargetModel is configured here.
        cameraProp = new SimCameraProperties();
        cameraProp.setFPS(60);
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(70));
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);

        aprilTagList = new Pose3d[] {};
        curStdDevs = VecBuilder.fill(0, 0, 0);
    }

    /**
     * Period method. Updates pose estimation and UI.
     */
    @Override
    public void periodic() {
        updatePose();
        updateUI();
    }

    /**
     * Fuses new camera frames into the drivetrain's pose estimator.
     *
     * <p>For each unread result we take the multi-tag PnP solution computed on the coprocessor
     * when available, and otherwise fall back to the single lowest-ambiguity tag. The accepted
     * pose is filtered for plausibility and added to the Kalman filter with distance-scaled
     * standard deviations (tight for multi-tag, looser for single-tag).
     */
    private void updatePose() {
        aprilTagSeen = false;

        for (var result : camera.getAllUnreadResults()) {
            // Display state is per-frame so stale tags don't linger in AdvantageScope.
            aprilTagList = new Pose3d[0];

            // Prefer the coprocessor multi-tag PnP fix; fall back to the best single tag.
            Optional<EstimatedRobotPose> estOpt = pose_est.estimateCoprocMultiTagPose(result);
            boolean multiTag = estOpt.isPresent();
            if (estOpt.isEmpty()) {
                estOpt = pose_est.estimateLowestAmbiguityPose(result);
            }
            if (estOpt.isEmpty()) {
                continue;
            }

            EstimatedRobotPose est = estOpt.get();
            int numTags = est.targetsUsed.size();
            if (numTags == 0) {
                continue;
            }
            Pose3d estPose3d = est.estimatedPose;

            // --- Rejection gates ---

            // 1. A single-tag fix must be unambiguous enough to trust.
            if (!multiTag) {
                double ambiguity = est.targetsUsed.get(0).getPoseAmbiguity();
                if (ambiguity < 0 || ambiguity > ambiguity_threshold) {
                    continue;
                }
            }

            // 2. Reject tags that are too far away to be reliable.
            double avgDist = averageTagDistance(est);
            if (avgDist > maxDistance) {
                continue;
            }

            // 3. Reject physically impossible poses (off the field, or not flat on the floor).
            if (!isPoseOnField(estPose3d) || !isPoseFlat(estPose3d)) {
                continue;
            }

            Pose2d estPose2d = estPose3d.toPose2d();

            // 4. While enabled, reject large single-frame teleports (almost always a bad solve).
            //    Disabled-time seeding is unrestricted so the robot can snap to its start pose, and
            //    the gate is bypassed after a vision dropout so the robot can recover from a bad pose
            //    (e.g. a code restart mid-match) rather than locking vision out forever.
            boolean haveRecentFix = Timer.getFPGATimestamp()
                    - lastAcceptedTimestamp < VISION_RECOVERY_TIMEOUT_SECONDS;
            if (DriverStation.isEnabled() && haveRecentFix
                    && estPose2d.getTranslation().getDistance(drive.getPose2d().getTranslation())
                            > MAX_POSE_JUMP_METERS) {
                continue;
            }

            // --- Trust: tight for multi-tag, looser for single-tag, scaled up with distance ---
            Matrix<N3, N1> stdDevs = (multiTag ? settings.multi_tag_std : settings.single_tag_std)
                    .times(1 + (avgDist * avgDist / 20));

            drive.addVisionMeasurement(estPose2d, est.timestampSeconds, stdDevs);

            //TODO check gryo heading vs. camera heading
            SmartDashboard.putNumber("gyroHeading", drive.getPose2d().getRotation().getDegrees());
            SmartDashboard.putNumber("cameraHeading", estPose2d.getRotation().getDegrees());

            // --- Bookkeeping / logging ---
            curStdDevs = stdDevs;
            last_pose = estPose2d;
            aprilTagSeen = true;
            lastAcceptedTimestamp = Timer.getFPGATimestamp();
            aprilTagList = tagsUsedToPoses(est);
        }
    }

    /** Mean 3D camera-to-tag distance across the tags used in an estimate, in meters. */
    private double averageTagDistance(EstimatedRobotPose est) {
        double total = 0;
        for (var target : est.targetsUsed) {
            total += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        return total / est.targetsUsed.size();
    }

    /** True if the pose's translation sits within the field (plus a small margin). */
    private boolean isPoseOnField(Pose3d pose) {
        double x = pose.getX();
        double y = pose.getY();
        return x >= -FIELD_MARGIN_METERS
                && x <= FieldLayout.fieldSizeX.in(Meters) + FIELD_MARGIN_METERS
                && y >= -FIELD_MARGIN_METERS
                && y <= FieldLayout.fieldSizeY.in(Meters) + FIELD_MARGIN_METERS;
    }

    /** True if the pose looks like a grounded robot (near-zero height, pitch and roll). */
    private boolean isPoseFlat(Pose3d pose) {
        return Math.abs(pose.getZ()) <= MAX_Z_METERS
                && Math.abs(pose.getRotation().getX()) <= MAX_TILT_RADIANS
                && Math.abs(pose.getRotation().getY()) <= MAX_TILT_RADIANS;
    }

    /** Field poses of the tags actually used in an estimate, for AdvantageScope display. */
    private Pose3d[] tagsUsedToPoses(EstimatedRobotPose est) {
        Pose3d[] poses = new Pose3d[est.targetsUsed.size()];
        for (int i = 0; i < poses.length; i++) {
            poses[i] = fieldLayout.getTagPose(est.targetsUsed.get(i).getFiducialId()).orElse(new Pose3d());
        }
        return poses;
    }

    /**
     * Returns the latest standard deviations of the estimated pose, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    @AutoLogOutput(key = "Camera: CamPose {cameraName}")
    public Pose3d getRobotRelativeCamPos() {
        return new Pose3d(drive.getPose2d()).transformBy(settings.robot_to_camera);
    }

    @AutoLogOutput(key = "Camera: LastPose {cameraName}")
    public Pose3d getLastPose() {
        return new Pose3d(last_pose);
    }

    @AutoLogOutput(key = "Camera: TagSeen {cameraName}")
    public boolean getAprilTagSeen() {
        return aprilTagSeen;
    }

    public PhotonCameraSim getCameraSim() {
        return cameraSim;
    }

    public Transform3d getOffset() {
        return settings.robot_to_camera;
    }

    /**
     * Publishes vision state to AdvantageScope.
     */
    private void updateUI() {
        as_aprilTags.set(aprilTagList);
        as_cameraPose.set(getRobotRelativeCamPos());
        as_estimatedCameraPose.set(last_pose);
    }
}
