package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Util.AprilTagPipelineSettings;

public class AprilTagPipeline extends SubsystemBase {

    /** Height off the floor a vision solution may report before it is rejected, in meters */
    private static final double kMaxPoseHeight = 0.5;
    /** Distance past the field perimeter a vision solution may report before it is rejected, in meters */
    private static final double kFieldMargin = 0.5;

    private final CommandSwerveDrivetrain drive;

    private final AprilTagPipelineSettings settings;
    /** < Pipeline Settings */
    private final PhotonCamera camera;
    @SuppressWarnings("unused")
    private final String cameraName;
    /** < Camera object */
    private final PhotonPoseEstimator pose_est;
    /** < Pose Estimator */

    /** < Most recent estimated pose */
    private double last_timestamp;
    /** < Timestamp of the most recent pose estimation */
    private double maxDistance;
    private final double ambiguity_threshold;
    private final AprilTagFieldLayout aprilTagField;

    private Matrix<N3, N1> curStdDevs;

    // Shuffleboard
    private GenericEntry sb_PoseX;
    private GenericEntry sb_PoseY;
    private GenericEntry sb_PoseR;
    private GenericEntry sb_lastTimestamp;
    private GenericEntry sb_lastUpdatePeriod;
    private GenericEntry sb_aprilTagSeen;
    private boolean aprilTagSeen;

    //Advantage Scope
    private StructArrayPublisher<Pose3d> as_aprilTags;
    private StructPublisher<Pose3d> as_cameraPose; //Camera Pose Relative to Robot on the Field
    private StructPublisher<Pose2d> as_estimatedCameraPose;

    private Pose3d[] aprilTagList;

    private AprilTagFields field;

    private Pose2d last_pose;  //Do NOT Use for any estimates


    // Camera Simulation
    private TargetModel targetModel;
    private SimCameraProperties cameraProp;
    private PhotonCameraSim cameraSim;
    private VisionTargetSim visionTargetSim;

    // Test Values
    private double displayNum;
    private String resultsList;
    private boolean tagPresent;

    /**
     * Constructor
     * 
     * @param settings Pipeline settings
     */
    public AprilTagPipeline(CommandSwerveDrivetrain drive, AprilTagPipelineSettings settings, String cameraName, String name) {
        this.cameraName = cameraName;
        this.drive = drive;

        this.settings = settings;
        // TODO: Find non-depricated method to get April Tag Field Locations
        camera = new PhotonCamera(cameraName);
        pose_est = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(settings.field_layout),
                settings.pose_strategy,
                settings.robot_to_camera);

        last_pose = new Pose2d(); //For AdvantageScope DO NOT USE FOR ESTIMATION
        last_timestamp = 0;
        maxDistance = settings.max_dist;
        ambiguity_threshold = settings.ambiguity_threshold;
        field = settings.field_layout;

        aprilTagField = AprilTagFieldLayout.loadField(field);

        // Setup Shuffleboard
        // var layout = Shuffleboard.getTab("AprilTags")
        //         .getLayout(cameraName, BuiltInLayouts.kList)
        //         .withSize(1, 4);
        // sb_PoseX = layout.add("Pose X" + cameraName, 0).getEntry();
        // sb_PoseY = layout.add("Pose Y" + cameraName, 0).getEntry();
        // sb_PoseR = layout.add("Pose R" + cameraName, 0).getEntry();
        // sb_lastTimestamp = layout.add("Last Timestamp" + cameraName, last_timestamp).getEntry();
        // sb_lastUpdatePeriod = layout.add("Time Since Last Update" + cameraName, 0).getEntry();
        // sb_aprilTagSeen = layout.add(cameraName + "April Tag Read", false).getEntry();

        //Advantage Scope
        as_aprilTags = NetworkTableInstance.getDefault()
            .getStructArrayTopic(cameraName, Pose3d.struct).publish();

        as_cameraPose = NetworkTableInstance.getDefault()
            .getStructTopic(cameraName + " pose", Pose3d.struct).publish();

        as_estimatedCameraPose = NetworkTableInstance.getDefault()
            .getStructTopic(cameraName + " Estimated Pose", Pose2d.struct).publish();

        // Vision Simulation
        targetModel = TargetModel.kAprilTag16h5;
        cameraProp = new SimCameraProperties();
        cameraProp.setFPS(60);
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(70));
        cameraSim = new PhotonCameraSim(camera, cameraProp);
        cameraSim.enableDrawWireframe(true);

        // Test Values
        // TODO Delete after testing
        displayNum = -1;
        resultsList = "";
        tagPresent = false;

        aprilTagList = new Pose3d[] {};
        
        curStdDevs = VecBuilder.fill(0, 0, 0);
    }

    /**
     * Period method. Updates UI.
     */
    @Override
    public void periodic() {
        updatePose();
        updateUI();
    }

    /**
     * Updates camera pose estimation.
     * <p>
     * Submits at most one vision measurement per camera frame. The coprocessor's
     * multi-tag PnP solution is preferred, since it is constrained by every tag in
     * view; when multi-tag is unavailable the lowest-ambiguity single-tag solution is
     * used instead, gated on the ambiguity threshold from the pipeline settings.
     */
    private void updatePose() {
        aprilTagSeen = false;
        last_pose = Pose2d.kZero;
        aprilTagList = new Pose3d[] {};

        for (var result : camera.getAllUnreadResults()) {
            var multiTagEst = pose_est.estimateCoprocMultiTagPose(result);
            boolean isMultiTag = multiTagEst.isPresent();
            var visionEst = isMultiTag ? multiTagEst : pose_est.estimateLowestAmbiguityPose(result);
            if (visionEst.isEmpty()) continue;

            var est = visionEst.get();
            List<PhotonTrackedTarget> contributingTags;
            double tagDist;

            if (isMultiTag) {
                // targetsUsed is the whole frame, but the coprocessor may have dropped tags
                // from its solve. Keep only the ones it actually used, so the range below
                // reflects the geometry that produced this pose.
                var usedIds = result.getMultiTagResult().get().fiducialIDsUsed;
                contributingTags = new ArrayList<>(usedIds.size());
                for (var tag : est.targetsUsed) {
                    if (usedIds.contains((short) tag.getFiducialId())) contributingTags.add(tag);
                }
                if (contributingTags.isEmpty()) continue;

                // Every one of those tags constrains the solve, so scale the standard
                // deviations off the average range to them.
                tagDist = 0;
                for (var tag : contributingTags) {
                    tagDist += tag.getBestCameraToTarget().getTranslation().getNorm();
                }
                tagDist /= contributingTags.size();
            } else {
                // The fallback solved off a single fiducial, but targetsUsed still holds the
                // whole frame. Find the tag the estimator actually picked - the lowest
                // ambiguity one - and gate on it alone, since nothing else moved the pose.
                PhotonTrackedTarget solveTag = null;
                for (var tag : est.targetsUsed) {
                    double ambiguity = tag.getPoseAmbiguity();
                    if (ambiguity != -1 && (solveTag == null || ambiguity < solveTag.getPoseAmbiguity())) {
                        solveTag = tag;
                    }
                }
                if (solveTag == null || solveTag.getPoseAmbiguity() > ambiguity_threshold) continue;

                contributingTags = List.of(solveTag);
                tagDist = solveTag.getBestCameraToTarget().getTranslation().getNorm();
            }

            if (tagDist > maxDistance) continue;

            // Throw out solves that put the robot off the floor or outside the field.
            if (!isPoseOnField(est.estimatedPose)) continue;

            curStdDevs = (isMultiTag ? settings.multi_tag_std : settings.single_tag_std)
                    .times(1 + (tagDist * tagDist / 30));

            Pose2d est_pose = est.estimatedPose.toPose2d();
            drive.addVisionMeasurement(est_pose, est.timestampSeconds, curStdDevs);

            last_pose = est_pose;
            last_timestamp = est.timestampSeconds;
            aprilTagSeen = true;

            List<Pose3d> tagPoses = new ArrayList<>(contributingTags.size());
            for (var tag : contributingTags) {
                aprilTagField.getTagPose(tag.getFiducialId()).ifPresent(tagPoses::add);
            }
            aprilTagList = tagPoses.toArray(new Pose3d[0]);
        }
    }

    /**
     * Sanity check on a raw vision solution. A bad solvePnP result usually shows up as
     * a robot floating above the carpet or standing outside the field perimeter, and
     * either one drags the pose estimator off course, so reject it before submitting.
     *
     * @param pose Estimated field relative robot pose
     * @return True if the pose is physically plausible
     */
    private boolean isPoseOnField(Pose3d pose) {
        return Math.abs(pose.getZ()) <= kMaxPoseHeight
                && pose.getX() >= -kFieldMargin
                && pose.getX() <= aprilTagField.getFieldLength() + kFieldMargin
                && pose.getY() >= -kFieldMargin
                && pose.getY() <= aprilTagField.getFieldWidth() + kFieldMargin;
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    /**
     * Standard deviations applied to the most recent accepted measurement, as
     * [x, y, theta]. Logged so the effect of tuning them is visible on the robot.
     */
    @AutoLogOutput (key = "Camera: StdDevs {cameraName}")
    public double[] getEstimationStdDevsLog() {
        return curStdDevs.getData();
    }

    /** True when the most recent frame produced a measurement the filter accepted. */
    @AutoLogOutput (key = "Camera: TagAccepted {cameraName}")
    public boolean getAprilTagSeen() {
        return aprilTagSeen;
    }

    @AutoLogOutput (key = "Camera: CamPose {cameraName}")
    public Pose3d getRobotRelativeCamPos(){
        return new Pose3d(drive.getPose2d()).transformBy(settings.robot_to_camera);
    }

    @AutoLogOutput (key = "Camera: LastPose {cameraName}")
    public Pose3d getLastPose(){
        return new Pose3d(last_pose);
    }

    public PhotonCameraSim getCameraSim() {
        return cameraSim;
    }

    public Transform3d getOffset() {
        return settings.robot_to_camera;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

    /**
     * Updates Shuffleboard
     */
    private void updateUI() {
        // sb_PoseX.setDouble(last_pose.getX());
        // sb_PoseY.setDouble(last_pose.getY());
        // sb_PoseR.setDouble(last_pose.getRotation().getDegrees());
        // sb_lastTimestamp.setDouble(last_timestamp);
        // sb_lastUpdatePeriod.setDouble(Timer.getFPGATimestamp() - last_timestamp);
        // sb_aprilTagSeen.setBoolean(aprilTagSeen);

        //Advantage Scope
        as_aprilTags.set(aprilTagList);
        as_cameraPose.set(getRobotRelativeCamPos());
        as_estimatedCameraPose.set(last_pose);
    }
}