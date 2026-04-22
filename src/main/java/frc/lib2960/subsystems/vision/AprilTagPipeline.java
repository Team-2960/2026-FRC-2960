package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
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
import edu.wpi.first.math.geometry.Translation3d;
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
        //updatePoseTest();
        updateUI();
    }

    /**
     * Updates camera pose estimation
     */
    private void updatePose() {
        var unreadResults = camera.getAllUnreadResults();
        aprilTagSeen = false;
        
        for (var change : unreadResults) {
            List<PhotonTrackedTarget> visionEst = change.getTargets();
            int iteration = 0;
            last_pose = Pose2d.kZero;
            // Check if a pose was estimated
            if (!visionEst.isEmpty()) {
                double est_timestamp = change.getTimestampSeconds();
                aprilTagList = new Pose3d[change.getTargets().size()];
                // Get found tag count and average distance
                for (var tag : visionEst) {
                    var tag_pose3d = PhotonUtils.estimateFieldToRobotAprilTag(
                        tag.bestCameraToTarget, 
                        pose_est.getFieldTags().getTagPose(tag.fiducialId).get(),
                        settings.robot_to_camera.inverse());

                        if (tag.getPoseAmbiguity() <= ambiguity_threshold) {
                            Translation3d tagDistances = tag.getBestCameraToTarget().getTranslation();
                            double dist = 
                                Math.sqrt(Math.pow(tagDistances.getX(), 2) + Math.pow(tagDistances.getY(), 2) + Math.pow(tagDistances.getZ(), 2));
                            if (dist <= maxDistance) {
                                var est_std = settings.single_tag_std;
                                est_std = est_std.times(1 + (dist * dist / 30));

                                drive.addVisionMeasurement(tag_pose3d.toPose2d(), est_timestamp, est_std);
                                last_pose = tag_pose3d.toPose2d();

                                aprilTagList[iteration] = AprilTagFieldLayout.loadField(field).getTagPose(tag.getFiducialId()).get();
                                iteration++;
                                aprilTagSeen = true;
                            }
                        }

                }

                if (iteration == 0) aprilTagList = new Pose3d[] {};

            }

        }
    }

    private void updatePoseTest(){
        last_pose = Pose2d.kZero;
        aprilTagList = new Pose3d[0];
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var result : camera.getAllUnreadResults()) {
            visionEst = pose_est.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = pose_est.estimateLowestAmbiguityPose(result);
            }
        // updateEstimationStdDevs(visionEst, result.getTargets());

        var targets = result.getTargets();
        double averageDist = 0;

        if (visionEst.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = settings.single_tag_std;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = settings.single_tag_std;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = pose_est.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(visionEst.get().estimatedPose.toPose2d().getTranslation());
            }

            averageDist = avgDist;

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = settings.single_tag_std;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = settings.multi_tag_std;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }

            if (visionEst.isPresent()){
                if (averageDist >= settings.max_dist){
                // Change our trust in the measurement based on the tags we can see
                    var estStdDevs = getEstimationStdDevs();
                    var est = visionEst.get();
                    last_pose = est.estimatedPose.toPose2d();
                    aprilTagList = new Pose3d[est.targetsUsed.size()];
                    for (int i = 0; i < est.targetsUsed.size(); i++){
                        aprilTagList[i] = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField).getTagPose(est.targetsUsed.get(i).getFiducialId()).get();
                    }

                    drive.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                }
                    
            }
            
        }
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = settings.single_tag_std;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = settings.single_tag_std;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = pose_est.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = settings.single_tag_std;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = settings.multi_tag_std;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
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
