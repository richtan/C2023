package frc.robot.util;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class Vision {
  private final ShuffleboardTab m_visionTab;

  private final ArrayList<VisionCamera> m_cameras = new ArrayList<>();
  private AprilTagFieldLayout m_aprilTagFieldLayout;

  /**
   * Sets up the cameras and pose estimator
   * @param drive The drivetrain
   * @param m_cameras The list of camera names and their translation from the center of robot
   */
  public Vision(List<Pair<String, Transform3d>> camList, ShuffleboardTab tab) {
    m_visionTab = tab;
    try {
      m_aprilTagFieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (IOException e) {
      m_aprilTagFieldLayout = new AprilTagFieldLayout(VisionConstants.kAprilTags, FieldConstants.kLength, FieldConstants.kWidth);
      DriverStation.reportWarning("Could not find k2023ChargedUp.m_resourceFile, check that GradleRIO is updated to at least 2023.2.1 in build.gradle",  e.getStackTrace());
    }
    m_aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

    for (int i = 0; i < camList.size(); i++) {
      m_cameras.add(this.new VisionCamera(camList.get(i).getFirst(), camList.get(i).getSecond(), m_visionTab));
    }

    setupShuffleboard();
  }

  public ArrayList<EstimatedRobotPose> getEstimatedPoses(Pose2d referencePose) {
    ArrayList<EstimatedRobotPose> estimatedPoses = new ArrayList<>();
    for (int i = 0; i < m_cameras.size(); i++) {
      Optional<EstimatedRobotPose> estimatedPose = m_cameras.get(i).getEstimatedPose(referencePose);
      if (estimatedPose.isPresent()) {
        estimatedPoses.add(estimatedPose.get());
      }
    }
    return estimatedPoses;
  }

  public AprilTagFieldLayout getAprilTagFieldLayout(){
    return m_aprilTagFieldLayout;
  }

  /**
   * @param id AprilTag id (1-8)
   * @return Pose3d of the AprilTag
   */
  public Pose3d getTagPose(int id){
    return getAprilTagFieldLayout().getTagPose(id).get();
  }

  private void setupShuffleboard() {

  }

  class VisionCamera {
    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_photonPoseEstimator;
    private final ShuffleboardTab m_visionTab;
    private Pair<Pose2d, Double> m_estimatedPose;
  
    public VisionCamera(String cameraName, Transform3d robotToCam, ShuffleboardTab visionTab) {
      m_camera = new PhotonCamera(cameraName);
      m_visionTab = visionTab;
      m_photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP, m_camera, robotToCam);
      m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      m_estimatedPose = new Pair<Pose2d, Double>(new Pose2d(), -1.0);

      setupShuffleboard();
    }
  
    /**
     * @param referencePose previous estimated robot pose
     * @return estimated robot pose
     */
    public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d referencePose) {
      m_photonPoseEstimator.setReferencePose(referencePose);
      Optional<EstimatedRobotPose> estimatedRobotPose = m_photonPoseEstimator.update();
      if (estimatedRobotPose.isPresent()) {
        m_estimatedPose = new Pair<Pose2d, Double>(
          estimatedRobotPose.get().estimatedPose.toPose2d(),
          Timer.getFPGATimestamp() - estimatedRobotPose.get().timestampSeconds
        );
      }
      return estimatedRobotPose;
    }

    private void setupShuffleboard() {
      m_visionTab.addDouble(m_camera.getName() + " Est. X (m)", () -> m_estimatedPose.getFirst().getTranslation().getX());
      m_visionTab.addDouble(m_camera.getName() + " Est. Y (m)", () -> m_estimatedPose.getFirst().getTranslation().getY());
      m_visionTab.addDouble(m_camera.getName() + " Est. Theta (deg)", () -> m_estimatedPose.getFirst().getRotation().getDegrees());
      m_visionTab.addDouble(m_camera.getName() + " Latency (s)", () -> m_estimatedPose.getSecond());
    }
  }
}
