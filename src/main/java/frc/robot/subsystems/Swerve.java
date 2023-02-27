package frc.robot.subsystems;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.Vision;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
  private final ShuffleboardTab m_swerveTab;

  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final SwerveModule[] m_modules;
  private final WPI_Pigeon2 m_gyro;
  private final Vision m_vision;

  private final PIDController m_xController;
  private final PIDController m_yController;
  private final PIDController m_rotationController;

  private final Field2d m_field;

  public Swerve(Vision vision, ShuffleboardTab swerveTab) {
    m_vision = vision;
    m_swerveTab = swerveTab;
    m_gyro = new WPI_Pigeon2(SwerveConstants.kPigeonID, SwerveConstants.kPigeonCAN);
    m_gyro.configFactoryDefault();
    zeroGyro();

    m_modules = new SwerveModule[] {
        new SwerveModule(0, SwerveConstants.FL.constants, SwerveConstants.FL.kModuleAbbr, m_swerveTab),
        new SwerveModule(1, SwerveConstants.FR.constants, SwerveConstants.FR.kModuleAbbr, m_swerveTab),
        new SwerveModule(2, SwerveConstants.BL.constants, SwerveConstants.BL.kModuleAbbr, m_swerveTab),
        new SwerveModule(3, SwerveConstants.BR.constants, SwerveConstants.BR.kModuleAbbr, m_swerveTab)
    };

    /*
     * By pausing init for a second before setting module offsets, we avoid a bug
     * with inverting motors.
     * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetModulesToAbsolute();

    m_poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.kKinematics, getYaw(), getModulePositions(), new Pose2d());
    m_poseEstimator.setVisionMeasurementStdDevs(VisionConstants.kBaseVisionPoseStdDevs);

    // Setup PID controllers for auto
    m_xController = new PIDController(AutoConstants.kXControllerP, 0, 0);
    m_yController = new PIDController(AutoConstants.kYControllerP, 0, 0);
    m_rotationController = new PIDController(AutoConstants.kRotationControllerP, 0, 0);
    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);

    m_field = new Field2d();
    m_field.setRobotPose(getPose());

    setupShuffleboard();
  }

  // PID controllers for PathPlanner auto
  public PIDController getXController() { return m_xController; }
  public PIDController getYController() { return m_yController; }
  public PIDController getRotationController() { return m_rotationController; }

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveConstants.kKinematics.toChassisSpeeds(getModuleStates());
  }

  public SwerveModule[] getModules() {
    return m_modules;
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = SwerveConstants.kKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            getYaw())
            : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kMaxSpeed);

    for (SwerveModule mod : m_modules) {
      mod.setDesiredState(swerveModuleStates[mod.getModuleIndex()], isOpenLoop);
    }
  }

  /**
   * Stop driving
   */
  public void stop() {
    drive(new Translation2d(0, 0), 0, true, true);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeed);

    for (SwerveModule mod : m_modules) {
      mod.setDesiredState(desiredStates[mod.getModuleIndex()], false);
    }
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void toggleAngleJitterPrevention(boolean enabled) {
    for (SwerveModule mod : m_modules) {
      mod.toggleAngleJitterPrevention(enabled);
    }
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : m_modules) {
      states[mod.getModuleIndex()] = mod.getState();
    }
    return states;
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = SwerveConstants.kKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kMaxSpeed);
    setModuleStates(swerveModuleStates);
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : m_modules) {
      positions[mod.getModuleIndex()] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    m_gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return (SwerveConstants.kInvertGyro) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw())
        : Rotation2d.fromDegrees(m_gyro.getYaw());
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : m_modules) {
      mod.resetToAbsolute();
    }
  }

  public void updateOdometry() {
    m_poseEstimator.update(getYaw(), getModulePositions());

    // Correct pose based on vision measurements
    // Trust vision more if robot is closer to Apriltags and less if it is farther
    ArrayList<EstimatedRobotPose> estimatedPoses = m_vision.getEstimatedPoses(m_poseEstimator.getEstimatedPosition());
    Translation2d currentEstimatedPoseTranslation = m_poseEstimator.getEstimatedPosition().getTranslation();
    for (int i = 0; i < estimatedPoses.size(); i++) {
      EstimatedRobotPose estimatedPose = estimatedPoses.get(i);
      Translation2d closestTagPoseTranslation = new Translation2d();
      for (int j = 0; j < estimatedPose.targetsUsed.size(); j++) {
        Translation2d currentTagPoseTranslation = m_vision.getTagPose(estimatedPose.targetsUsed.get(j).getFiducialId()).toPose2d().getTranslation();
        if (j == 0 || currentEstimatedPoseTranslation.getDistance(currentTagPoseTranslation) < currentEstimatedPoseTranslation.getDistance(closestTagPoseTranslation)) {
          closestTagPoseTranslation = currentTagPoseTranslation;
        }
      }
      m_poseEstimator.addVisionMeasurement(
        estimatedPose.estimatedPose.toPose2d(),
        estimatedPose.timestampSeconds,
        VisionConstants.kBaseVisionPoseStdDevs.plus(
          currentEstimatedPoseTranslation.getDistance(closestTagPoseTranslation) * VisionConstants.kVisionPoseStdDevFactor
        )
      );
    }

    m_field.setRobotPose(getPose());
  }

  private void setupShuffleboard() {
    m_swerveTab.addDouble("Yaw (deg)", () -> getYaw().getDegrees());
    m_swerveTab.add("Field", m_field);
  }

  @Override
  public void periodic() {
    updateOdometry();
  }
}