package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
  public static final boolean kIsComp = true;

  public static final double kLoopTime = 0.02; // Periodic loop time in seconds

  public static final String kRioCAN = "rio";
  public static final String kCANivoreCAN = "CANivore";

  public static final double kFieldLength = Units.inchesToMeters(54*12 + 3.25);
  public static final double kFieldWidth = Units.inchesToMeters(26*12 + 3.5);

  public static final class OIConstants {
    public static final double kDeadband = 0.05;

    public static final int kDriverJoy = 0;
    public static final int kOperatorJoy = 1;
    public static final int kManualJoy = 2;
    public static final int kTestJoy = 3;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeed = 4; // m/s
    public static final double kMaxAccel = 3; // m/s^2
    public static final double kMaxAngularSpeed = Math.PI; // rad/s
    public static final double kMaxAngularAccel = Math.PI; // rad/s^2

    public static final double kXControllerP = 1;
    public static final double kYControllerP = 1;
    public static final double kRotationControllerP = 1;
  }

  public static final class VisionConstants {
    public static final ArrayList<AprilTag> kAprilTags = new ArrayList<AprilTag>(List.of(
      new AprilTag(1, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters( 42.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))),
      new AprilTag(2, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))),
      new AprilTag(3, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))),
      new AprilTag(4, new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, Math.PI))),
      new AprilTag(5, new Pose3d(Units.inchesToMeters( 14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, 0.0))),
      new AprilTag(6, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0))),
      new AprilTag(7, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0))),
      new AprilTag(8, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters( 42.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0)))
    ));

    public static final ArrayList<Pair<String, Transform3d>> kCameras = new ArrayList<Pair<String, Transform3d>>(
      kIsComp ? List.of(

      ) : List.of(
      new Pair<String, Transform3d>(
        "Camera_2",
        new Transform3d(
          new Translation3d(-Units.inchesToMeters(4.75), Units.inchesToMeters(10.375), Units.inchesToMeters(10)),
          new Rotation3d(0, 0, 0)
        )
      )
    ));

    // How much to trust vision measurements normally
    public static final Matrix<N3, N1> kBaseVisionPoseStdDevs = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
      0.9, // x in meters (default=0.9)
      0.9, // y in meters (default=0.9)
      0.9 // heading in radians (default=0.9)
    );

    // Increasing this makes pose estimation trust vision measurements less as distance from Apriltags increases
    // This is how much is added to std dev for vision when closest visible Apriltag is 1 meter away
    public static final double kVisionPoseStdDevFactor = 0.1;
  }

  public static final class SwerveConstants {
    public static final int kPigeonID = kIsComp ? 0 : 13;

    /* CAN buses */
    public static final String kDriveMotorCAN = kIsComp ? kCANivoreCAN : kRioCAN;
    public static final String kAngleMotorCAN = kCANivoreCAN;
    public static final String kCANCoderCAN = kCANivoreCAN;
    public static final String kPigeonCAN = kCANivoreCAN;

    public static final boolean kInvertGyro = false; // Make sure gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants kModuleConstants =
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);   

    /* Robot dimensions */
    public static final double kTrackWidth = Units.inchesToMeters(kIsComp ? 20.75 : 22.75); // Center to Center distance of left and right modules
    public static final double kWheelBase = Units.inchesToMeters(kIsComp ? 20.75 : 22.75); // Center to Center distance of front and rear module wheels
    public static final double kWheelCircumference = kModuleConstants.wheelCircumference; 

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // Front left
      new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // Front right
      new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // Back left
      new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0) // Back right
    );

    /* Motor gear ratios */
    public static final double kDriveGearRatio = kModuleConstants.driveGearRatio;
    public static final double kAngleGearRatio = kModuleConstants.angleGearRatio;

    /* Motor inversions */
    public static final boolean kDriveMotorInvert = kModuleConstants.driveMotorInvert;
    public static final boolean kAngleMotorInvert = kModuleConstants.angleMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean kCANcoderInvert = kModuleConstants.canCoderInvert;

    /* Swerve Current Limiting */
    public static final int kAngleContinuousCurrentLimit = 25;
    public static final int kAnglePeakCurrentLimit = 40;
    public static final double kAnglePeakCurrentDuration = 0.1;
    public static final boolean kAngleEnableCurrentLimit = true;

    public static final int kDriveContinuousCurrentLimit = 35;
    public static final int kDrivePeakCurrentLimit = 60;
    public static final double kDrivePeakCurrentDuration = 0.1;
    public static final boolean kDriveEnableCurrentLimit = true;

    /* Ramp values for drive motors in open and closed loop driving */
    public static final double kOpenLoopRamp = 0.25; // A small open loop ramp (0.25) helps with tread wear, tipping, etc
    public static final double kClosedLoopRamp = 0.0;

    /* Drive Motor PID Values */
    public static final double kDriveP = 0.05;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
    public static final double kDriveF = 0.0;

    /* Drive Motor Characterization Values 
      * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double kDriveKS = (kIsComp ? 0.32 : 0) / 12.0;
    public static final double kDriveKV = (kIsComp ? 1.51 : 0) / 12.0;
    public static final double kDriveKA = (kIsComp ? 0.27 : 0) / 12.0;

    /* Angle Motor PID Values */
    public static final double kAngleP = kModuleConstants.angleKP;
    public static final double kAngleI = kModuleConstants.angleKI;
    public static final double kAngleD = kModuleConstants.angleKD;
    public static final double kAngleF = kModuleConstants.angleKF;

    /* Swerve Profiling Values */
    public static final double kMaxSpeed = (6380 / 60.0) * kModuleConstants.wheelCircumference / kModuleConstants.driveGearRatio ; // m/s
    public static final double kMaxAngularVelocity = kMaxSpeed / Math.hypot(kTrackWidth / 2, kWheelBase / 2); // rad/s, omega = V/r

    /* Neutral Modes */
    public static final NeutralMode kDriveNeutralMode = NeutralMode.Brake;
    public static final NeutralMode kAngleNeutralMode = NeutralMode.Coast;

    /* Module Specific Constants */
    /* Front Left Module */
    public static final class FL {
        public static final String kModuleName = "Front Left";
        public static final String kModuleAbbr = "FL"; // Module abbreviation
        public static final int kDriveMotorID = kIsComp ? 20 : 1;
        public static final int kAngleMotorID = kIsComp ? 15 : 2;
        public static final int kCANcoderID = kIsComp ? 40 : 3;
        public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(kIsComp ? 0.0 : 0.0);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCANcoderID, kAngleOffset);
    }

    /* Front Right Module */
    public static final class FR {
        public static final String kModuleName = "Front Right";
        public static final String kModuleAbbr = "FR"; // Module abbreviation
        public static final int kDriveMotorID = kIsComp ? 33 : 4;
        public static final int kAngleMotorID = kIsComp ? 30 : 5;
        public static final int kCANcoderID = kIsComp ? 41 : 6;
        public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(kIsComp ? 0.0 : 0.0);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCANcoderID, kAngleOffset);
    }
    
    /* Back Left Module */
    public static final class BL {
        public static final String kModuleName = "Back Left";
        public static final String kModuleAbbr = "BL"; // Module abbreviation
        public static final int kDriveMotorID = kIsComp ? 16 : 7;
        public static final int kAngleMotorID = kIsComp ? 18 : 8;
        public static final int kCANcoderID = kIsComp ? 42 : 9;
        public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(kIsComp ? 0.0 : 0.0);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCANcoderID, kAngleOffset);
    }

    /* Back Right Module */
    public static final class BR {
        public static final String kModuleName = "Back Right";
        public static final String kModuleAbbr = "BR"; // Module abbreviation
        public static final int kDriveMotorID = kIsComp ? 32 : 10;
        public static final int kAngleMotorID = kIsComp ? 35 : 11;
        public static final int kCANcoderID = kIsComp ? 43 : 12;
        public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(kIsComp ? 0.0 : 0.0);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCANcoderID, kAngleOffset);
    }
  }
}
