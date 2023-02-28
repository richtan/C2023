package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.revrobotics.CANSparkMax.IdleMode;

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
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.lib.math.Conversions;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import com.revrobotics.Rev2mDistanceSensor.Port;

public final class Constants {
  public static final boolean kIsComp = true;

  public static final double kLoopTime = 0.02; // Periodic loop time in seconds

  public static final String kRioCAN = "rio";
  public static final String kCANivoreCAN = "CANivore";
  
  public static final double kNormalOperatingVoltage = 12.0;

  public static final class FieldConstants {
    public static final double kLength = Units.inchesToMeters(54*12 + 3.25);
    public static final double kWidth = Units.inchesToMeters(26*12 + 3.5);
    public static final double kBumperToShelf = Units.inchesToMeters(29.6); // From bumpers to apriltag next to shelf
  }

  public static final class OIConstants {
    public static final double kDriverDeadband = 0.00;

    public static final int kDriverJoy = 0;
    public static final int kOperatorJoy = 1;
    public static final int kManualJoy = 2;
    public static final int kTestJoy = 3;
  }

  public static final class RobotConstants {
    public static final double kBumperThickness = Units.inchesToMeters(3);
    public static final double kBumperGap = Units.inchesToMeters(0.25);
  }

  public static final class AutoConstants {
    public static final double kMaxSpeed = 4.0; // m/s
    public static final double kMaxAccel = 3.0; // m/s^2
    public static final double kMaxAngularSpeed = 2 * Math.PI; // rad/s
    public static final double kMaxAngularAccel = 4 * Math.PI; // rad/s^2

    public static final double kTranslationControllerP = 0.8; // FIXME: Auto
    public static final double kRotationControllerP = 3; // FIXME: Auto

    public static final double kBalancedAngle = 0;
    public static final double kBalanceKP = 0.007;
  }

  // Units are degrees, zero is max deploy position, positive is towards the robot,
  // angle measurement is angle of polycarb plates on intake from horizontal
  public static final class ArmConstants {
    public static final int kMotorID = 5; // sm6
    public static final IdleMode kMotorIdleMode = IdleMode.kBrake;
    public static final boolean kMotorInvert = false; // FIXME: Arm
    public static final double kMotorToAbsEncoderGearRatio = (5.0 / 1.0) * (3.0 / 1.0) * (3.0 / 1.0);
    public static final double kAbsEncoderToEndEffectorGearRatio = (48.0 / 32.0);
    // public static final double kMotorEncoderDistancePerRotation = 360.0 / kMotorToAbsEncoderGearRatio / kAbsEncoderToEndEffectorGearRatio;
    public static final double kMotorEncoderDistancePerRotation = 3.55300894;


    public static final int kAbsEncoderID = 7;
    public static final double kAbsEncoderZeroAngle = 60.33 - 22.33;
    public static final boolean kAbsEncoderInvert = false;
    // public static final double kAbsEncoderDistancePerRotation = 360.0 * (kAbsEncoderInvert ? -1 : 1) / kAbsEncoderToEndEffectorGearRatio;
    public static final double kAbsEncoderDistancePerRotation = 126.502736;
    // public static final double kAbsEncoderDistancePerRotation = 1;

    public static final double kGravityCompensationFactor = 1.5; // volts

    public static final double kAngleTolerance = 1;
    public static final double kVelocityTolerance = 0.5;

    public static final double kStowAngle = 55; // Might need to use 45 or 50 since IRL assembly is different from CAD
    public static final double kDeployAngle = 0;
    public static final double kIntakeConeAngle = -6;
    public static final double kIntakeCubeAngle = 0;
    public static final double kTopConeAngle = 18;
    public static final double kTopCubeAngle = 0;
    public static final double kMiddleConeAngle = 28;
    public static final double kMiddleCubeAngle = 25;
    public static final double kBottomConeAngle = 0;
    public static final double kBottomCubeAngle = 0;
    public static final double kShelfConeAngle = 0;
    public static final double kShelfCubeAngle = 0;
    public static final double kHalfStowAngle = 20;

    public static final double kP = 0.01271428571; // FIXME: Arm
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kMaxAngularVelocity = 20; // FIXME: Arm
    public static final double kMaxAngularAccel = 15; // FIXME: Arm
  }

  public static final class BarConstants {
    public static final int kMotorID = -1; // FIXME
    public static final boolean kMotorInvert = false;
    public static final IdleMode kMotorIdleMode = IdleMode.kBrake;
    public static final double kGearRatio = (25.0 / 1.0) * (32.0 / 18.0); // 44.44:1

    public static final double kMotorEncoderDistancePerRotation = -1; // FIXME

    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kGravityCompensationFactor = 0; // volts

    public static final int kMotorCurrentLimit = 12;

    public static final double kAngleTolerance = 0.5;
    public static final double kVelocityTolerance = 0.5;

    public static final double kStowAngle = 90;
    public static final double kDeployAngle = 0;

    public static final double kMaxAngularVelocity = 20;
    public static final double kMaxAngularAccel = 15;
  }

  // Positive is outtaking direction
  public static final class IntakeConstants {
    public static final int kLeftMotorID = 6; // sm7
    public static final int kRightMotorID = 11; // sm5
    public static final boolean kLeftMotorInvert = false;
    public static final boolean kRightMotorInvert = true;
    public static final IdleMode kLeftMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kRightMotorIdleMode = IdleMode.kBrake;

    public static final int kMotorCurrentLimit = 10;

    public static final double kGearRatio = (3.0 / 1.0); // 3:1

    // TODO: separate cube and cone intaking power
    public static final double kIntakePower = -0.7;
    public static final double kShootingPower = 0.6;
    public static final double kOuttakePower = 0.12;
    public static final double kEjectPower = 0.4;
    public static final double kDroppingPower = 0.2;

    public static final Port kDistanceSensorPort = Port.kOnboard;
    public static final double kMaxConeRange = 1.5;
    public static final double kMaxCubeRange = 7;
    public static final double kCubeTimeThreshold = 0.2; // 0.2 seconds of an intaked cube counts as intaked
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

    // NWU: +X = forward, +Y = left, +Z = up; +roll = x-axis CCW, +pitch = y-axis CCW, +yaw = z-axis CCW; use right hand rule
    public static final ArrayList<Pair<String, Transform3d>> kCameras = new ArrayList<Pair<String, Transform3d>>(
      Constants.kIsComp ? List.of(
        // new Pair<String, Transform3d>(
        //   "Left_Camera",
        //   new Transform3d(
        //     new Translation3d(-Units.inchesToMeters(1.623424), Units.inchesToMeters(7.314853), Units.inchesToMeters(20.97428)),
        //     new Rotation3d(0, 0, 0)
        //   )
        // ),
        // new Pair<String, Transform3d>(
        //   "Right_Camera",
        //   new Transform3d(
        //     new Translation3d(-Units.inchesToMeters(1.623424), Units.inchesToMeters(-7.314853), Units.inchesToMeters(20.97428)),
        //     new Rotation3d(0, 0, 0)
        //   )
        // )
      ) : List.of(

    ));

    // How much to trust vision measurements normally
    public static final Matrix<N3, N1> kBaseVisionPoseStdDevs = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
      0.9, // x in meters (default=0.9)
      0.9, // y in meters (default=0.9)
      0.9 // heading in radians (default=0.9)
    );

    // Increasing this makes pose estimation trust vision measurements less as distance from Apriltags increases
    // This is how much is added to std dev for vision when closest visible Apriltag is 1 meter away
    public static final double kVisionPoseStdDevFactor = 0;
  }

  public static final class PowerConstants {
    public static final int kPDModuleID = 1;
    public static final ModuleType kPDModuleType = ModuleType.kRev;
  }

  // Units are meters, zero is bottom, positive is upwards,
  // position measured along elevator axis from bottom carriage hardstop within first stage to bottom of carriage
  public static final class ElevatorConstants {
    public static final int kMotorID = 13;
    public static final String kElevatorCAN = kCANivoreCAN;
    public static final TalonFXInvertType kMotorInvert = TalonFXInvertType.Clockwise; // CW goes up
    public static final NeutralMode kNeutralMode = NeutralMode.Brake;

    public static final double kGearRatio = (50.0 / 12.0) * (50.0 / 30.0) * (36.0 / 24.0); // 10.416:1
    public static final double kSpoolDiameter = Units.inchesToMeters(1.375);
    public static final double kStringThickness = Units.inchesToMeters(0.125);
    public static final double kSpoolCircumference = (kSpoolDiameter + kStringThickness) * Math.PI;

    public static final int kBottomLimitSwitchPort = 8;
    public static final int kTopLimitSwitchPort = 9;

    public static final double kPositionTolerance = 0.01;
    public static final double kVelocityTolerance = 0.05; // FIXME: Elevator
    
    // Whether limit switch is normally-closed (activated = open circuit) or normally-open (activated = closed circuit)
    public static final boolean kTopLimitSwitchNC = true;
    public static final boolean kBottomLimitSwitchNC = true;

    // Slot 0
    public static final double kBottomP = 0.05; // FIXME: Elevator
    public static final double kBottomI = 0;
    public static final double kBottomD = 0;
    public static final double kBottomF = 0;
    public static final double kBottomGravityCompensation = 0;

    // Slot 1
    public static final double kBottomWithConeP = kBottomP;
    public static final double kBottomWithConeI = kBottomI;
    public static final double kBottomWithConeD = kBottomD;
    public static final double kBottomWithConeF = kBottomF;
    public static final double kBottomWithConeGravityCompensation = kBottomGravityCompensation;

    // Slot 2
    // public static final double kTopP = 0.1;
    // public static final double kTopI = 0;
    // public static final double kTopD = 0;
    // public static final double kTopF = 0;
    // public static final double kTopGravityCompensation = 0;
    public static final double kTopP = 0.05;
    public static final double kTopI = kBottomI;
    public static final double kTopD = kBottomD;
    public static final double kTopF = kBottomF;
    public static final double kTopGravityCompensation = 0.2;

    // Slot 3
    public static final double kTopWithConeP = kTopP;
    public static final double kTopWithConeI = kTopI;
    public static final double kTopWithConeD = kTopD;
    public static final double kTopWithConeF = kTopF;
    public static final double kTopWithConeGravityCompensation = kTopGravityCompensation;

    public static final int kContinuousCurrentLimit = 30; // FIXME: Elevator
    public static final int kPeakCurrentLimit = 50;
    public static final double kPeakCurrentDuration = 0.1;
    public static final boolean kEnableCurrentLimit = false;

    // Max distance that the carriage can travel within the first stage
    public static final double kCarriageMaxDistance = Units.inchesToMeters(25 - 0.25); // The 0.25 inches is the bottom hardstop
    // Max distance that the first stage can travel within the base stage
    public static final double kFirstStageMaxDistance = Units.inchesToMeters(26);
    // Total max travel distance of elevator (how far it can extend)
    public static final double kMaxPosition = kCarriageMaxDistance + kFirstStageMaxDistance;
    // Max height of elevator
    public static final double kMaxHeight = Conversions.ElevatorLengthToHeight(kMaxPosition);
    // Vertical height of the center of the top surface of the tread hardstop for the carriage when elevator is at minimum height
    public static final double kElevatorBaseHeight = Units.inchesToMeters(12.974338);
    // Angle of elevator from the horizontal axis
    public static final double kElevatorAngle = 55.0;

    public static final double kIntakeConeHeight = Conversions.ElevatorLengthToHeight(Units.inchesToMeters(0));
    public static final double kIntakeCubeHeight = Conversions.ElevatorLengthToHeight(Units.inchesToMeters(3));
    public static final double kTopConeHeight = Conversions.ElevatorLengthToHeight(kMaxPosition);
    public static final double kTopCubeHeight = Units.inchesToMeters(46.15);
    public static final double kMiddleConeHeight = Units.inchesToMeters(35);
    public static final double kMiddleCubeHeight = Units.inchesToMeters(24.2);
    public static final double kBottomConeHeight = Conversions.ElevatorLengthToHeight(Units.inchesToMeters(3));
    public static final double kBottomCubeHeight = Conversions.ElevatorLengthToHeight(Units.inchesToMeters(3));
    public static final double kShelfConeHeight = Units.inchesToMeters(49.1);
    public static final double kShelfCubeHeight = Units.inchesToMeters(52.1);
    public static final double kStowHeight = Conversions.ElevatorLengthToHeight(Units.inchesToMeters(0));


    public static final double kCalibrationPower = -0.2;
    public static final double kMotorRamp = 0.2;
  }

  // linear units are meters, angular units are radians
  public static final class SwerveConstants {
    public static final int kPigeonID = Constants.kIsComp ? 0 : 13;

    /* CAN buses */
    public static final String kDriveMotorCAN = Constants.kIsComp ? kCANivoreCAN : kRioCAN;
    public static final String kAngleMotorCAN = kCANivoreCAN;
    public static final String kCANCoderCAN = kCANivoreCAN;
    public static final String kPigeonCAN = kCANivoreCAN;

    public static final boolean kInvertGyro = false; // Make sure gyro is CCW+ CW- // FIXME: Swerve

    public static final double kSlowDriveFactor = 0.3;

    public static final COTSFalconSwerveConstants kModuleConstants =
        COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

    /* Robot dimensions */
    public static final double kTrackWidth = Units.inchesToMeters(Constants.kIsComp ? 20.75 : 22.75); // Center to Center distance of left and right modules
    public static final double kWheelBase = Units.inchesToMeters(Constants.kIsComp ? 20.75 : 22.75); // Center to Center distance of front and rear module wheels
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
    public static final double kOpenLoopRamp = 0.1; // A small open loop ramp (0.25) helps with tread wear, tipping, etc
    public static final double kClosedLoopRamp = 0.0;

    /* Drive Motor PID Values */
    public static final double kDriveP = 0.05;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
    public static final double kDriveF = 0.0;

    /* Drive Motor Characterization Values 
      * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
    public static final double kDriveKS = (Constants.kIsComp ? 0.32 : 0.65559) / 12.0;
    public static final double kDriveKV = (Constants.kIsComp ? 1.51 : 1.93074) / 12.0;
    public static final double kDriveKA = (Constants.kIsComp ? 0.27 : 0.00214) / 12.0;

    /* Angle Motor PID Values */
    public static final double kAngleP = kModuleConstants.angleKP;
    public static final double kAngleI = kModuleConstants.angleKI;
    public static final double kAngleD = kModuleConstants.angleKD;
    public static final double kAngleF = kModuleConstants.angleKF;

    /* Swerve Profiling Values */
    public static final double kMaxSpeed = (6380 / 60.0) * kModuleConstants.wheelCircumference / kModuleConstants.driveGearRatio; // m/s
    public static final double kMaxAngularVelocity = 0.5 * kMaxSpeed / Math.hypot(kTrackWidth / 2, kWheelBase / 2); // rad/s, omega = V/r

    /* Neutral Modes */
    public static final NeutralMode kDriveNeutralMode = NeutralMode.Brake;
    public static final NeutralMode kAngleNeutralMode = NeutralMode.Coast;

    /* Module Specific Constants */
    /* Front Left Module */
    public static final class FL {
        public static final String kModuleName = "Front Left";
        public static final String kModuleAbbr = "FL"; // Module abbreviation
        public static final int kDriveMotorID = Constants.kIsComp ? 20 : 1;
        public static final int kAngleMotorID = Constants.kIsComp ? 15 : 2;
        public static final int kCANcoderID = Constants.kIsComp ? 40 : 3;
        public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(Constants.kIsComp ? 0.26367 : 268.9);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCANcoderID, kAngleOffset);
    }

    /* Front Right Module */
    public static final class FR {
        public static final String kModuleName = "Front Right";
        public static final String kModuleAbbr = "FR"; // Module abbreviation
        public static final int kDriveMotorID = Constants.kIsComp ? 33 : 4;
        public static final int kAngleMotorID = Constants.kIsComp ? 30 : 5;
        public static final int kCANcoderID = Constants.kIsComp ? 41 : 6;
        public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(Constants.kIsComp ? 190.107 : 110.3);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCANcoderID, kAngleOffset);
    }
    
    /* Back Left Module */
    public static final class BL {
        public static final String kModuleName = "Back Left";
        public static final String kModuleAbbr = "BL"; // Module abbreviation
        public static final int kDriveMotorID = Constants.kIsComp ? 16 : 7;
        public static final int kAngleMotorID = Constants.kIsComp ? 18 : 8;
        public static final int kCANcoderID = Constants.kIsComp ? 42 : 9;
        public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(Constants.kIsComp ? 219.46289 : 179.85);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCANcoderID, kAngleOffset);
    }

    /* Back Right Module */
    public static final class BR {
        public static final String kModuleName = "Back Right";
        public static final String kModuleAbbr = "BR"; // Module abbreviation
        public static final int kDriveMotorID = Constants.kIsComp ? 32 : 10;
        public static final int kAngleMotorID = Constants.kIsComp ? 35 : 11;
        public static final int kCANcoderID = Constants.kIsComp ? 43 : 12;
        public static final Rotation2d kAngleOffset = Rotation2d.fromDegrees(Constants.kIsComp ? 150.732 : 338.03);
        public static final SwerveModuleConstants constants = 
            new SwerveModuleConstants(kDriveMotorID, kAngleMotorID, kCANcoderID, kAngleOffset);
    }
  }
}
