package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.SwerveConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;

public class SwerveModule {
  private final ShuffleboardTab m_swerveTab;

  private final int m_moduleIndex;
  private Rotation2d m_angleOffset;
  private Rotation2d m_lastAngle;

  private final String m_moduleAbbr;

  private final WPI_TalonFX m_angleMotor;
  private final WPI_TalonFX m_driveMotor;
  private final WPI_CANCoder m_CANcoder;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(SwerveConstants.kDriveKS, SwerveConstants.kDriveKV,
      SwerveConstants.kDriveKA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants, String moduleAbbr, ShuffleboardTab swerveTab) {
    m_swerveTab = swerveTab;
    m_moduleIndex = moduleNumber;
    m_moduleAbbr = moduleAbbr;
    m_angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    m_CANcoder = new WPI_CANCoder(moduleConstants.cancoderID, SwerveConstants.kCANCoderCAN);
    configCANcoder();

    /* Angle Motor Config */
    m_angleMotor = new WPI_TalonFX(moduleConstants.angleMotorID, SwerveConstants.kAngleMotorCAN);
    configAngleMotor();

    /* Drive Motor Config */
    m_driveMotor = new WPI_TalonFX(moduleConstants.driveMotorID, SwerveConstants.kDriveMotorCAN);
    configDriveMotor();

    m_lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    /*
     * This is a custom optimize function, since default WPILib optimize assumes
     * continuous controller which CTRE and Rev onboard is not
     */
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.kMaxSpeed;
      m_driveMotor.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, SwerveConstants.kWheelCircumference,
          SwerveConstants.kDriveGearRatio);
      m_driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.kMaxSpeed * 0.01)) ? m_lastAngle
        : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

    m_angleMotor.set(ControlMode.Position,
        Conversions.degreesToFalcon(angle.getDegrees(), SwerveConstants.kAngleGearRatio));
    m_lastAngle = angle;
  }

  public int getModuleIndex() {
    return m_moduleIndex;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(
        Conversions.falconToDegrees(m_angleMotor.getSelectedSensorPosition(), SwerveConstants.kAngleGearRatio));
  }

  public Rotation2d getCANcoder() {
    return Rotation2d.fromDegrees(m_CANcoder.getAbsolutePosition());
  }

  public void resetToAbsolute() {
    double absolutePosition = Conversions.degreesToFalcon(getCANcoder().getDegrees() - m_angleOffset.getDegrees(),
        SwerveConstants.kAngleGearRatio);
    m_angleMotor.setSelectedSensorPosition(absolutePosition);
  }

  private void configCANcoder() {
    m_CANcoder.configFactoryDefault();

    CANCoderConfiguration config = new CANCoderConfiguration();

    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.sensorDirection = SwerveConstants.kCANcoderInvert;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;

    m_CANcoder.configAllSettings(config);
  }

  private void configAngleMotor() {
    m_angleMotor.configFactoryDefault();

    TalonFXConfiguration config = new TalonFXConfiguration();
    SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
        SwerveConstants.kAngleEnableCurrentLimit,
        SwerveConstants.kAngleContinuousCurrentLimit,
        SwerveConstants.kAnglePeakCurrentLimit,
        SwerveConstants.kAnglePeakCurrentDuration);
    config.slot0.kP = SwerveConstants.kAngleP;
    config.slot0.kI = SwerveConstants.kAngleI;
    config.slot0.kD = SwerveConstants.kAngleD;
    config.slot0.kF = SwerveConstants.kAngleF;
    config.supplyCurrLimit = angleSupplyLimit;

    m_angleMotor.configAllSettings(config);

    m_angleMotor.setInverted(SwerveConstants.kAngleMotorInvert);
    m_angleMotor.setNeutralMode(SwerveConstants.kAngleNeutralMode);
    resetToAbsolute();
  }

  private void configDriveMotor() {
    m_driveMotor.configFactoryDefault();

    TalonFXConfiguration config = new TalonFXConfiguration();
    SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        SwerveConstants.kDriveEnableCurrentLimit,
        SwerveConstants.kDriveContinuousCurrentLimit,
        SwerveConstants.kDrivePeakCurrentLimit,
        SwerveConstants.kDrivePeakCurrentDuration);
    config.slot0.kP = SwerveConstants.kDriveP;
    config.slot0.kI = SwerveConstants.kDriveI;
    config.slot0.kD = SwerveConstants.kDriveD;
    config.slot0.kF = SwerveConstants.kDriveF;
    config.supplyCurrLimit = driveSupplyLimit;
    config.openloopRamp = SwerveConstants.kOpenLoopRamp;
    config.closedloopRamp = SwerveConstants.kClosedLoopRamp;

    m_driveMotor.configAllSettings(config);

    m_driveMotor.setInverted(SwerveConstants.kDriveMotorInvert);
    m_driveMotor.setNeutralMode(SwerveConstants.kDriveNeutralMode);
    m_driveMotor.setSelectedSensorPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), SwerveConstants.kWheelCircumference,
            SwerveConstants.kDriveGearRatio),
        getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.falconToMeters(m_driveMotor.getSelectedSensorPosition(), SwerveConstants.kWheelCircumference,
            SwerveConstants.kDriveGearRatio),
        getAngle());
  }

  public void setupShuffleboard() {
    m_swerveTab.addDouble(m_moduleAbbr + " CANcoder Angle (deg)", getCANcoder()::getDegrees);
    m_swerveTab.addDouble(m_moduleAbbr + " FX Angle (deg)", getPosition().angle::getDegrees);
    m_swerveTab.addDouble(m_moduleAbbr + " Velocity (m/s)", () -> getState().speedMetersPerSecond);
  }
}