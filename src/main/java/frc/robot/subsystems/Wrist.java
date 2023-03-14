package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
  private final ShuffleboardTab m_wristTab;

  private final WPI_TalonFX m_motor;
  private final DutyCycleEncoder m_absEncoder;

  private WristMode m_mode;
  private double m_desiredAngle;
  private double m_desiredPower;

  public Wrist(ShuffleboardTab wristTab) {
    m_wristTab = wristTab;

    m_motor = new WPI_TalonFX(WristConstants.kMotorID, WristConstants.kWristCAN);
    configWristMotor();

    m_absEncoder = new DutyCycleEncoder(WristConstants.kAbsEncoderID);
    Timer.delay(1);
    calibrateEncoder();

    m_mode = WristMode.DISABLED;
    m_desiredAngle = WristConstants.kStowAngle;
    m_desiredPower = 0.0;

    setupShuffleboard();
  }

  private void configWristMotor() {
    m_motor.configFactoryDefault();

    m_motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      WristConstants.kEnableCurrentLimit,
      WristConstants.kContinuousCurrentLimit,
      WristConstants.kPeakCurrentLimit,
      WristConstants.kPeakCurrentDuration
    ));


    m_motor.config_kP(0, WristConstants.kP);
    m_motor.config_kI(0, WristConstants.kI);
    m_motor.config_kD(0, WristConstants.kD);
    m_motor.config_kF(0, WristConstants.kF);

    m_motor.setInverted(WristConstants.kMotorInvert);
    m_motor.setNeutralMode(WristConstants.kNeutralMode);

    m_motor.configVoltageCompSaturation(Constants.kNormalOperatingVoltage);
    m_motor.enableVoltageCompensation(true);

    m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_motor.configForwardSoftLimitEnable(true);
    m_motor.configReverseSoftLimitEnable(true);
    m_motor.configForwardSoftLimitThreshold(WristConstants.kStowAngle);
    m_motor.configReverseSoftLimitThreshold(WristConstants.kMaxDeployAngle);
  }

  /**
   * Get absolute encoder position in degrees
   * @return
   */
  private double getAbsEncoder() {
    return m_absEncoder.getAbsolutePosition() * WristConstants.kAbsEncoderDistancePerRotation;
  }

  public void calibrateEncoder() {
    double absEncoderError = getAbsEncoder() - WristConstants.kAbsEncoderZeroAngle;
    m_motor.setSelectedSensorPosition(absEncoderError / 360 * 2048);
  }

  public void zeroEncoder() {
    m_motor.setSelectedSensorPosition(0);
  }

  public enum WristMode {
    DISABLED, MANUAL, POSITION
  }

  public void setMode(WristMode mode) {
    m_mode = mode;
  }

  public WristMode getMode() {
    return m_mode;
  }

  public void setNeutralMode(NeutralMode neutralMode) {
    m_motor.setNeutralMode(neutralMode);
  }

  public void setDesiredAngle(double desiredAngle) {
    m_desiredAngle = desiredAngle;
  }
  
  public void setDesiredPower(double desiredPower) {
    m_desiredPower = desiredPower;
  }

  public double getAngle() {
    return Conversions.falconToDegrees(m_motor.getSelectedSensorPosition(), WristConstants.kMotorToAbsEncoderGearRatio);
  }

  public double getVelocity() {
    return Units.radiansToDegrees(Units.rotationsPerMinuteToRadiansPerSecond(Conversions.falconToRPM(m_motor.getSelectedSensorVelocity(), WristConstants.kMotorToAbsEncoderGearRatio)));
  }

  public boolean reachedDesiredAngle() {
    return Math.abs(m_desiredAngle - getAngle()) < WristConstants.kAngleTolerance
          && Math.abs(getVelocity()) < WristConstants.kVelocityTolerance;
  }

  @Override
  public void periodic() {
    switch (m_mode) {
      case DISABLED:
        m_motor.stopMotor();
        break;
      case MANUAL:
        m_motor.set(ControlMode.PercentOutput, m_desiredPower);
        break;
      case POSITION:
        m_motor.set(
          ControlMode.Position,
          Conversions.degreesToFalcon(m_desiredAngle, WristConstants.kMotorToAbsEncoderGearRatio),
          DemandType.ArbitraryFeedForward,
          WristConstants.kGravityCompensationFactor * Math.cos(Units.degreesToRadians(getAngle()))
        );
    }
  }

  private void setupShuffleboard() {
    if (Constants.kUseTelemetry) {
      m_wristTab.addDouble("Current angle (deg)", this::getAngle);
      m_wristTab.addDouble("Desired angle (deg)", () -> m_desiredAngle);
      m_wristTab.addDouble("Desired Power", () -> m_desiredPower);
      m_wristTab.addBoolean("Reached Desired Angle", this::reachedDesiredAngle);
      m_wristTab.addDouble("Absolute encoder (deg)", this::getAbsEncoder);
      m_wristTab.addString("Mode", () -> m_mode.toString());
    }
  }
}
