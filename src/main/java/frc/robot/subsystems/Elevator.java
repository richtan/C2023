package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final ShuffleboardTab m_elevatorTab;

  private final BooleanSupplier m_hasConeSupplier;
  private ElevatorMode m_mode;

  private final WPI_TalonFX m_motor;
  private final DigitalInput m_bottomLimitSwitch;
  private final DigitalInput m_topLimitSwitch;
  private double m_desiredHeight = 0;
  private double m_desiredPower = 0;

  public Elevator(ShuffleboardTab elevatorTab, BooleanSupplier hasConeSupplier) {
    m_elevatorTab = elevatorTab;

    m_hasConeSupplier = hasConeSupplier;
    m_mode = ElevatorMode.DISABLED;

    m_motor = new WPI_TalonFX(ElevatorConstants.kMotorID, ElevatorConstants.kElevatorCAN);
    configElevatorMotor();
    m_motor.setNeutralMode(NeutralMode.Brake);

    m_bottomLimitSwitch = new DigitalInput(ElevatorConstants.kBottomLimitSwitchPort);
    m_topLimitSwitch = new DigitalInput(ElevatorConstants.kTopLimitSwitchPort);
  }

  private void configElevatorMotor() {
    m_motor.configFactoryDefault();
    m_motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      ElevatorConstants.kEnableCurrentLimit,
      ElevatorConstants.kContinuousCurrentLimit,
      ElevatorConstants.kPeakCurrentLimit,
      ElevatorConstants.kPeakCurrentDuration
    ));

    m_motor.config_kP(0, ElevatorConstants.kBottomP);
    m_motor.config_kI(0, ElevatorConstants.kBottomI);
    m_motor.config_kD(0, ElevatorConstants.kBottomD);
    m_motor.config_kF(0, ElevatorConstants.kBottomF);

    m_motor.config_kP(1, ElevatorConstants.kBottomWithConeP);
    m_motor.config_kI(1, ElevatorConstants.kBottomWithConeI);
    m_motor.config_kD(1, ElevatorConstants.kBottomWithConeD);
    m_motor.config_kF(1, ElevatorConstants.kBottomWithConeF);

    m_motor.config_kP(2, ElevatorConstants.kTopP);
    m_motor.config_kI(2, ElevatorConstants.kTopI);
    m_motor.config_kD(2, ElevatorConstants.kTopD);
    m_motor.config_kF(2, ElevatorConstants.kTopF);

    m_motor.config_kP(3, ElevatorConstants.kTopWithConeP);
    m_motor.config_kI(3, ElevatorConstants.kTopWithConeI);
    m_motor.config_kD(3, ElevatorConstants.kTopWithConeD);
    m_motor.config_kF(3, ElevatorConstants.kTopWithConeF);

    m_motor.setInverted(ElevatorConstants.kMotorInvert);
    m_motor.setNeutralMode(ElevatorConstants.kNeutralMode);

    m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_motor.configForwardSoftLimitThreshold(
      Conversions.MetersToFalcon(heightToLength(ElevatorConstants.kMaxTravelDistance),
      ElevatorConstants.kSpoolCircumference,
      ElevatorConstants.kGearRatio
    ));
    m_motor.configReverseSoftLimitThreshold(0);

    toggleSoftLimits(false);
  }

  public void toggleSoftLimits(boolean enabled) {
    m_motor.configForwardSoftLimitEnable(enabled);
    m_motor.configReverseSoftLimitEnable(enabled);
  }

  /**
   * Checks if top limit switch is reached
   * @return true or false
   */
  public boolean isTopLimitSwitchReached() {
    return m_topLimitSwitch.get() != ElevatorConstants.kTopLimitSwitchNC;
  }

  /**
   * Checks if bottom limit switch is reached
   * @return true or false
   */
  public boolean isBottomLimitSwitchReached() {
    return m_bottomLimitSwitch.get() != ElevatorConstants.kBottomLimitSwitchNC;
  }

  /**
   * Calculate how far up along elevator length from bottom of elevator based on vertical height from ground
   * @param height in meters
   * @return length from elevator base to height in meters
   */
  public double heightToLength(double height) {
    return (height - ElevatorConstants.kElevatorBaseHeight) / Math.sin(ElevatorConstants.kElevatorAngle);
  }

  /**
   * Calculate how far up along elevator length from bottom of elevator based on vertical height from ground
   * @param length in meters
   * @return length from elevator base to height in meters
   */
  public double lengthToHeight(double length) {
    return (length * Math.sin(ElevatorConstants.kElevatorAngle)) + ElevatorConstants.kElevatorBaseHeight;
  }

  /**
   * Get position of carriage along length of elevator
   * @return position (m)
   */
  public double getPosition() {
    return heightToLength(getHeight());
  }

  /**
   * Get height of carriage above ground
   * @return height (m)
   */
  public double getHeight() {
    return Conversions.falconToMeters(m_motor.getSelectedSensorPosition(), ElevatorConstants.kSpoolCircumference, ElevatorConstants.kGearRatio);
  }

  public enum ElevatorMode {
    CALIBRATION, MANUAL, MOTIONMAGIC, DISABLED
  }

  public void setDesiredHeight(double desiredHeight) {
    m_desiredHeight = desiredHeight;
  }

  public void setDesiredPower(double power) {
    m_desiredPower = power;
  }

  public void zeroEncoder() {
    m_motor.setSelectedSensorPosition(0.0);
  }

  public void setMode(ElevatorMode mode) {
    m_mode = mode;
  }

  public boolean reachedDesiredHeight() {
    return Math.abs(m_desiredHeight - getHeight()) < ElevatorConstants.kPositionTolerance
          && Math.abs(m_motor.getSelectedSensorVelocity()) < ElevatorConstants.kVelocityTolerance;
  }

  public void updateClosedLoopSlot() {
    boolean hasCone = m_hasConeSupplier.getAsBoolean();
    if (getPosition() < ElevatorConstants.kFirstStageMaxTravelDistance) {
      m_motor.selectProfileSlot(hasCone ? 1 : 0, 0);
    } else {
      m_motor.selectProfileSlot(hasCone ? 3 : 2, 0);
    }
  }

  @Override
  public void periodic() {
    // Make sure we aren't going past the limit switches
    if (isBottomLimitSwitchReached() || (isTopLimitSwitchReached() && m_mode != ElevatorMode.CALIBRATION)) {
      m_motor.neutralOutput();
      return;
    }

    switch (m_mode) {
      case CALIBRATION:
        m_motor.set(ControlMode.PercentOutput, ElevatorConstants.kCalibrationPower);
        break;
      case DISABLED:
        m_motor.neutralOutput();
        break;
      case MANUAL:
        m_motor.set(ControlMode.PercentOutput, m_desiredPower);
        break;
      case MOTIONMAGIC:
        updateClosedLoopSlot();
        m_motor.set(
          ControlMode.MotionMagic,
          Conversions.MetersToFalcon(heightToLength(m_desiredHeight), ElevatorConstants.kSpoolCircumference, ElevatorConstants.kGearRatio)
        );
        break;
    }
  }

  public void setupShuffleboard() {
    m_elevatorTab.addDouble("Current Height (m)", this::getHeight);
    m_elevatorTab.addDouble("Current Position (m)", this::getPosition);
    m_elevatorTab.addDouble("Desired Height (m)", () -> m_desiredHeight);
    m_elevatorTab.addDouble("Desired Power", () -> m_desiredPower);
    m_elevatorTab.addString("Mode", m_mode::toString);
  }
}
