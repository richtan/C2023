package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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
  private double m_desiredPosition = 0;
  private double m_desiredPower = 0;

  public Elevator(ShuffleboardTab elevatorTab, BooleanSupplier hasConeSupplier) {
    m_elevatorTab = elevatorTab;

    m_hasConeSupplier = hasConeSupplier;
    m_mode = ElevatorMode.DISABLED;

    m_motor = new WPI_TalonFX(ElevatorConstants.kMotorID, ElevatorConstants.kElevatorCAN);
    configElevatorMotor();

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
      Conversions.MetersToFalcon(
        ElevatorConstants.kMaxPosition,
        ElevatorConstants.kSpoolCircumference,
        ElevatorConstants.kGearRatio
      )
    );
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
   * Get position of carriage above bottom position
   * @return position (m)
   */
  public double getPosition() {
    return Conversions.falconToMeters(
      m_motor.getSelectedSensorPosition(),
      ElevatorConstants.kSpoolCircumference,
      ElevatorConstants.kGearRatio
    );
  }

  public enum ElevatorMode {
    CALIBRATION, MANUAL, MOTIONMAGIC, DISABLED
  }

  public void setDesiredPosition(double desiredPosition) {
    m_desiredPosition = desiredPosition;
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

  public double getVelocity() {
    return Conversions.falconToMPS(
      m_motor.getSelectedSensorVelocity(),
      ElevatorConstants.kSpoolCircumference,
      ElevatorConstants.kGearRatio
    );
  }

  public boolean reachedDesiredPosition() {
    return Math.abs(m_desiredPosition - getPosition()) < ElevatorConstants.kPositionTolerance
          && Math.abs(getVelocity()) < ElevatorConstants.kVelocityTolerance;
  }

  private void updateClosedLoopSlot() {
    boolean hasCone = m_hasConeSupplier.getAsBoolean();
    if (getPosition() < ElevatorConstants.kCarriageMaxDistance) {
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
          Conversions.MetersToFalcon(m_desiredPosition, ElevatorConstants.kSpoolCircumference, ElevatorConstants.kGearRatio)
        );
        break;
    }
  }

  public void setupShuffleboard() {
    m_elevatorTab.addDouble("Current Position (m)", this::getPosition);
    m_elevatorTab.addDouble("Desired Position (m)", () -> m_desiredPosition);
    m_elevatorTab.addDouble("Desired Power", () -> m_desiredPower);
    m_elevatorTab.addBoolean("Reached desired position", this::reachedDesiredPosition);
    m_elevatorTab.addBoolean("Reached Top Limit Switch", this::isTopLimitSwitchReached);
    m_elevatorTab.addBoolean("Reached Bottom Limit Switch", this::isBottomLimitSwitchReached);
    m_elevatorTab.addString("Mode", () -> m_mode.toString());
  }
}
