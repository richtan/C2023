package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final ShuffleboardTab m_armTab;

  private final CANSparkMax m_motor;
  private final DutyCycleEncoder m_absEncoder;

  private SparkMaxPIDController m_pid;
  private RelativeEncoder m_encoder;

  private ArmMode m_mode;
  private double m_desiredPosition;
  private double m_desiredPower;

  public Arm(ShuffleboardTab armTab) {
    m_armTab = armTab;

    m_motor = new CANSparkMax(ArmConstants.kMotorID, MotorType.kBrushless);
    configArmMotor();

    m_absEncoder = new DutyCycleEncoder(ArmConstants.kAbsEncoderID);
    calibrateEncoder();

    m_mode = ArmMode.DISABLED;
    m_desiredPosition = ArmConstants.kStowPosition;
    m_desiredPower = 0.0;
  }

  private void configArmMotor() {
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(ArmConstants.kIdleMode);
    m_motor.setInverted(ArmConstants.kMotorInvert);

    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) ArmConstants.kDeployPosition);
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) ArmConstants.kStowPosition);
    
    m_encoder = m_motor.getEncoder();
    m_encoder.setPositionConversionFactor(1.0 / ArmConstants.kGearRatio); // Rotations at motor to rotations at throughbore

    m_pid = m_motor.getPIDController();

    m_pid.setFeedbackDevice(m_encoder);
    m_pid.setP(ArmConstants.kP);
    m_pid.setI(ArmConstants.kI);
    m_pid.setD(ArmConstants.kD);

    m_pid.setSmartMotionMaxVelocity(ArmConstants.kMaxVelocity, 0);
    m_pid.setSmartMotionMaxAccel(ArmConstants.kMaxAccel, 0);
  }

  private void calibrateEncoder() {
    double absEncoderError = m_absEncoder.getAbsolutePosition() - ArmConstants.kAbsEncoderZeroPos;
    m_encoder.setPosition(absEncoderError);
  }

  public enum ArmMode {
    DISABLED, MANUAL, POSITION
  }

  public void setMode(ArmMode mode) {
    m_mode = mode;
  }

  public ArmMode getMode() {
    return m_mode;
  }

  public void setDesiredPosition(double desiredPosition) {
    m_desiredPosition = desiredPosition;
  }
  
  public void setDesiredPower(double desiredPower) {
    m_desiredPower = desiredPower;
  }

  public double getPosition() {
    return m_encoder.getPosition();
  }

  public boolean reachedDesiredPosition() {
    return Math.abs(m_desiredPosition - getPosition()) < ArmConstants.kPositionTolerance
          && Math.abs(m_encoder.getVelocity()) < ArmConstants.kVelocityTolerance;
  }

  @Override
  public void periodic() {
    switch (m_mode) {
      case DISABLED:
        m_pid.setReference(0, ControlType.kDutyCycle);
        break;
      case MANUAL:
        m_pid.setReference(m_desiredPower, ControlType.kDutyCycle);
        break;
      case POSITION:
        m_pid.setReference(m_desiredPosition, ControlType.kSmartMotion);
    }
  }

  public void setupShuffleboard() {
    m_armTab.addDouble("Current position", this::getPosition);
  }
}
