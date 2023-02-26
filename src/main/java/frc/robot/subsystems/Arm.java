package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  private final ShuffleboardTab m_armTab;

  private final CANSparkMax m_motor;
  private final DutyCycleEncoder m_absEncoder;

  private final SparkMaxPIDController m_pid;
  private final RelativeEncoder m_encoder;

  private ArmMode m_mode;
  private double m_desiredAngle;
  private double m_desiredPower;

  public Arm(ShuffleboardTab armTab) {
    m_armTab = armTab;

    m_motor = new CANSparkMax(ArmConstants.kMotorID, MotorType.kBrushless);
    m_encoder = m_motor.getEncoder();
    m_pid = m_motor.getPIDController();
    configArmMotor();

    m_absEncoder = new DutyCycleEncoder(ArmConstants.kAbsEncoderID);
    Timer.delay(1);
    calibrateEncoder();

    m_mode = ArmMode.DISABLED;
    m_desiredAngle = ArmConstants.kStowAngle;
    m_desiredPower = 0.0;

    setupShuffleboard();
  }

  private void configArmMotor() {
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(ArmConstants.kIdleMode);
    m_motor.setInverted(ArmConstants.kMotorInvert);

    m_motor.enableVoltageCompensation(Constants.kNormalOperatingVoltage);

    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_motor.setSoftLimit(SoftLimitDirection.kForward, (float) 45);
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float) 10);
    
    // Rotations at motor shaft to degrees at Throughbore
    m_encoder.setPositionConversionFactor(ArmConstants.kMotorEncoderDistancePerRotation);

    m_pid.setFeedbackDevice(m_encoder);
    m_pid.setP(ArmConstants.kP);
    m_pid.setI(ArmConstants.kI);
    m_pid.setD(ArmConstants.kD);

    m_pid.setSmartMotionMaxVelocity(ArmConstants.kMaxAngularVelocity, 0);
    m_pid.setSmartMotionMaxAccel(ArmConstants.kMaxAngularAccel, 0);
  }

  /**
   * Get absolute encoder position in degrees
   * @return
   */
  private double getAbsEncoder() {
    return m_absEncoder.getAbsolutePosition() * ArmConstants.kAbsEncoderDistancePerRotation;
  }

  public void calibrateEncoder() {
    double absEncoderError = getAbsEncoder() - ArmConstants.kAbsEncoderZeroAngle;
    m_encoder.setPosition(absEncoderError);
  }

  public void zeroEncoder() {
    m_encoder.setPosition(0);
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

  public void setDesiredAngle(double desiredAngle) {
    m_desiredAngle = desiredAngle;
  }
  
  public void setDesiredPower(double desiredPower) {
    m_desiredPower = desiredPower;
  }

  public double getAngle() {
    return m_encoder.getPosition();
  }

  public boolean reachedDesiredAngle() {
    return Math.abs(m_desiredAngle - getAngle()) < ArmConstants.kAngleTolerance
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
        m_pid.setReference(m_desiredAngle, ControlType.kPosition, 0, 1.5 * Math.cos(Units.degreesToRadians(getAngle()))); // 
    }
  }

  private void setupShuffleboard() {
    m_armTab.addDouble("Current angle (deg)", this::getAngle);
    m_armTab.addDouble("Desired angle (deg)", () -> m_desiredAngle);
    m_armTab.addDouble("Desired Power", () -> m_desiredPower);
    m_armTab.addBoolean("Reached Desired Angle", this::reachedDesiredAngle);
    m_armTab.addDouble("Absolute encoder (deg)", this::getAbsEncoder);
    m_armTab.addDouble("Output Current (A)", m_motor::getAppliedOutput);
    m_armTab.addString("Mode", () -> m_mode.toString());
  }
}
