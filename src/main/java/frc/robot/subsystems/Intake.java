package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final ShuffleboardTab m_intakeTab;

  private final WPI_TalonFX m_motor;

  private IntakeMode m_mode;

  private double m_holdingPosition;

  public Intake(ShuffleboardTab intakeTab) {
    m_intakeTab = intakeTab;

    m_motor = new WPI_TalonFX(IntakeConstants.kMotorID, IntakeConstants.kIntakeCAN);
    configIntakeMotor();

    m_mode = IntakeMode.DISABLED;
    m_holdingPosition = getPosition();

    setupShuffleboard();
  }

  private void configIntakeMotor() {
    m_motor.configFactoryDefault();

    m_motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      IntakeConstants.kEnableCurrentLimit,
      IntakeConstants.kContinuousCurrentLimit,
      IntakeConstants.kPeakCurrentLimit,
      IntakeConstants.kPeakCurrentDuration
    ));

    m_motor.config_kP(0, IntakeConstants.kP);
    m_motor.config_kI(0, IntakeConstants.kI);
    m_motor.config_kD(0, IntakeConstants.kD);
    m_motor.config_kF(0, IntakeConstants.kF);

    m_motor.setInverted(IntakeConstants.kMotorInvert);
    m_motor.setNeutralMode(IntakeConstants.kNeutralMode);

    m_motor.configVoltageCompSaturation(Constants.kNormalOperatingVoltage);
    m_motor.enableVoltageCompensation(true);

    m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  public enum IntakeMode {
    DISABLED, HOLDING_CONE, INTAKE_CONE, INTAKE_CUBE, OUTTAKE_CONE, OUTTAKE_CUBE, EJECT_CONE, EJECT_CUBE
  }

  public double getPosition() {
    return m_motor.getSelectedSensorPosition();
  }

  public void setHoldingPosition(double holdingPosition) {
    m_holdingPosition = holdingPosition;
  }

  public void setMode(IntakeMode mode) {
    m_mode = mode;
  }

  public IntakeMode getMode() {
    return m_mode;
  }

  public void setNeutralMode(NeutralMode neutralMode) {
    m_motor.setNeutralMode(neutralMode);
  }

  public double getTangentialVelocity() {
    return Conversions.falconToMPS(m_motor.getSelectedSensorVelocity(), IntakeConstants.kRollerCircumference, IntakeConstants.kGearRatio);
  }

  @Override
  public void periodic() {
    switch (m_mode) {
      case DISABLED:
        m_motor.stopMotor();
        break;
      case HOLDING_CONE:
        m_motor.set(ControlMode.Position, m_holdingPosition);
        break;
      case INTAKE_CONE:
        m_motor.set(ControlMode.PercentOutput, IntakeConstants.kIntakeConePower);
        break;
      case INTAKE_CUBE:
        m_motor.set(ControlMode.PercentOutput, IntakeConstants.kIntakeCubePower);
        break;
      case OUTTAKE_CONE:
        m_motor.set(ControlMode.PercentOutput, IntakeConstants.kOuttakeConePower);
        break;
      case OUTTAKE_CUBE:
        m_motor.set(ControlMode.PercentOutput, IntakeConstants.kOuttakeCubePower);
        break;
      case EJECT_CONE:
        m_motor.set(ControlMode.PercentOutput, IntakeConstants.kEjectConePower);
        break;
      case EJECT_CUBE:
        m_motor.set(ControlMode.PercentOutput, IntakeConstants.kEjectCubePower);
        break;
    }
  }

  private void setupShuffleboard() {
    if (Constants.kUseTelemetry) {
      m_intakeTab.addString("Mode", () -> m_mode.toString());
      m_intakeTab.addDouble("Holding position", () -> m_holdingPosition);
      m_intakeTab.addDouble("Tangential velocity (m/s)", () -> getTangentialVelocity());
    }
  }
}
