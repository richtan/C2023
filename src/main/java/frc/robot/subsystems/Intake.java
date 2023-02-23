package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private final ShuffleboardTab m_intakeTab;

  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;

  private final ColorSensorV3 m_colorSensor;
  private final ColorMatch m_colorMatcher;

  private double m_proximity;
  private Color m_color;
  private Color m_closestColor;

  private IntakeMode m_mode;

  public Intake(ShuffleboardTab intakeTab) {
    m_intakeTab = intakeTab;

    m_leftMotor = new CANSparkMax(IntakeConstants.kLeftMotorID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(IntakeConstants.kRightMotorID, MotorType.kBrushless);
    configMotors();

    m_colorSensor = new ColorSensorV3(IntakeConstants.kColorSensorPort);
    m_colorMatcher = new ColorMatch();
    configColorSensor();

    m_mode = IntakeMode.DISABLED;
  }

  private void configMotors() {
    m_leftMotor.setInverted(IntakeConstants.kLeftMotorInvert);
    m_rightMotor.setInverted(IntakeConstants.kRightMotorInvert);

    m_leftMotor.setIdleMode(IntakeConstants.kLeftMotorIdleMode);
    m_rightMotor.setIdleMode(IntakeConstants.kRightMotorIdleMode);
  }

  private void configColorSensor() {
    m_colorMatcher.addColorMatch(IntakeConstants.kConeColor);
    m_colorMatcher.addColorMatch(IntakeConstants.kCubeColor);
  }

  public enum IntakeMode {
    DISABLED, INTAKE, OUTTAKE, EJECT
  }

  public void setMode(IntakeMode mode) {
    m_mode = mode;
  }

  public IntakeMode getMode() {
    return m_mode;
  }

  public boolean isConeColor() {
    return m_closestColor == IntakeConstants.kConeColor;
  }

  public boolean hasCone() {
    return isConeColor() && m_proximity > IntakeConstants.kConeProximityThreshold;
  }

  public boolean isCubeColor() {
    return m_closestColor == IntakeConstants.kCubeColor;
  }

  public boolean hasCube() {
    return isCubeColor() && m_proximity > IntakeConstants.kCubeProximityThreshold;
  }

  public boolean isEmpty() {
    return m_proximity < IntakeConstants.kEmptyProximityThreshold;
  }

  private void setMotorPowers(double power) {
    m_leftMotor.set(power);
    m_rightMotor.set(power);
  }

  private void updateColorSensorValues() {
    m_proximity = m_colorSensor.getProximity();
    m_color = m_colorSensor.getColor();
    m_closestColor = m_colorMatcher.matchClosestColor(m_color).color;
  }

  @Override
  public void periodic() {
    updateColorSensorValues();

    switch (m_mode) {
      case DISABLED:
        setMotorPowers(0);
        break;
      case INTAKE:
        setMotorPowers(IntakeConstants.kIntakePower);
        break;
      case OUTTAKE:
        setMotorPowers(IntakeConstants.kOuttakePower);
        break;
      case EJECT:
        setMotorPowers(IntakeConstants.kEjectPower);
        break;
    }
  }

  public void setupShuffleboard() {
    m_intakeTab.addBoolean("Has Cone", this::hasCone);
    m_intakeTab.addBoolean("Has Cube", this::hasCube);
    m_intakeTab.addBoolean("Is Empty", this::isEmpty);
    m_intakeTab.addDouble("Proximity (2400 [close] to 0 [far])", () -> m_proximity);
    m_intakeTab.addString("Color (hex)", () -> m_color.toHexString());
    m_intakeTab.addString("Closest color (hex)", () -> m_closestColor.toHexString());
    m_intakeTab.addString("Mode", () -> m_mode.toString());
    m_intakeTab.add("Left Motor", m_leftMotor);
    m_intakeTab.add("Right Motor", m_rightMotor);
    m_intakeTab.add("Color Sensor", m_colorSensor);
  }
}
