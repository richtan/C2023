package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

// import com.revrobotics.Rev2mDistanceSensor;
// import com.revrobotics.Rev2mDistanceSensor.Port;
// import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
// import com.revrobotics.Rev2mDistanceSensor.Unit;

public class Intake extends SubsystemBase {
  private final ShuffleboardTab m_intakeTab;

  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;

  // private final Rev2mDistanceSensor m_distanceSensor;

  private double m_range = -1;
  private boolean m_hasCone = false;
  private boolean m_hasCube = false;
  private double m_cubeTrackingStartTime = 0;

  private IntakeMode m_mode;

  public Intake(ShuffleboardTab intakeTab) {
    m_intakeTab = intakeTab;

    m_leftMotor = new CANSparkMax(IntakeConstants.kLeftMotorID, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(IntakeConstants.kRightMotorID, MotorType.kBrushless);
    configMotors();

    // m_distanceSensor = new Rev2mDistanceSensor(IntakeConstants.kDistanceSensorPort, Unit.kInches, RangeProfile.kDefault);
    // configDistanceSensor();

    m_mode = IntakeMode.DISABLED;

    setupShuffleboard();
  }

  private void configMotors() {
    m_leftMotor.setInverted(IntakeConstants.kLeftMotorInvert);
    m_rightMotor.setInverted(IntakeConstants.kRightMotorInvert);

    m_leftMotor.setIdleMode(IntakeConstants.kLeftMotorIdleMode);
    m_rightMotor.setIdleMode(IntakeConstants.kRightMotorIdleMode);

    m_leftMotor.enableVoltageCompensation(Constants.kNormalOperatingVoltage);
    m_rightMotor.enableVoltageCompensation(Constants.kNormalOperatingVoltage);
  }

  private void configDistanceSensor() {
    // m_distanceSensor.setAutomaticMode(true);
    // m_distanceSensor.setEnabled(true);
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

  public boolean hasCone() {
    return m_hasCone;
  }

  public boolean hasCube() {
    return m_hasCube;
  }

  public boolean isEmpty() {
    return !hasCone() && !hasCube();
  }

  private void setMotorPowers(double power) {
    m_leftMotor.set(power);
    m_rightMotor.set(power);
  }

  private void updateSensorValues() {
    // if (!m_distanceSensor.isRangeValid()) {
    //   m_hasCone = false;
    //   m_hasCube = false;
    // }
    // m_range = m_distanceSensor.getRange();
    // if (m_range <= IntakeConstants.kMaxConeRange) { // Has cone
    //   m_cubeTrackingStartTime = Timer.getFPGATimestamp();
    //   m_hasCone = true;
    //   m_hasCube = false;
    // } else if (m_range <= IntakeConstants.kMaxCubeRange) {
    //   if (Timer.getFPGATimestamp() - m_cubeTrackingStartTime >= IntakeConstants.kCubeTimeThreshold) { // Has cube
    //     m_hasCone = false;
    //     m_hasCube = true;
    //   } else { // Cone is in the middle of entering intake
    //     m_cubeTrackingStartTime = Timer.getFPGATimestamp();
    //     m_hasCone = false;
    //     m_hasCube = false;
    //   }
    // } else { // Is empty
    //   m_cubeTrackingStartTime = Timer.getFPGATimestamp();
    //   m_hasCone = false;
    //   m_hasCube = false;
    // }
  }

  @Override
  public void periodic() {
    updateSensorValues();

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

  private void setupShuffleboard() {
    m_intakeTab.addBoolean("Has Cone", this::hasCone);
    m_intakeTab.addBoolean("Has Cube", this::hasCube);
    m_intakeTab.addBoolean("Is Empty", this::isEmpty);
    m_intakeTab.addDouble("Range (in)", () -> m_range);
    m_intakeTab.addString("Mode", () -> m_mode.toString());
    m_intakeTab.addDouble("Left Output Current (A)", () -> m_leftMotor.getOutputCurrent());
    m_intakeTab.addDouble("Right Output Current (A)", () -> m_rightMotor.getOutputCurrent());
  }
}
