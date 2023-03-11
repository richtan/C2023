package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve;

public class TeleopDrive extends CommandBase {
  private final Swerve m_swerve;
  private final PIDController m_pid;
  
  private final DoubleSupplier m_translationSup;
  private final DoubleSupplier m_strafeSup;
  private final DoubleSupplier m_rotationSup;
  private final BooleanSupplier m_robotCentricSup;
  private final BooleanSupplier m_slowModeSup;
  private final BooleanSupplier m_alignModeSup;

  public TeleopDrive(
    Swerve swerve,
    DoubleSupplier translationSup,
    DoubleSupplier strafeSup,
    DoubleSupplier rotationSup,
    BooleanSupplier robotCentricSup,
    BooleanSupplier slowModeSup,
    BooleanSupplier alignModeSup
  ) {
    m_swerve = swerve;
    addRequirements(swerve);

    m_translationSup = translationSup;
    m_strafeSup = strafeSup;
    m_rotationSup = rotationSup;
    m_robotCentricSup = robotCentricSup;
    m_slowModeSup = slowModeSup;
    m_alignModeSup = alignModeSup;

    m_pid = new PIDController(0.1, 0, 0);
    m_pid.enableContinuousInput(-180, 180);
    m_pid.setTolerance(0.25, 0.25);
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal = MathUtil.applyDeadband(m_translationSup.getAsDouble(), OIConstants.kDriverDeadband);
    double strafeVal = MathUtil.applyDeadband(m_strafeSup.getAsDouble(), OIConstants.kDriverDeadband);
    double rotationVal = MathUtil.applyDeadband(m_rotationSup.getAsDouble(), OIConstants.kDriverDeadband);

    translationVal = Math.copySign(translationVal * translationVal, translationVal);
    strafeVal = Math.copySign(strafeVal * strafeVal, strafeVal);
    rotationVal = Math.copySign(rotationVal * rotationVal, rotationVal);

    double slowFactor = m_slowModeSup.getAsBoolean() ? SwerveConstants.kSlowDriveFactor : 1;

    Rotation2d currentYaw = m_swerve.getYaw();
    double turnEffort = m_alignModeSup.getAsBoolean()
        ? (-m_pid.calculate(Units.radiansToDegrees(MathUtil.angleModulus(currentYaw.getRadians())),
            ((Math.abs(currentYaw.getDegrees()) % 360 > 90 && Math.abs(currentYaw.getDegrees()) % 360 < 270) ? 180 : 0)))
        : (rotationVal * SwerveConstants.kMaxAngularVelocity * slowFactor);

    /* Drive */
    m_swerve.drive(
      new Translation2d(translationVal, strafeVal).times(SwerveConstants.kMaxSpeed).times(slowFactor), 
      turnEffort, 
      !m_robotCentricSup.getAsBoolean(), 
      true
    );
  }
}