package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve;

public class TeleopDrive extends CommandBase {
  private final Swerve m_swerve;    
  private final DoubleSupplier m_translationSup;
  private final DoubleSupplier m_strafeSup;
  private final DoubleSupplier m_rotationSup;
  private final BooleanSupplier m_robotCentricSup;
  private final BooleanSupplier m_slowModeSup;

  public TeleopDrive(
    Swerve swerve,
    DoubleSupplier translationSup,
    DoubleSupplier strafeSup,
    DoubleSupplier rotationSup,
    BooleanSupplier robotCentricSup,
    BooleanSupplier slowModeSup
  ) {
    m_swerve = swerve;
    addRequirements(swerve);

    m_translationSup = translationSup;
    m_strafeSup = strafeSup;
    m_rotationSup = rotationSup;
    m_robotCentricSup = robotCentricSup;
    m_slowModeSup = slowModeSup;
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

    /* Drive */
    m_swerve.drive(
      new Translation2d(translationVal, strafeVal).times(SwerveConstants.kMaxSpeed).times(slowFactor), 
      rotationVal * SwerveConstants.kMaxAngularVelocity * slowFactor, 
      !m_robotCentricSup.getAsBoolean(), 
      true
    );
  }
}