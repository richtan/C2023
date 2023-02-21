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

  public TeleopDrive(Swerve swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    m_swerve = swerve;
    addRequirements(swerve);

    m_translationSup = translationSup;
    m_strafeSup = strafeSup;
    m_rotationSup = rotationSup;
    m_robotCentricSup = robotCentricSup;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband*/
    double translationVal = MathUtil.applyDeadband(m_translationSup.getAsDouble(), OIConstants.kDeadband);
    double strafeVal = MathUtil.applyDeadband(m_strafeSup.getAsDouble(), OIConstants.kDeadband);
    double rotationVal = MathUtil.applyDeadband(m_rotationSup.getAsDouble(), OIConstants.kDeadband);

    /* Drive */
    m_swerve.drive(
      new Translation2d(translationVal, strafeVal).times(SwerveConstants.kMaxSpeed), 
      rotationVal * SwerveConstants.kMaxAngularVelocity, 
      !m_robotCentricSup.getAsBoolean(), 
      true
    );
  }
}