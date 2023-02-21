package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class LockModules extends CommandBase {
  private final Swerve m_swerve;    

  public LockModules(Swerve swerve) {
    m_swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    m_swerve.setModuleStates(new SwerveModuleState[] {
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45))),
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45))),
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45))),
      new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45)))
    });
  }
}