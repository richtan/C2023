package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class LockModules extends SequentialCommandGroup {
  public LockModules(Swerve swerve) {
    addRequirements(swerve);
    addCommands(
      new InstantCommand(() -> swerve.toggleAngleJitterPrevention(false), swerve),
      new InstantCommand(() -> swerve.setModuleStates(new SwerveModuleState[] {
        new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45))),
        new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45))),
        new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(-45))),
        new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(45)))
      }), swerve),
      new InstantCommand(() -> swerve.toggleAngleJitterPrevention(true), swerve)
    );
  }
}