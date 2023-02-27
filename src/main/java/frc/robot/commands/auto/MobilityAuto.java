package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class MobilityAuto extends SequentialCommandGroup {
  private final Swerve m_swerve;

  public MobilityAuto(Swerve swerve) {
    m_swerve = swerve;
    addRequirements(swerve);

    addCommands(
      new FollowPath("MobilityAuto", 0, m_swerve),
      new InstantCommand(() -> m_swerve.stop())
    );
  }
}
