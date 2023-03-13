package frc.robot.commands.scoring.wrist;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristMode;

public class MoveWrist extends SequentialCommandGroup {
  public MoveWrist(Wrist wrist, double desiredPosition) {
    addRequirements(wrist);
    addCommands(
      new InstantCommand(() -> wrist.setDesiredAngle(desiredPosition), wrist),
      new InstantCommand(() -> wrist.setMode(WristMode.POSITION), wrist)
    );
  }
}
