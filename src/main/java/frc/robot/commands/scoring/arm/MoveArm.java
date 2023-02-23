package frc.robot.commands.scoring.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmMode;

public class MoveArm extends SequentialCommandGroup {
  public MoveArm(Arm arm, double desiredPosition) {
    addRequirements(arm);
    addCommands(
      new InstantCommand(() -> arm.setDesiredPosition(desiredPosition), arm),
      new InstantCommand(() -> arm.setMode(ArmMode.POSITION), arm)
    );
  }
}
