package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.intake.StopIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class Stow extends ParallelCommandGroup {
  public Stow(Intake intake, Elevator elevator, Arm arm) {
    addRequirements(intake, elevator, arm);
    addCommands(
      new StopIntake(intake),
      new PositionIntake(elevator, arm, intake::isConeColor, Position.STOW)
    );
  }
}