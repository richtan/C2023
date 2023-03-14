package frc.robot.commands.scoring;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake.IntakeMode;

public class Stow extends SequentialCommandGroup {
  public Stow(Intake intake, Elevator elevator, Wrist wrist, BooleanSupplier isConeSup) {
    addRequirements(intake, elevator, wrist);
    addCommands(
      new ConditionalCommand(
        new InstantCommand(() -> intake.setMode(IntakeMode.HOLDING_CONE)),
        new InstantCommand(() -> intake.setMode(IntakeMode.DISABLED)),
        isConeSup
      ),
      new PositionIntake(elevator, wrist, isConeSup, Position.STOW)
    );
  }
}