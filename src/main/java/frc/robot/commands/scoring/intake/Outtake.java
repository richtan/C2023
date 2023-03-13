package frc.robot.commands.scoring.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;

public class Outtake extends SequentialCommandGroup {
  public Outtake(Intake intake, BooleanSupplier isConeSup) {
    addRequirements(intake);
    addCommands(
      new ConditionalCommand(
        new InstantCommand(() -> intake.setMode(IntakeMode.OUTTAKE_CONE), intake),
        new InstantCommand(() -> intake.setMode(IntakeMode.OUTTAKE_CUBE), intake),
        isConeSup
      )
    );
  }
}
