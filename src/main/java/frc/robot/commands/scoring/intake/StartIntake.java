package frc.robot.commands.scoring.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;

public class StartIntake extends SequentialCommandGroup {
  public StartIntake(Intake intake, BooleanSupplier isConeSup) {
    addRequirements(intake);
    addCommands(
      new ConditionalCommand(
        new InstantCommand(() -> intake.setMode(IntakeMode.INTAKE_CONE), intake),
        new InstantCommand(() -> intake.setMode(IntakeMode.INTAKE_CUBE), intake),
        isConeSup
      )
    );
  }
}
