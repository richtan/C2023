package frc.robot.commands.scoring.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;

public class Eject extends SequentialCommandGroup {
  public Eject(Intake intake) {
    addRequirements(intake);
    addCommands(
      new InstantCommand(() -> intake.setMode(IntakeMode.EJECT), intake),
      new WaitUntilCommand(intake::isEmpty),
      new StopIntake(intake)
    );
  }
}
