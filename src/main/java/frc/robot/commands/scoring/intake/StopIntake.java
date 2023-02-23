package frc.robot.commands.scoring.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;

public class StopIntake extends InstantCommand {
  public StopIntake(Intake intake) {
    super(() -> intake.setMode(IntakeMode.DISABLED), intake);
  }
}