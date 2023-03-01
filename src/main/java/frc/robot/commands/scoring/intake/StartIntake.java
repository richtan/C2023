package frc.robot.commands.scoring.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;

public class StartIntake extends SequentialCommandGroup {
  public StartIntake(Intake intake, Elevator elevator, Arm arm) {
    addRequirements(intake, elevator, arm);
    addCommands(
      new InstantCommand(() -> intake.setMode(IntakeMode.INTAKE), intake),
      new WaitUntilCommand(() -> !intake.isEmpty()),
      new StopIntake(intake)
    );
  }
}
