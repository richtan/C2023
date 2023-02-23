package frc.robot.commands.scoring.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.scoring.PositionMechanisms;
import frc.robot.commands.scoring.PositionMechanisms.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;

public class IntakeCommand extends SequentialCommandGroup {
  public IntakeCommand(Intake intake, Elevator elevator, Arm arm) {
    addRequirements(intake);
    addCommands(
      new PositionMechanisms(elevator, arm, intake::isConeColor, Position.INTAKE),
      new InstantCommand(() -> intake.setMode(IntakeMode.INTAKE), intake),
      new ParallelRaceGroup(
        new WaitUntilCommand(intake::hasCube),
        new WaitUntilCommand(intake::hasCone)
      ),
      new StopIntake(intake)
    );
  }

  public enum Level {
    GROUND, SHELF
  }
}
