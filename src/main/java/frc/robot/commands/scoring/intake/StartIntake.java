package frc.robot.commands.scoring.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;

public class StartIntake extends SequentialCommandGroup {
  public StartIntake(Intake intake, Elevator elevator, Arm arm) {
    addRequirements(intake, elevator, arm);
    // addCommands(
    //   new PositionIntake(elevator, arm, intake::hasCone, Position.BOTTOM),
    //   new InstantCommand(() -> intake.setMode(IntakeMode.INTAKE), intake),
    //   new ParallelRaceGroup(
    //     new WaitUntilCommand(intake::hasCube),
    //     new WaitUntilCommand(intake::hasCone)
    //   ),
    //   new StopIntake(intake)
    // );
    addCommands(
      new PositionIntake(elevator, arm, intake::hasCone, Position.INTAKE),
      new InstantCommand(() -> intake.setMode(IntakeMode.INTAKE), intake)
    );
  }
}
