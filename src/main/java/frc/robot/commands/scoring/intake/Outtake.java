package frc.robot.commands.scoring.intake;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;

public class Outtake extends SequentialCommandGroup {
  public Outtake(Intake intake) {
    addRequirements(intake);
    // addCommands(
    //   new InstantCommand(() -> intake.setMode(IntakeMode.DROPPING), intake),
    //   new WaitUntilCommand(intake::isEmpty),
    //   new StopIntake(intake)
    // );
    // addCommands(
    //   new InstantCommand(() -> intake.setMode(IntakeMode.DROPPING), intake)
    // );
    addCommands(
      new InstantCommand(() -> intake.setIdleMode(IdleMode.kCoast))
    );
  }
}
