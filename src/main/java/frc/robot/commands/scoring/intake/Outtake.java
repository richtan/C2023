package frc.robot.commands.scoring.intake;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.scoring.Stow;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeMode;

public class Outtake extends SequentialCommandGroup {
  public Outtake(Intake intake, Elevator elevator, Arm arm, boolean isPassive) {
    addRequirements(intake, elevator, arm);
    // addCommands(
    //   new InstantCommand(() -> intake.setMode(IntakeMode.DROPPING), intake),
    //   new WaitUntilCommand(intake::isEmpty),
    //   new StopIntake(intake)
    // );
    // addCommands(
    //   new InstantCommand(() -> intake.setMode(IntakeMode.DROPPING), intake)
    // );
    addCommands(
      new ConditionalCommand(
        new InstantCommand(() -> intake.setIdleMode(IdleMode.kCoast)),
        new InstantCommand(() -> intake.setMode(IntakeMode.OUTTAKE)),
        () -> isPassive
      ),
      new WaitUntilCommand(intake::isEmpty),
      new Stow(intake, elevator, arm)
    );
  }
}
