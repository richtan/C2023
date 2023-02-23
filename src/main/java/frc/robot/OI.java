package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.controllers.GameController;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.scoring.PositionMechanisms;
import frc.robot.commands.scoring.PositionMechanisms.Position;
import frc.robot.commands.scoring.arm.MoveArm;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.intake.IntakeCommand;
import frc.robot.commands.scoring.intake.Outtake;
import frc.robot.commands.scoring.intake.StopIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class OI {
  public static void configureDriverControls(GameController driver, Swerve swerve) {
    driver.START.onTrue(new InstantCommand(() -> swerve.zeroGyro(), swerve));
  }

  public static void configureOperatorControls(GameController operator, Elevator elevator, Arm arm, Intake intake) {
    operator.START.onTrue(new CalibrateElevator(elevator));
    operator.Y.onTrue(new PositionMechanisms(elevator, arm, intake::isConeColor, Position.TOP));
    operator.X.onTrue(new PositionMechanisms(elevator, arm, intake::isConeColor, Position.MIDDLE));
    operator.A.onTrue(new PositionMechanisms(elevator, arm, intake::isConeColor, Position.BOTTOM));
    operator.B.onTrue(new PositionMechanisms(elevator, arm, intake::isConeColor, Position.SHELF));
    operator.RB.onTrue(new IntakeCommand(intake, elevator, arm)).onFalse(new StopIntake(intake));
    operator.RT.onTrue(new Outtake(intake)).onFalse(new StopIntake(intake));
    operator.LB.onTrue(new MoveArm(arm, ArmConstants.kStowAngle));
  }

  public static void configureManualControls(GameController manual) {

  }

  public static void configureTestControls(GameController test) {

  }
}
