package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.controllers.GameController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.intake.StartIntake;
import frc.robot.commands.swerve.CharacterizeSwerve;
import frc.robot.commands.swerve.LockModules;
import frc.robot.commands.scoring.intake.Outtake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm.ArmMode;

public class OI {
  public static void configureDriverControls(GameController driver, Swerve swerve) {
    driver.START.onTrue(new InstantCommand(swerve::zeroGyro, swerve));
    driver.X.onTrue(new LockModules(swerve));
    driver.B.whileTrue(new CharacterizeSwerve(swerve, true, true));
    driver.A.whileTrue(new CharacterizeSwerve(swerve, false, true));
  }

  public static void configureOperatorControls(GameController operator, Elevator elevator, Arm arm, Intake intake) {
    // operator.START.onTrue(new CalibrateElevator(elevator));
    // operator.Y.onTrue(new PositionIntake(elevator, arm, intake::isConeColor, Position.TOP));
    // operator.X.onTrue(new PositionIntake(elevator, arm, intake::isConeColor, Position.MIDDLE));
    // operator.A.onTrue(new PositionIntake(elevator, arm, intake::isConeColor, Position.BOTTOM));
    // operator.B.onTrue(new PositionIntake(elevator, arm, intake::isConeColor, Position.SHELF));
    // // TODO: Add debounce for RB and RT triggers so the mechanisms don't start stowing due to accidental release of triggers
    // operator.RB.onTrue(new StartIntake(intake, elevator, arm)).onFalse(new Stow(intake, elevator, arm));
    // operator.RT.onTrue(new Outtake(intake)).onFalse(new Stow(intake, elevator, arm));
    // operator.LB.onTrue(new Stow(intake, elevator, arm));
    operator.A.onTrue(new InstantCommand(() -> arm.setDesiredPower(0.3), arm).andThen(() -> arm.setMode(ArmMode.MANUAL), arm));
  }

  public static void configureManualControls(GameController manual, Elevator elevator, Arm arm, Intake intake) {
  }

  public static void configureTestControls(GameController test, Elevator elevator, Arm arm, Intake intake) {

  }
}
