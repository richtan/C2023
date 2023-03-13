package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.controllers.GameController;
import frc.lib.math.Conversions;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.auto.BalanceOnChargeStation;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.intake.Outtake;
import frc.robot.commands.scoring.intake.StartIntake;
import frc.robot.commands.swerve.CharacterizeSwerve;
import frc.robot.commands.swerve.LockModules;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.IntakeMode;

public class OI {
  public static void configureDriverControls(GameController driver, Swerve swerve) {
    driver.START.onTrue(new InstantCommand(swerve::zeroGyro, swerve));
    driver.X.onTrue(new LockModules(swerve));
    driver.DPAD_UP.onTrue(new BalanceOnChargeStation(swerve));
  }

  public static void configureOperatorControls(GameController operator, Elevator elevator, Wrist wrist, Intake intake) {
    operator.START.onTrue(new CalibrateElevator(elevator));

    operator.DPAD_UP.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));

    operator.Y.onTrue(new PositionIntake(elevator, wrist, operator.RT, Position.TOP));
    operator.X.onTrue(new PositionIntake(elevator, wrist, operator.RT, Position.MIDDLE));
    operator.A.onTrue(new PositionIntake(elevator, wrist, operator.RT, Position.INTAKE));
    operator.B.onTrue(new PositionIntake(elevator, wrist, operator.RT, Position.SHELF));
    operator.RB.onTrue(new PositionIntake(elevator, wrist, operator.RT, Position.STOW));

    operator.LB.onTrue(new StartIntake(intake, operator.RT)).onFalse(new Stow(intake, elevator, wrist, operator.RT));
    operator.LT.onTrue(new Outtake(intake, operator.RT)).onFalse(new Stow(intake, elevator, wrist, operator.RT));
  }

  public static void configureManualControls(GameController manual, Elevator elevator, Wrist wrist, Intake intake) {
    manual.START.onTrue(new CalibrateElevator(elevator));

    manual.LT.onTrue(new InstantCommand(() -> intake.setMode(IntakeMode.OUTTAKE_CONE)));
    manual.LB.onTrue(new InstantCommand(() -> intake.setMode(IntakeMode.INTAKE_CONE)));
    manual.RT.onTrue(new InstantCommand(() -> intake.setMode(IntakeMode.OUTTAKE_CUBE)));
    manual.RB.onTrue(new InstantCommand(() -> intake.setMode(IntakeMode.INTAKE_CUBE)));

    manual.BACK.onTrue(new InstantCommand(() -> intake.setMode(IntakeMode.DISABLED)));

    manual.Y.onTrue(new PositionIntake(elevator, wrist, manual.RT, Position.TOP));
    manual.X.onTrue(new PositionIntake(elevator, wrist, manual.RT, Position.MIDDLE));
    manual.A.onTrue(new PositionIntake(elevator, wrist, manual.RT, Position.INTAKE));
    manual.B.onTrue(new PositionIntake(elevator, wrist, manual.RT, Position.SHELF));
    manual.DPAD_DOWN.onTrue(new PositionIntake(elevator, wrist, manual.RT, Position.STOW));
  }

  public static void configureTestControls(GameController test, Swerve swerve, Elevator elevator, Wrist wrist, Intake intake) {
    test.B.whileTrue(new CharacterizeSwerve(swerve, true, true));
    test.A.whileTrue(new CharacterizeSwerve(swerve, false, true));
  }
}