package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.controllers.GameController;
import frc.lib.math.Conversions;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.arm.CalibrateArm;
import frc.robot.commands.scoring.elevator.CalibrateElevator;
import frc.robot.commands.scoring.intake.Outtake;
import frc.robot.commands.scoring.intake.StartIntake;
import frc.robot.commands.swerve.CharacterizeSwerve;
import frc.robot.commands.swerve.LockModules;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.IntakeMode;

public class OI {
  public static void configureDriverControls(GameController driver, Swerve swerve) {
    driver.START.onTrue(new InstantCommand(swerve::zeroGyro, swerve));
    driver.X.onTrue(new LockModules(swerve));
  }

  public static void configureOperatorControls(GameController operator, Elevator elevator, Arm arm, Intake intake) {
    operator.BACK.onTrue(new CalibrateElevator(elevator));
    operator.START.onTrue(new CalibrateArm(arm));
    operator.Y.onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.TOP));
    operator.X.onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.MIDDLE));
    operator.A.onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.BOTTOM));
    operator.B.onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.SHELF));
    operator.RB.onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.STOW));
    operator.LB.onTrue(new StartIntake(intake, elevator, arm)).onFalse(new Stow(intake, elevator, arm)); // add sequence
    operator.LT.onTrue(new Outtake(intake, elevator, arm, true)).onFalse(new Stow(intake, elevator, arm));
    operator.RT.onTrue(new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()));
    operator.DPAD_DOWN.onTrue(new InstantCommand(() -> intake.setMode(IntakeMode.INTAKE)));
    operator.DPAD_RIGHT.onTrue(new InstantCommand(() -> intake.setMode(IntakeMode.DISABLED)));
    operator.DPAD_LEFT.onTrue(new InstantCommand(() -> intake.setMode(IntakeMode.DROPPING)));
    operator.DPAD_UP.onTrue(new InstantCommand(() -> intake.setMode(IntakeMode.OUTTAKE)));
  }

  public static void configureManualControls(GameController manual, Elevator elevator, Arm arm, Intake intake) {
    // manual.B.onTrue(new InstantCommand(() -> arm.setDesiredAngle(25), arm).andThen(() -> arm.setMode(ArmMode.POSITION), arm));
    // manual.A.onTrue(new InstantCommand(() -> arm.setDesiredAngle(-5), arm).andThen(() -> arm.setMode(ArmMode.POSITION), arm));
    // manual.Y.onTrue(new InstantCommand(() -> arm.setDesiredAngle(45), arm).andThen(() -> arm.setMode(ArmMode.POSITION), arm));
    manual.START.onTrue(new InstantCommand(() -> arm.zeroEncoder(), arm));
    manual.DPAD_DOWN.onTrue(new InstantCommand(() -> intake.setMode(IntakeMode.INTAKE)));
    manual.DPAD_RIGHT.onTrue(new InstantCommand(() -> intake.setMode(IntakeMode.DISABLED)));
    manual.DPAD_LEFT.onTrue(new InstantCommand(() -> intake.setMode(IntakeMode.DROPPING)));
    manual.DPAD_UP.onTrue(new InstantCommand(() -> intake.setMode(IntakeMode.OUTTAKE)));
    manual.BACK.onTrue(new CalibrateElevator(elevator));
    // manual.LB.onTrue(new Stow(intake, elevator, arm));
    // manual.RB.onTrue(new StartIntake(intake, elevator, arm));

    //  operator.RB.onTrue(new StartIntake(intake, elevator, arm)).onFalse(new Stow(intake, elevator, arm));
    // operator.RT.onTrue(new Outtake(intake)).onFalse(new Stow(intake, elevator, arm));
    // operator.LB.onTrue(new Stow(intake, elevator, arm));
    // manual.RT.onTrue(new MoveElevator(elevator, Conversions.ElevatorLengthToHeight(1.25)));
    // manual.LT.onTrue(new MoveElevator(elevator, 1));

    // manual.DPAD_DOWN.onTrue(new StartIntake(intake, elevator, arm)).onFalse(new PositionIntake(elevator, arm, intake::hasCone, Position.STOW));
    // manual.DPAD_UP.onTrue(new Outtake(intake)).onFalse(new PositionIntake(elevator, arm, intake::hasCone, Position.STOW));
    // manual.LB.onTrue(new Stow(intake, elevator, arm));
    manual.Y.onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.TOP));
    manual.X.onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.MIDDLE));
    manual.A.onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.INTAKE));
    manual.B.onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.SHELF));
    manual.RB.onTrue(new PositionIntake(elevator, arm, intake::hasCone, Position.STOW));
  }

  public static void configureTestControls(GameController test, Swerve swerve, Elevator elevator, Arm arm, Intake intake) {
    test.B.whileTrue(new CharacterizeSwerve(swerve, true, true));
    test.A.whileTrue(new CharacterizeSwerve(swerve, false, true));
  }
}