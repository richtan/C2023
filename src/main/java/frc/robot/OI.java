package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.controllers.GameController;
import frc.robot.subsystems.Swerve;

public class OI {
  public static void configureDriverControls(GameController driver, Swerve swerve) {
    driver.START.onTrue(new InstantCommand(() -> swerve.zeroGyro(), swerve));
  }

  public static void configureOperatorControls(GameController operator) {

  }

  public static void configureManualControls(GameController manual) {

  }

  public static void configureTestControls(GameController test) {

  }
}
