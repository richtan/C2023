package frc.robot.commands.swerve;

import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.DoubleStream;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;

import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;

public class CharacterizeSwerve extends SequentialCommandGroup {
  private boolean isCharacterizing;
  public CharacterizeSwerve(Swerve swerve, boolean isForwards, boolean isDriveMotors) {
    SwerveModule[] modules = swerve.getModules();
    Consumer<Double> voltageConsumer = isDriveMotors
        ? (Double voltage) -> {
          for (SwerveModule module : modules) {
            module.setDriveCharacterizationVoltage(voltage);
          }
        }
        : (Double voltage) -> {
          for (SwerveModule module : modules) {
            module.setAngleCharacterizationVoltage(voltage);
          }
        };

    Supplier<Double> velocitySupplier = isDriveMotors
        ? () -> {
          return DoubleStream.of(
              modules[0].getState().speedMetersPerSecond,
              modules[1].getState().speedMetersPerSecond,
              modules[2].getState().speedMetersPerSecond,
              modules[3].getState().speedMetersPerSecond)
              .average()
              .getAsDouble();
        }
        : () -> {
          return DoubleStream.of(
              modules[0].getAngularVelocity(),
              modules[1].getAngularVelocity(),
              modules[2].getAngularVelocity(),
              modules[3].getAngularVelocity())
              .average()
              .getAsDouble();
        };

    addCommands(
      new FeedForwardCharacterization(
        swerve,
        isForwards,
        new FeedForwardCharacterizationData("Swerve Drive"),
        voltageConsumer,
        velocitySupplier
      ).beforeStarting(() -> isCharacterizing = true).finallyDo((boolean interrupted) -> isCharacterizing = false)
    );
  }
}
