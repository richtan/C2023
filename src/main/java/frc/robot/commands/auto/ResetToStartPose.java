package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.util.PathLoader;

public class ResetToStartPose extends SequentialCommandGroup {
  public ResetToStartPose(Swerve swerve, List<PathPlannerTrajectory> pathGroup) {
    addRequirements(swerve);

    addCommands(
      new InstantCommand(() -> swerve.resetOdometry(
        PathLoader.transformTrajectoryForAlliance(
          pathGroup.get(0),
          DriverStation.getAlliance()
        ).getInitialHolonomicPose()
      ), swerve),
      new InstantCommand(() -> swerve.setYaw(
        PathLoader.transformYawForAlliance(
          PathLoader.transformTrajectoryForAlliance(
            pathGroup.get(0),
            DriverStation.getAlliance()
          ).getInitialHolonomicPose().getRotation(),
          DriverStation.getAlliance()
        ).getDegrees()
      ), swerve)
    );
  }
}
