package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.util.PathLoader;

public class MobilityAuto extends SequentialCommandGroup {
  public MobilityAuto(Swerve swerve) {
    addRequirements(swerve);
    
    List<PathPlannerTrajectory> pathGroup = PathLoader.getPathGroup("MobilityAuto");

    addCommands(
      new InstantCommand(() -> swerve.resetOdometry(pathGroup.get(0).getInitialHolonomicPose()), swerve),
      new FollowPath(pathGroup.get(0), swerve),
      new InstantCommand(() -> swerve.stop())
    );
  }
}
