package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothing;
import frc.robot.subsystems.Swerve;
import frc.robot.util.PathLoader;

public class FollowPath extends SequentialCommandGroup {
  public FollowPath(String pathGroupName, int pathIndex, Swerve swerve){
    this(PathLoader.getPathGroup(pathGroupName), pathIndex, swerve);
  }

  public FollowPath(List<PathPlannerTrajectory> pathGroup, int pathIndex, Swerve swerve){
    addRequirements(swerve);

    PathPlannerTrajectory path = PathLoader.getPath(pathGroup, pathIndex);
    
    addCommands(
      (pathIndex == 0 ? new InstantCommand(() -> swerve.resetOdometry(path.getInitialHolonomicPose()), swerve) : new DoNothing()),
      new PPSwerveControllerCommand(
        path,
        swerve::getPose, // Pose supplier
        swerve.getXController(), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        swerve.getYController(), // Y controller (usually the same values as X controller)
        swerve.getRotationController(), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        swerve::setChassisSpeeds, // ChassisSpeeds consumer
        true,
        swerve // Requires this drive subsystem
      )
    );
  } 
}