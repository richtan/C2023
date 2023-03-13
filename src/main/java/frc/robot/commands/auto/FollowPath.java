package frc.robot.commands.auto;


import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.PathLoader;

public class FollowPath extends SequentialCommandGroup {
  public FollowPath(PathPlannerTrajectory path, Swerve swerve) {
    addRequirements(swerve);
    addCommands(
      new InstantCommand(() -> 
        new PPSwerveControllerCommand(
          PathLoader.transformTrajectoryForAlliance(path, DriverStation.getAlliance()),
          swerve::getPose, // Pose supplier
          SwerveConstants.kKinematics, // kinematics
          swerve.getXController(), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          swerve.getYController(), // Y controller (usually the same values as X controller)
          swerve.getRotationController(), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          swerve::setModuleStates, // ChassisSpeeds consumer
          false,
          swerve // Requires this drive subsystem
        ).schedule()
      )
    );
  } 
}