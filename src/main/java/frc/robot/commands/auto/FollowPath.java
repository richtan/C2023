package frc.robot.commands.auto;


import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class FollowPath extends PPSwerveControllerCommand {
  public FollowPath(PathPlannerTrajectory path, Swerve swerve) {
    this(path, swerve, true);
  }

  public FollowPath(PathPlannerTrajectory path, Swerve swerve, boolean useAllianceColor) {
    super(
      path,
      swerve::getPose, // Pose supplier
      SwerveConstants.kKinematics, // kinematics
      swerve.getXController(), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      swerve.getYController(), // Y controller (usually the same values as X controller)
      swerve.getRotationController(), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
      swerve::setModuleStates, // ChassisSpeeds consumer
      useAllianceColor,
      swerve // Requires this drive subsystem
    );
  } 

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    super.execute();
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return super.isFinished();
  }
}