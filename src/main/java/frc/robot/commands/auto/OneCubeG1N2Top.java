package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.intake.Outtake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.util.PathLoader;

public class OneCubeG1N2Top extends SequentialCommandGroup {
  public OneCubeG1N2Top(Swerve swerve, Elevator elevator, Arm arm, Intake intake) {
    addRequirements(swerve, elevator, arm, intake);
    
    List<PathPlannerTrajectory> pathGroup = PathLoader.getPathGroup("OneCubeG1N1Top");

    addCommands(
      new InstantCommand(() -> swerve.resetOdometry(pathGroup.get(0).getInitialHolonomicPose()), swerve),
      new PositionIntake(elevator, arm, intake::hasCone, Position.TOP),
      new Outtake(intake, elevator, arm, false),
      new FollowPath(pathGroup.get(0), swerve),
      new InstantCommand(() -> swerve.stop())
    );
  }
}
