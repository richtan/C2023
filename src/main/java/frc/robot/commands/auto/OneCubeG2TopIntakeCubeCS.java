package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.scoring.PositionIntake;
import frc.robot.commands.scoring.Stow;
import frc.robot.commands.scoring.PositionIntake.Position;
import frc.robot.commands.scoring.intake.Outtake;
import frc.robot.commands.scoring.intake.StartIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.util.PathLoader;

public class OneCubeG2TopIntakeCubeCS extends SequentialCommandGroup {
  public OneCubeG2TopIntakeCubeCS(Swerve swerve, Elevator elevator, Wrist wrist, Intake intake) {
    addRequirements(swerve, elevator, wrist, intake);
    
    List<PathPlannerTrajectory> pathGroup = PathLoader.getPathGroup("OneCubeG2TopIntakeCubeCS");

    addCommands(
      // new ResetToStartPose(swerve, pathGroup),
      // new PositionIntake(elevator, wrist, () -> false, Position.TOP),
      // new Outtake(intake, () -> false),
      // new Stow(intake, elevator, wrist, () -> false),
      // new FollowPath(pathGroup.get(0), swerve),
      // new PositionIntake(elevator, wrist, () -> false, Position.INTAKE),
      // new FollowPath(pathGroup.get(1), swerve).alongWith(new StartIntake(intake, () -> false).andThen(new Stow(intake, elevator, wrist, () -> false))),
      // new FollowPath(pathGroup.get(2), swerve),
      // new InstantCommand(() -> swerve.stop()),
      // new BalanceOnChargeStation(swerve)
    );
  }
}
