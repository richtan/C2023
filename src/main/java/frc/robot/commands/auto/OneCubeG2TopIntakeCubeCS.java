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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.util.PathLoader;

public class OneCubeG2TopIntakeCubeCS extends SequentialCommandGroup {
  public OneCubeG2TopIntakeCubeCS(Swerve swerve, Elevator elevator, Arm arm, Intake intake) {
    addRequirements(swerve, elevator, arm, intake);
    
    List<PathPlannerTrajectory> pathGroup = PathLoader.getPathGroup("OneCubeG2TopIntakeCubeCS");

    addCommands(
      new InstantCommand(() -> swerve.resetOdometry(pathGroup.get(0).getInitialHolonomicPose()), swerve),
      new PositionIntake(elevator, arm, intake::hasCone, Position.TOP),
      new Outtake(intake, elevator, arm, false),
      new FollowPath(pathGroup.get(0), swerve).alongWith(new Stow(intake, elevator, arm)),
      new PositionIntake(elevator, arm, intake::hasCone, Position.INTAKE),
      new FollowPath(pathGroup.get(1), swerve).alongWith(new StartIntake(intake, elevator, arm).andThen(new Stow(intake, elevator, arm))),
      new FollowPath(pathGroup.get(2), swerve),
      new InstantCommand(() -> swerve.stop()),
      new BalanceOnChargeStation(swerve)
    );
  }
}