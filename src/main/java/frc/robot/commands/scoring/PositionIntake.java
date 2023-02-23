package frc.robot.commands.scoring;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.scoring.arm.MoveArm;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class PositionIntake extends SequentialCommandGroup {
  public PositionIntake(Elevator elevator, Arm arm, BooleanSupplier isConeColorSupplier, Position position) {
    addRequirements(elevator, arm);
    addCommands(
      new SelectCommand(Map.ofEntries(
        Map.entry(Position.TOP, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kTopConeHeight).alongWith(new MoveArm(arm, ArmConstants.kTopConeAngle)),
          new MoveElevator(elevator, ElevatorConstants.kTopCubeHeight).alongWith(new MoveArm(arm, ArmConstants.kTopCubeAngle)),
          isConeColorSupplier
        )),
        Map.entry(Position.MIDDLE, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kMiddleConeHeight).alongWith(new MoveArm(arm, ArmConstants.kMiddleConeAngle)),
          new MoveElevator(elevator, ElevatorConstants.kMiddleCubeHeight).alongWith(new MoveArm(arm, ArmConstants.kMiddleCubeAngle)),
          isConeColorSupplier
        )),
        Map.entry(Position.BOTTOM, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kBottomConeHeight).alongWith(new MoveArm(arm, ArmConstants.kBottomConeAngle)),
          new MoveElevator(elevator, ElevatorConstants.kBottomCubeHeight).alongWith(new MoveArm(arm, ArmConstants.kBottomCubeAngle)),
          isConeColorSupplier
        )),
        Map.entry(Position.SHELF, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kShelfConeHeight).alongWith(new MoveArm(arm, ArmConstants.kShelfConeAngle)),
          new MoveElevator(elevator, ElevatorConstants.kShelfCubeHeight).alongWith(new MoveArm(arm, ArmConstants.kShelfCubeAngle)),
          isConeColorSupplier
        )),
        Map.entry(Position.INTAKE, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kIntakeConeHeight).alongWith(new MoveArm(arm, ArmConstants.kIntakeConeAngle)),
          new MoveElevator(elevator, ElevatorConstants.kIntakeCubeHeight).alongWith(new MoveArm(arm, ArmConstants.kIntakeCubeAngle)),
          isConeColorSupplier
        )),
        Map.entry(Position.STOW, 
          new MoveElevator(elevator, ElevatorConstants.kStowHeight).alongWith(new MoveArm(arm, ArmConstants.kStowAngle))
        )
      ), () -> position),
      new ParallelCommandGroup(
        new WaitUntilCommand(arm::reachedDesiredAngle),
        new WaitUntilCommand(elevator::reachedDesiredPosition)
      )
    );
  }

  public enum Position {
    TOP, MIDDLE, BOTTOM, SHELF, INTAKE, STOW
  }
}
