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

public class PositionMechanisms extends SequentialCommandGroup {
  public PositionMechanisms(Elevator elevator, Arm arm, BooleanSupplier isConeColorSupplier, Position row) {
    addRequirements(elevator, arm);
    addCommands(
      new SelectCommand(Map.ofEntries(
        Map.entry(Position.TOP, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kTopConeHeight).alongWith(new MoveArm(arm, ArmConstants.kTopConePosition)),
          new MoveElevator(elevator, ElevatorConstants.kTopCubeHeight).alongWith(new MoveArm(arm, ArmConstants.kTopCubePosition)),
          isConeColorSupplier
        )),
        Map.entry(Position.MIDDLE, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kMiddleConeHeight).alongWith(new MoveArm(arm, ArmConstants.kMiddleConePosition)),
          new MoveElevator(elevator, ElevatorConstants.kMiddleCubeHeight).alongWith(new MoveArm(arm, ArmConstants.kMiddleCubePosition)),
          isConeColorSupplier
        )),
        Map.entry(Position.BOTTOM, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kBottomConeHeight).alongWith(new MoveArm(arm, ArmConstants.kBottomConePosition)),
          new MoveElevator(elevator, ElevatorConstants.kBottomCubeHeight).alongWith(new MoveArm(arm, ArmConstants.kBottomCubePosition)),
          isConeColorSupplier
        )),
        Map.entry(Position.SHELF, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kShelfConeHeight).alongWith(new MoveArm(arm, ArmConstants.kShelfConePosition)),
          new MoveElevator(elevator, ElevatorConstants.kShelfCubeHeight).alongWith(new MoveArm(arm, ArmConstants.kShelfCubePosition)),
          isConeColorSupplier
        )),
        Map.entry(Position.INTAKE, new ConditionalCommand(
          new MoveElevator(elevator, ElevatorConstants.kIntakeConeHeight).alongWith(new MoveArm(arm, ArmConstants.kIntakeConePosition)),
          new MoveElevator(elevator, ElevatorConstants.kIntakeCubeHeight).alongWith(new MoveArm(arm, ArmConstants.kIntakeCubePosition)),
          isConeColorSupplier
        ))
      ), () -> row),
      new ParallelCommandGroup(
        new WaitUntilCommand(arm::reachedDesiredPosition),
        new WaitUntilCommand(elevator::reachedDesiredHeight)
      )
    );
  }

  public enum Position {
    TOP, MIDDLE, BOTTOM, SHELF, INTAKE
  }
}
