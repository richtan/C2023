package frc.robot.commands.scoring;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.scoring.wrist.MoveWrist;
import frc.robot.commands.scoring.elevator.MoveElevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator;

public class PositionIntake extends SequentialCommandGroup {
  private final Elevator m_elevator;
  private final Wrist m_wrist;

  public PositionIntake(Elevator elevator, Wrist wrist, BooleanSupplier isConeSup, Position position) {
    m_elevator = elevator;
    m_wrist = wrist;

    addRequirements(elevator, wrist);
    addCommands(
      new SelectCommand(Map.ofEntries(
        Map.entry(Position.TOP, new ConditionalCommand(
          positionIntake(ElevatorConstants.kTopConeHeight, WristConstants.kTopConeAngle),
          positionIntake(ElevatorConstants.kTopCubeHeight, WristConstants.kTopCubeAngle),
          isConeSup
        )),
        Map.entry(Position.MIDDLE, new ConditionalCommand(
          positionIntake(ElevatorConstants.kMiddleConeHeight, WristConstants.kMiddleConeAngle),
          positionIntake(ElevatorConstants.kMiddleCubeHeight, WristConstants.kMiddleCubeAngle),
          isConeSup
        )),
        Map.entry(Position.BOTTOM, new ConditionalCommand(
          positionIntake(ElevatorConstants.kBottomConeHeight, WristConstants.kBottomConeAngle),
          positionIntake(ElevatorConstants.kBottomCubeHeight, WristConstants.kBottomCubeAngle),
          isConeSup
        )),
        Map.entry(Position.SHELF, new ConditionalCommand(
          positionIntake(ElevatorConstants.kShelfConeHeight, WristConstants.kShelfConeAngle),
          positionIntake(ElevatorConstants.kShelfCubeHeight, WristConstants.kShelfCubeAngle),
          isConeSup
        )),
        Map.entry(Position.INTAKE, new ConditionalCommand(
          positionIntake(ElevatorConstants.kIntakeConeHeight, WristConstants.kIntakeConeAngle),
          positionIntake(ElevatorConstants.kIntakeCubeHeight, WristConstants.kIntakeCubeAngle),
          isConeSup
        )),
        Map.entry(Position.STOW, 
          positionIntake(ElevatorConstants.kStowHeight, WristConstants.kStowAngle)
        )
      ), () -> position),
      new ParallelCommandGroup(
        new WaitUntilCommand(wrist::reachedDesiredAngle),
        new WaitUntilCommand(elevator::reachedDesiredPosition)
      )
    );
  }

  public PositionIntake(Elevator elevator, Wrist wrist, double elevatorHeight, double wristAngle) {
    m_elevator = elevator;
    m_wrist = wrist;

    addRequirements(elevator, wrist);
    addCommands(
      positionIntake(elevatorHeight, wristAngle)
    );
  }

  private Command positionIntake(double elevatorHeight, double wristAngle) {
    return Commands.parallel(
      new MoveElevator(m_elevator, elevatorHeight),
      new MoveWrist(m_wrist, wristAngle)
    );
  }

  public enum Position {
    TOP, MIDDLE, BOTTOM, SHELF, INTAKE, STOW
  }
}
