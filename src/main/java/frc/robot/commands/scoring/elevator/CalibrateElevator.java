package frc.robot.commands.scoring.elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorMode;

public class CalibrateElevator extends CommandBase {
  private final Elevator m_elevator;

  public CalibrateElevator(Elevator elevator) {
    m_elevator = elevator;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    m_elevator.setMode(ElevatorMode.CALIBRATION);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    m_elevator.zeroEncoder();
    m_elevator.toggleSoftLimits(true);
    m_elevator.setDesiredHeight(0);
    m_elevator.setMode(ElevatorMode.MOTIONMAGIC);
  }

  @Override
  public boolean isFinished() {
    return m_elevator.isBottomLimitSwitchReached();
  }
}
