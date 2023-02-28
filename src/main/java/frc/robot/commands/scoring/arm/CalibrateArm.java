package frc.robot.commands.scoring.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmMode;

public class CalibrateArm extends SequentialCommandGroup {
  public CalibrateArm(Arm arm) {
    addRequirements(arm);
    addCommands(
      new InstantCommand(() -> arm.calibrateEncoder()),
      new InstantCommand(() -> arm.setIsCalibrated()),
      new InstantCommand(() -> arm.setIdleMode(ArmConstants.kMotorIdleMode)),
      new InstantCommand(() -> arm.setMode(ArmMode.DISABLED))
    );
  }
}
