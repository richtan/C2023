package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class BalanceOnChargeStation extends CommandBase {
  private final Swerve m_swerve;

  private double balanceEffort;
  private double turningEffort;

  public BalanceOnChargeStation(Swerve swerve) {
    m_swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    turningEffort = 0;
    balanceEffort = (AutoConstants.kBalancedAngle - m_swerve.getPitch().getDegrees()) * AutoConstants.kBalanceKP;
    m_swerve.drive(new Translation2d(balanceEffort, 0), turningEffort, false, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_swerve.getPitch().getDegrees()) < 2;
  }
}
