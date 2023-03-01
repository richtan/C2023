package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class BalanceOnChargeStation extends CommandBase {
  private final Swerve m_swerve;

  private double balanceEffort;
  private double turningEffort;

  private final PIDController m_balancePID = new PIDController(AutoConstants.kBalanceKP, AutoConstants.kBalanceKI, AutoConstants.kBalanceKD);

  public BalanceOnChargeStation(Swerve swerve) {
    m_swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    m_balancePID.setSetpoint(0);
    m_swerve.setIsBalancingOnChargeStation(true);
  }

  @Override
  public void execute() {
    turningEffort = 0;
    balanceEffort = m_balancePID.calculate(m_swerve.getPitch().getDegrees());
    m_swerve.drive(new Translation2d(balanceEffort, 0), turningEffort, false, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_swerve.stop();
    m_swerve.setIsBalancingOnChargeStation(false);
  }

  @Override
  public boolean isFinished() {
    return DriverStation.isAutonomousEnabled() ? false : Math.abs(m_swerve.getPitch().getDegrees()) < 2;
  }
}
