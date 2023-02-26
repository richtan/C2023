package frc.robot.util;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.PowerConstants;

public class PowerManager {
  private final ShuffleboardTab m_powerTab;

  private final PowerDistribution m_PDModule;

  public PowerManager(ShuffleboardTab powerTab) {
    m_powerTab = powerTab;

    m_PDModule = new PowerDistribution(PowerConstants.kPDModuleID, PowerConstants.kPDModuleType);

    setupShuffleboard();
  }

  private void setupShuffleboard() {
    m_powerTab.addDouble("Total Current (A)", m_PDModule::getTotalCurrent);
    m_powerTab.addDouble("Elevator current (A)", () -> m_PDModule.getCurrent(7));
  }
}
