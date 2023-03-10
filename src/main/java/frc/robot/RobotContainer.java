// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.controllers.GameController;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DoNothing;
import frc.robot.commands.auto.BalanceOnChargeStation;
import frc.robot.commands.auto.MobilityAuto;
import frc.robot.commands.auto.OneCubeG2Top;
import frc.robot.commands.auto.OneCubeG2TopIntakeCubeCS;
import frc.robot.commands.swerve.TeleopDrive;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.util.PathLoader;
import frc.robot.util.PowerManager;
import frc.robot.util.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> m_autoCommand = new SendableChooser<>();

  // Controllers
  private final GameController m_driverJoy;
  private final GameController m_operatorJoy;
  private final GameController m_manualJoy;
  private final GameController m_testJoy;

  private final ShuffleboardTab m_swerveTab = Shuffleboard.getTab("Swerve");
  private final ShuffleboardTab m_autoTab = Shuffleboard.getTab("Auto");
  private final ShuffleboardTab m_elevatorTab = Shuffleboard.getTab("Elevator");
  private final ShuffleboardTab m_intakeTab = Shuffleboard.getTab("Intake");
  private final ShuffleboardTab m_barTab = Shuffleboard.getTab("Bar");
  private final ShuffleboardTab m_wristTab = Shuffleboard.getTab("Wrist");
  private final ShuffleboardTab m_visionTab = Shuffleboard.getTab("Vision");
  private final ShuffleboardTab m_powerTab = Shuffleboard.getTab("Power");

  private final Vision m_vision;
  private final PowerManager m_powerManager;

  // Subsystems
  private final Swerve m_swerve;
  private final Elevator m_elevator;
  private final Intake m_intake;
  private final Wrist m_wrist;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Load auto path groups
    PathLoader.loadPathGroups();

    // Disable LiveWindow to reduce loop overruns
    LiveWindow.disableAllTelemetry();
    LiveWindow.setEnabled(false);

    // Don't flood console with warnings about missing controllers
    DriverStation.silenceJoystickConnectionWarning(true);

    m_driverJoy = new GameController(Constants.OIConstants.kDriverJoy);
    m_operatorJoy = new GameController(Constants.OIConstants.kOperatorJoy);
    m_manualJoy = new GameController(Constants.OIConstants.kManualJoy);
    m_testJoy = new GameController(Constants.OIConstants.kTestJoy);

    // Create vision util instance with cameras
    m_vision = new Vision(VisionConstants.kCameras, m_visionTab);

    // Create current manager
    m_powerManager = new PowerManager(m_powerTab);

    // Initialize subsystems
    m_swerve = new Swerve(m_vision, m_swerveTab);

    // Setup driver controls
    OI.configureDriverControls(m_driverJoy, m_swerve);

    if (Constants.kIsComp) {
      System.out.println("Running COMPETITION robot");

      m_intake = new Intake(m_intakeTab);
      // m_intake = null;
      m_elevator = new Elevator(m_elevatorTab, m_operatorJoy.RT);
      // m_elevator = new Elevator(m_elevatorTab, () -> false);
      // m_elevator = null;

      m_wrist = new Wrist(m_wristTab);

      // Setup compbot-only controls
      OI.configureOperatorControls(m_operatorJoy, m_elevator, m_wrist, m_intake);
      OI.configureManualControls(m_manualJoy, m_elevator, m_wrist, m_intake);
      // OI.configureTestControls(m_testJoy, m_swerve, m_elevator, m_wrist, m_intake);
    } else {
      System.out.println("Running TEST robot");

      // These subsystems don't exist
      m_elevator = null;
      m_intake = null;
      // m_intake = new Intake(m_intakeTab);
      m_wrist = null;
    }

    setupSchedulerShuffleboard();

    // Start RoboRIO driver camera stream
    CameraServer.startAutomaticCapture();

    // Create auto chooser and add auto command options
    setupAutoChooser();

    m_swerve.setDefaultCommand(new TeleopDrive(
      m_swerve,
      () -> m_driverJoy.LEFT_Y(),
      () -> m_driverJoy.LEFT_X(),
      () -> m_driverJoy.RIGHT_X(),
      m_driverJoy.LT,
      m_driverJoy.RT,
      m_driverJoy.LB
      // () -> false
    ));
  }

  public void calibrateMechanisms() {
    m_swerve.resetModulesToAbsolute();
    if (Constants.kIsComp) {
      m_wrist.calibrateEncoder();
    }
  }

  public void setupAutoChooser() {
    // Add auto commands here
    m_autoCommand.setDefaultOption("Do Nothing", new DoNothing());
    m_autoCommand.addOption("Balance on Charge Station", new BalanceOnChargeStation(m_swerve));
    m_autoCommand.addOption("MobilityAuto", new MobilityAuto(m_swerve));

    if (Constants.kIsComp) {
      m_autoCommand.addOption("One Cube G1 Top", new OneCubeG2Top(m_swerve, m_elevator, m_wrist, m_intake));
      m_autoCommand.addOption("One Cube G1 Top CS", new OneCubeG2TopIntakeCubeCS(m_swerve, m_elevator, m_wrist, m_intake));
      m_autoCommand.addOption("One Cube G1 Top Intake Cube CS", new OneCubeG2TopIntakeCubeCS(m_swerve, m_elevator, m_wrist, m_intake));
    }

    // Add auto chooser to Shuffleboard tab
    m_autoTab.add("Auto Chooser", m_autoCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoCommand.getSelected();
  }

  /**
   * Loads the command scheduler shuffleboard which will add event markers whenever a command finishes, ends, or is interrupted.
   */
  public void setupSchedulerShuffleboard() {
    if (Constants.kUseTelemetry) {
      // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish
      CommandScheduler.getInstance().onCommandInitialize(command -> Shuffleboard.addEventMarker("Command initialized", command.getName(), EventImportance.kNormal));
      CommandScheduler.getInstance().onCommandInterrupt(command -> Shuffleboard.addEventMarker("Command interrupted", command.getName(), EventImportance.kNormal));
      CommandScheduler.getInstance().onCommandFinish(command -> Shuffleboard.addEventMarker("Command finished", command.getName(), EventImportance.kNormal));
    }
  }
}
