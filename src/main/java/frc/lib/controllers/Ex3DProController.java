package frc.lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

public class Ex3DProController {

  private final Joystick m_controller;

  public final JoystickButton B1, B2, B3, B4, B6, B7, B8, B9, B10, B11, B12;

  public Ex3DProController(int port) {
    m_controller = new Joystick(port);

    B1 = new JoystickButton(m_controller, 1);
    B2 = new JoystickButton(m_controller, 2);
    B3 = new JoystickButton(m_controller, 3);
    B4 = new JoystickButton(m_controller, 4);
    B6 = new JoystickButton(m_controller, 6);
    B7 = new JoystickButton(m_controller, 7);
    B8 = new JoystickButton(m_controller, 8);
    B9 = new JoystickButton(m_controller, 9);
    B10 = new JoystickButton(m_controller, 10);
    B11 = new JoystickButton(m_controller, 11);
    B12 = new JoystickButton(m_controller, 12);
  }

  public double X() { return m_controller.getRawAxis(0); }
  public double Y() { return m_controller.getRawAxis(1); }
  public double Z() { return m_controller.getRawAxis(2); }
  public double SLIDER() { return m_controller.getRawAxis(3); }
}