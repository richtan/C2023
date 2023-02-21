package frc.lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;


public class MadCatzController {

  private final Joystick m_controller;

  public final JoystickButton B1, B2, B3, B4, B6, B7;
  public final POVButton DPAD_UNPRESSED, DPAD_UP, DPAD_UP_RIGHT, DPAD_RIGHT, DPAD_DOWN_RIGHT, DPAD_DOWN, DPAD_DOWN_LEFT, DPAD_LEFT, DPAD_UP_LEFT;
  public final Trigger ALL_UP, ALL_DOWN, ALL_LEFT, ALL_RIGHT;

  public MadCatzController(int port) {
    m_controller = new Joystick(port);

    B1 = new JoystickButton(m_controller, 1);
    B2 = new JoystickButton(m_controller, 2);
    B3 = new JoystickButton(m_controller, 3);
    B4 = new JoystickButton(m_controller, 4);
    B6 = new JoystickButton(m_controller, 6);
    B7 = new JoystickButton(m_controller, 7);

    DPAD_UNPRESSED = new POVButton(m_controller, -1);
    DPAD_UP = new POVButton(m_controller, 0);
    DPAD_UP_RIGHT = new POVButton(m_controller, 45);
    DPAD_RIGHT = new POVButton(m_controller, 90);
    DPAD_DOWN_RIGHT = new POVButton(m_controller, 135);
    DPAD_DOWN = new POVButton(m_controller, 180);
    DPAD_DOWN_LEFT = new POVButton(m_controller, 235);
    DPAD_LEFT = new POVButton(m_controller, 270);
    DPAD_UP_LEFT = new POVButton(m_controller, 315);

    ALL_UP = new Trigger(DPAD_UP.or(DPAD_UP_LEFT).or(DPAD_UP_RIGHT));
    ALL_DOWN = new Trigger(DPAD_DOWN.or(DPAD_DOWN_LEFT).or(DPAD_DOWN_RIGHT));
    ALL_LEFT = new Trigger(DPAD_LEFT.or(DPAD_UP_LEFT).or(DPAD_DOWN_LEFT));
    ALL_RIGHT = new Trigger(DPAD_RIGHT.or(DPAD_UP_RIGHT).or(DPAD_DOWN_RIGHT));
  }

  public double X() { return m_controller.getRawAxis(0); }
  public double Y() { return m_controller.getRawAxis(1); }
  public double SLIDER() { return m_controller.getRawAxis(2); }
  public double ZROTATE() { return m_controller.getRawAxis(3); }
}
