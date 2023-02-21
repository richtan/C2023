package frc.lib.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PistolController {
  private final Joystick m_controller;

  public final JoystickButton TOP_BACK, TOP_FRONT, BOTTOM_FRONT, BOTTOM_BACK, BOTTOM;
  public final Trigger TOP_BACK_ONLY, TOP_FRONT_ONLY, BOTTOM_BACK_ONLY, BOTTOM_FRONT_ONLY;

  public PistolController(int port) {
    m_controller = new Joystick(port);

    TOP_BACK = new JoystickButton(m_controller, 1);
    TOP_FRONT = new JoystickButton(m_controller, 2);
    BOTTOM_FRONT = new JoystickButton(m_controller, 3);
    BOTTOM_BACK = new JoystickButton(m_controller, 4);
    BOTTOM = new JoystickButton(m_controller, 5);

    TOP_BACK_ONLY = new Trigger(TOP_BACK.and(TOP_FRONT.negate()));
    TOP_FRONT_ONLY = new Trigger(TOP_FRONT.and(TOP_BACK.negate()));
    BOTTOM_BACK_ONLY = new Trigger(BOTTOM_BACK.and(BOTTOM_FRONT.negate()));
    BOTTOM_FRONT_ONLY = new Trigger(BOTTOM_FRONT.and(BOTTOM_BACK.negate()));
  }

  public double WHEEL() { return m_controller.getRawAxis(0); }
  public double TRIGGER() { return m_controller.getRawAxis(1); }
}
