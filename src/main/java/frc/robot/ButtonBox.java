// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/** Add your docs here. */
public class ButtonBox extends GenericHID implements Sendable {
/** Represents a digital button on a ButtonBox. */
  public enum Button {
    /** A button. */
    kA(1),
    /** B button. */
    kB(2),
    /** X button. */
    kC(3),
    /** Y button. */
    kD(4),
    /** Left bumper button. */
    kLeftBumper(5),
    /** Right bumper button. */
    kRightBumper(6),
    /** Back button. */
    kBack(7),
    /** Start button. */
    kStart(8),
    /** Left stick button. */
    kLeftStick(9),
    /** Right stick button. */
    kRightStick(10);

    /** Button value. */
    public final int value;

    Button(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the button, matching the relevant methods. This is done by
     * stripping the leading `k`, and appending `Button`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the button.
     */
    @Override
    public String toString() {
      // Remove leading `k`
      return this.name().substring(1) + "Button";
    }
  }

/**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into (0-5).
   */
  public ButtonBox(final int port) {
    super(port);
    HAL.report(tResourceType.kResourceType_Controller, port + 1); // TODO: find proper resource type
  }

  /**
   * Read the value of the A button on the controller.
   *
   * @return The state of the button.
   */
  public boolean getAButton() {
    return getRawButton(Button.kA.value);
  }

  /**
   * Whether the A button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean getAButtonPressed() {
    return getRawButtonPressed(Button.kA.value);
  }

  /**
   * Whether the A button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean getAButtonReleased() {
    return getRawButtonReleased(Button.kA.value);
  }

  /**
   * Constructs an event instance around the A button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent a(EventLoop loop) {
    return button(Button.kA.value, loop);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("HID");
    builder.publishConstString("ControllerType", "Button Box");
    builder.addBooleanProperty("A", this::getAButton, null);
  }

}
