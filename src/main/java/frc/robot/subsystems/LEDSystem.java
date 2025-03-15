// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utilities.PriorityMap;

public class LEDSystem extends SubsystemBase {
  private static final int pmwPort = 9;
  private static final int stripLength = 18;
  private static final double distancePerLED = 15;
  // private ElevatorSystem elevator;
  private PriorityMap<String, LEDPattern> patterns = new PriorityMap<String, LEDPattern>();
  AddressableLED prettyLights;
  AddressableLEDBuffer prettyLightsBuffer;

  /** Creates a new LEDSystem. */
  public LEDSystem() {

    // this.elevator = elevator;
    prettyLights = new AddressableLED(pmwPort);
    prettyLightsBuffer = new AddressableLEDBuffer(stripLength);

    prettyLights.setLength(stripLength);
    // sets the data? figure out what this does soon.
    prettyLights.start();
    // test pattern which makes the lights evil and 190 themed
    LEDPattern test = LEDPattern.solid(Color.kRed);
    // fun assabet-y pattern
    LEDPattern assabet = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGold, Color.kBlue);

    LEDPattern assabetScroll = assabet.scrollAtRelativeSpeed(Percent.per(Second).of(25))
        .atBrightness(Percent.of(20));
    // assabetScroll.atBrightness(Percent.of(0.2));
    // applies that test pattern to the strip
    // test.applyTo(prettyLightsBuffer);
    addPattern("Assabet Scroll", 157, assabetScroll);

  }

  public void addPattern(String name, int priority, LEDPattern pattern) {
    patterns.put(name, priority, pattern);
  }

  public LEDPattern removePattern(String name) {
    return patterns.remove(name);
  }

  @Override
  public void periodic() {

    patterns.firstValue().applyTo(prettyLightsBuffer);

    // This method will be called once per scheduler run

    // LEDPattern elevatorProgress = LEDPattern.progressMaskLayer(() ->
    // elevator.getScaledPos() / 1);
    prettyLights.setData(prettyLightsBuffer);
  }
}
