// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utilities.PriorityMap;

@Logged(strategy = Strategy.OPT_OUT)
public class LEDSystem extends SubsystemBase {
  private static final int pmwPort = 9;
  private static final int stripLength = 18;
  private static final double distancePerLED = 15;
  // private ElevatorSystem elevator;
  private PriorityMap<String, LEDPattern> fullPatterns = new PriorityMap<String, LEDPattern>();
  private PriorityMap<String, LEDPattern> topPatterns = new PriorityMap<String, LEDPattern>();
  private PriorityMap<String, LEDPattern> midPatterns = new PriorityMap<String, LEDPattern>();
  private PriorityMap<String, LEDPattern> botPatterns = new PriorityMap<String, LEDPattern>();

  AddressableLED prettyLights;
  AddressableLEDBuffer prettyLightsBuffer;

  AddressableLEDBufferView topBuffer;
  AddressableLEDBufferView midBuffer;
  AddressableLEDBufferView botBuffer;

  /** Creates a new LEDSystem. */
  public LEDSystem() {

    // this.elevator = elevator;
    prettyLights = new AddressableLED(pmwPort);
    prettyLightsBuffer = new AddressableLEDBuffer(stripLength);

    topBuffer = prettyLightsBuffer.createView(12, 17);
    midBuffer = prettyLightsBuffer.createView(6, 11);
    botBuffer = prettyLightsBuffer.createView(5, 0);

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

  public void isFMS() {
    LEDPattern assabet = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGold, Color.kBlue);

    LEDPattern assabetScroll = assabet.scrollAtRelativeSpeed(Percent.per(Second).of(25));

    fullPatterns.replace("Assabet Scroll", assabetScroll);
  }

  // full
  public void addPattern(String name, int priority, LEDPattern pattern) {
    fullPatterns.put(name, priority, pattern);
  }

  public LEDPattern removePattern(String name) {
    return fullPatterns.remove(name);
  }

  public boolean hasPattern(String name) {
    return fullPatterns.containsKey(name);
  }

  // bot
  public void addBotPattern(String name, int priority, LEDPattern pattern) {
    botPatterns.put(name, priority, pattern);
  }

  public LEDPattern removeBotPattern(String name) {
    return botPatterns.remove(name);
  }

  public boolean hasBotPattern(String name) {
    return botPatterns.containsKey(name);
  }

  // mid
  public void addMidPattern(String name, int priority, LEDPattern pattern) {
    midPatterns.put(name, priority, pattern);
  }

  public LEDPattern removeMidPattern(String name) {
    return midPatterns.remove(name);
  }

  public boolean hasMidPattern(String name) {
    return midPatterns.containsKey(name);
  }

  // top
  public void addTopPattern(String name, int priority, LEDPattern pattern) {
    topPatterns.put(name, priority, pattern);
  }

  public LEDPattern removeTopPattern(String name) {
    return topPatterns.remove(name);
  }

  public boolean hasTopPattern(String name) {
    return topPatterns.containsKey(name);
  }

  @Override
  public void periodic() {

    fullPatterns.firstValue().applyTo(prettyLightsBuffer);

    if (botPatterns.firstPriority() != null && botPatterns.firstPriority() <= fullPatterns.firstPriority()) {
      botPatterns.firstValue().applyTo(botBuffer);
    }

    if (midPatterns.firstPriority() != null && midPatterns.firstPriority() <= fullPatterns.firstPriority()) {
      midPatterns.firstValue().applyTo(midBuffer);
    }

    if (topPatterns.firstPriority() != null && topPatterns.firstPriority() <= fullPatterns.firstPriority()) {
      topPatterns.firstValue().applyTo(topBuffer);
    }

    // if (!useBotPattern && !useTopPattern) {
    // fullPatterns.firstValue().applyTo(prettyLightsBuffer);
    // } else if (useBotPattern && !useTopPattern) {
    // fullPatterns.firstValue().applyTo(topBuffer);
    // } else if (useTopPattern && !useBotPattern) {
    // fullPatterns.firstValue().applyTo(botBuffer);
    // }
    // This method will be called once per scheduler run

    // LEDPattern elevatorProgress = LEDPattern.progressMaskLayer(() ->
    // elevator.getScaledPos() / 1);
    prettyLights.setData(prettyLightsBuffer);
  }
}
