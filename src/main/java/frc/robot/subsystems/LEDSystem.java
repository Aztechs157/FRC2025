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
import frc.robot.Constants.LEDConstants;

@Logged(strategy = Strategy.OPT_OUT)
public class LEDSystem extends SubsystemBase {
  private PriorityMap<String, LEDPattern> fullPatterns = new PriorityMap<String, LEDPattern>();
  private PriorityMap<String, LEDPattern> topPatterns = new PriorityMap<String, LEDPattern>();
  private PriorityMap<String, LEDPattern> botPatterns = new PriorityMap<String, LEDPattern>();

  AddressableLED prettyLights;
  AddressableLEDBuffer prettyLightsBuffer;

  AddressableLEDBufferView topBuffer;
  AddressableLEDBufferView botBuffer;

  /** Creates a new LEDSystem. */
  public LEDSystem() {

    prettyLights = new AddressableLED(LEDConstants.PMW_PORT);
    prettyLightsBuffer = new AddressableLEDBuffer(LEDConstants.STRIP_LENGTH);

    topBuffer = prettyLightsBuffer.createView(9, 17);
    botBuffer = prettyLightsBuffer.createView(0, 8);

    prettyLights.setLength(LEDConstants.STRIP_LENGTH);

    prettyLights.start();
    // test pattern which makes the lights evil and 190 themed
    LEDPattern test = LEDPattern.solid(Color.kRed);
    // fun assabet-y pattern
    LEDPattern assabet = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGold, Color.kBlue);
    // the same one but scrolling
    LEDPattern assabetScroll = assabet.scrollAtRelativeSpeed(Percent.per(Second).of(25))
        .atBrightness(Percent.of(20));
    addPattern("Assabet Scroll", 157, assabetScroll);

  }

  public void isFMS() {
    LEDPattern assabet = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGold, Color.kBlue);
    // runs the pattern without lowering the brightness, only when connected to an FMS.
    LEDPattern assabetScroll = assabet.scrollAtRelativeSpeed(Percent.per(Second).of(25));

    fullPatterns.replace("Assabet Scroll", assabetScroll);
  }

  /* for now, im using the same method as the fms checker, 
   * but i would like to know why we're making a method 
   * instead of just using DriverStation's isFMSAttached.
   */
  public void isEStop(){
    LEDPattern unpleasant = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kSpringGreen, Color.kMagenta, Color.kSaddleBrown);
    // hopefully runs the pattern in place of the default scrolly pattern
    fullPatterns.replace("Assabet Scroll", unpleasant);
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

    if (topPatterns.firstPriority() != null && topPatterns.firstPriority() <= fullPatterns.firstPriority()) {
      topPatterns.firstValue().applyTo(topBuffer);
    }

    prettyLights.setData(prettyLightsBuffer);
  }
}
