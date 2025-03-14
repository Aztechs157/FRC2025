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

public class LEDSystem extends SubsystemBase {
  private static final int pmwPort = 9;
  private static final int stripLength = 18;
  private ElevatorSystem elevator;
  AddressableLED prettyLights;
  AddressableLEDBuffer prettyLightsBuffer;
    
  
    /** Creates a new LEDSystem. */
    public LEDSystem(ElevatorSystem elevator) {
    
      this.elevator = elevator;
      prettyLights = new AddressableLED(pmwPort);
    prettyLightsBuffer = new AddressableLEDBuffer(stripLength);
    
    prettyLights.setLength(stripLength);
    // sets the data? figure out what this does soon.
    prettyLights.start();
    // test pattern which makes the lights evil and 190 themed
    LEDPattern test = LEDPattern.solid(Color.kRed);
    // applies that test pattern to the strip
    // test.applyTo(prettyLightsBuffer);

  }

  @Override
  public void periodic() {
    // fun assabet-y pattern
    LEDPattern assabet = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGold, Color.kBlue, Color.kRed);

    LEDPattern assabetScroll = assabet.scrollAtRelativeSpeed(Percent.per(Second).of(25));
    assabetScroll.applyTo(prettyLightsBuffer);
   
    // This method will be called once per scheduler run

    // LEDPattern elevatorProgress = LEDPattern.progressMaskLayer(() -> elevator.getScaledPos() / 1);
    prettyLights.setData(prettyLightsBuffer);
  }
}
