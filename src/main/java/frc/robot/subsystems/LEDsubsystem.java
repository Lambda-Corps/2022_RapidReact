// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDsubsystem extends SubsystemBase {
  public static enum LED_State{
    KLED_ZERO,
    kLED_ONE_BALL,
    KLED_TWO_BALLS,
    kLED_OFF,
    KLED_DEFAULT;
  }
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer1;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;
  
  // Intake State Collector
  private NetworkTableEntry m_LEDSNetworkTableEntry;
  /** Creates a new LEDsubsystem. */
  public LEDsubsystem() { // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_LEDSNetworkTableEntry = Shuffleboard.getTab("LEDs").add("Intake", 0)
                        .withSize(1, 1)
                        .getEntry();
    m_led = new AddressableLED(9);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer1 = new AddressableLEDBuffer(90);
    m_led.setLength(m_ledBuffer1.getLength());

    // Set the data
    m_led.setData(m_ledBuffer1);
    m_led.start();
    
    
  }

  

  @Override
  public void periodic() {
    double led_color = m_LEDSNetworkTableEntry.getDouble(0);
  
      if (led_color == LED_State.kLED_ONE_BALL.ordinal()) {
      blue();
    } else if (led_color == LED_State.KLED_TWO_BALLS.ordinal()) {
      green(); 
    } else if (led_color == LED_State.KLED_ZERO.ordinal()) {
      red();
    }
    else {rainbow();}
    // This method will be called once per scheduler run
    m_LEDSNetworkTableEntry.setDefaultNumber(0);
  }

  @Override
  public void simulationPeriodic() {

    // This method will be called once per scheduler run during simulation
  }

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer1.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer1.getLength())) % 180;
      // final var hue = 90;
      // Set the value
      m_ledBuffer1.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    // System.out.println("==============================================");
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer1);
  }
  
  public void red() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer1.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      // final var hue = (m_rainbowFirstPixelHue + (i * 180 /
      // m_ledBuffer1.getLength())) % 180;
      final var hue = 0;
      // System.out.println("hello");;
      // Set the value
      m_ledBuffer1.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    // System.out.println("==============================================");
    m_led.setData(m_ledBuffer1);
  }

  public void blue() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer1.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      // final var hue = (m_rainbowFirstPixelHue + (i * 180 /
      // m_ledBuffer1.getLength())) % 180;
      final var hue = 120;
      // System.out.println("hello");;
      // Set the value
      m_ledBuffer1.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    // System.out.println("==============================================");
    m_led.setData(m_ledBuffer1);
  }

  public void green() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer1.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      // final var hue = (m_rainbowFirstPixelHue + (i * 180 /
      // m_ledBuffer1.getLength())) % 180;
      final var hue = 40;
      // System.out.println("hello");;
      // Set the value
      m_ledBuffer1.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    // System.out.println("==============================================");
    m_led.setData(m_ledBuffer1);
  }

  public void purple() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer1.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      // final var hue = (m_rainbowFirstPixelHue + (i * 180 /
      // m_ledBuffer1.getLength())) % 180;
      final var hue = 340;
      // System.out.println("hello");;
      // Set the value
      m_ledBuffer1.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    // m_rainbowFirstPixelHue %= 180;
    // System.out.println("==============================================");
    m_led.setData(m_ledBuffer1);
  }
}
