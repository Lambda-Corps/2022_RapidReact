// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDsubsystem extends SubsystemBase {
  // No enum, but use the same construct
  public static final int LED_DEFAULT       = 0;
  public static final int LED_ONE_BALL      = 1;
  public static final int LED_TWO_BALL      = 2;
  public static final int LED_CLIMBER_DONE  = 3;
  public static final int LED_HIGH_TARGET   = 4;
  public static final int LED_CARGO_TARGET  = 5;
  public static final int LED_SHOOTER_READY = 6;
  public static final int LED_SHOOT_CHASE   = 7;

  private final int ALLIANCE_COLOR_BLUE = 0;
  private final int ALLIANCE_COLOR_RED = 1;

  private int m_alliance_color;
  
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  // Store what the last hue of the first pixel is
  private int m_rainbowFirstPixelHue;


  private int m_loopcount;
  private int m_LEDPoint;
  
  private int LEFT_SIDE_COUNT = 36, MIDDLE_COUNT = 14, RIGHT_SIDE_COUNT = 36; //86
  private int TOTAL_LED_COUNT = LEFT_SIDE_COUNT + RIGHT_SIDE_COUNT + MIDDLE_COUNT;

  private boolean climberLEDHighBar = false; // Actually the mid bar
  private boolean climberLEDLowBar = false;

  private boolean climberFinishedClimb = false;
  private boolean climbInProgress = false;

  // Intake State Collector
  private NetworkTableEntry m_LEDSNetworkTableEntry;
  /** Creates a new LEDsubsystem. */
  public LEDsubsystem() { // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_LEDSNetworkTableEntry = NetworkTableInstance.getDefault().getTable("Shuffleboard")
                                                               .getSubTable("Drive").getEntry("Ball Count Test");

    m_led = new AddressableLED(9);
    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(TOTAL_LED_COUNT);
    m_led.setLength(m_ledBuffer.getLength());


    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();

  }

  @Override
  public void periodic() {

    if( DriverStation.getAlliance() == Alliance.Blue){
      m_alliance_color = ALLIANCE_COLOR_BLUE;
    }
    else {
      m_alliance_color = ALLIANCE_COLOR_RED;
    }
    int led_color_state = (int)m_LEDSNetworkTableEntry.getDouble(0);
    
    switch(led_color_state){
      case LED_ONE_BALL:
          bluechase();
          break;
      case LED_TWO_BALL:
          break;
      case LED_CLIMBER_DONE:
          rainbow();
          break;
      case LED_HIGH_TARGET:
          break;
      case LED_CARGO_TARGET:
          break;
      case LED_SHOOTER_READY:
          break;
      case LED_SHOOT_CHASE:
          break;
      case LED_DEFAULT:
      default:
        if(m_alliance_color == ALLIANCE_COLOR_BLUE){
          bluechase();
        }
        else{
          redchase();
        }
        break;
    }
    if (climberLEDHighBar) {
      green();
    } else if (climberLEDLowBar) {
      green();
    }
    if (climberFinishedClimb) {
      rainbow();
    }
    m_loopcount =  m_loopcount + 1;
    if (m_loopcount > 258){
      m_loopcount = 0;
      m_LEDPoint = 0;
      
      if (!climberLEDHighBar || !climberLEDLowBar) {
        if (m_alliance_color == ALLIANCE_COLOR_BLUE) {
          bluechase();
        } else {
          redchase();
        }
      }
      //System.out.print("Reached reset " + m_loopcount);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // final var hue = 90;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    // System.out.println("==============================================");
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }
  
  public void red() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Set the value
      m_ledBuffer.setHSV(i, 0, 255, 128);
    }
    m_led.setData(m_ledBuffer);
  }

  public void blue() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Set the value
      m_ledBuffer.setHSV(i, 120, 255, 128);
    }
    m_led.setData(m_ledBuffer);
  }

  public void green() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Set the value
      m_ledBuffer.setHSV(i, 40, 255, 128);
    }
    m_led.setData(m_ledBuffer);
  }

  public void purple() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Set the value
      m_ledBuffer.setHSV(i, 40, 255, 128);
    }
    m_led.setData(m_ledBuffer);
  }
  public void bluechase() {
    // m_loopcount = 0;
    // System.out.print("Reached here " + m_loopcount);
     if(m_loopcount %5 == 0){
      for (var i = (m_LEDPoint); i < m_ledBuffer.getLength(); i += 86) {
        m_ledBuffer.setHSV((TOTAL_LED_COUNT -1) - i, 120, 255, 128);
        m_ledBuffer.setHSV(i, 120, 255, 128);
        m_LEDPoint = m_LEDPoint + 2;
        m_led.setData(m_ledBuffer);
        }
       }
      else if(m_loopcount %258 == 0){
        
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Set the value
          m_ledBuffer.setHSV(i, 0, 100, 0);
          //System.out.print("Works" + m_loopcount);
          m_led.setData(m_ledBuffer);
        }
        
      }
  }
  public void redchase() {
    // m_loopcount = 0;
    // System.out.print("Reached here " + m_loopcount);
     if(m_loopcount %5 == 0){
      for (var i = (m_LEDPoint); i < m_ledBuffer.getLength(); i += 86) {
        m_ledBuffer.setHSV((TOTAL_LED_COUNT -1) - i, 0, 255, 128);
        m_ledBuffer.setHSV(i, 0, 255, 128);
        m_LEDPoint = m_LEDPoint + 2;
        m_led.setData(m_ledBuffer);
        }
       }
      else if(m_loopcount %258 == 0){
        
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Set the value
          m_ledBuffer.setHSV(i,  0, 100, 0);
          //System.out.print("Works" + m_loopcount);
          m_led.setData(m_ledBuffer);
        }
        
      }
  }
  
  public void updateClimberLEDInformation(double prompt) {
    if (prompt == 1) {
      climberLEDHighBar = true;
    } else {
      climberLEDLowBar = true;
    }
  }

  public void resetClimberLEDInformation() {
    climberLEDHighBar = false;
    climberLEDLowBar = false;
  }

  public void setClimbInProgress(double checker) {
    if (checker == 1) {
      climbInProgress = true;
    } else {
      climbInProgress = false;
    }
  }

  public void climbFinished() {
    climberFinishedClimb = true;
  }

  public boolean checkClimbInProgress() {
    return climbInProgress;
  }
}
  
