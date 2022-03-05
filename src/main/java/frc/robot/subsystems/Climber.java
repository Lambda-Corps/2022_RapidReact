// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.*;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  TalonSRX m_climberMotor;
  Faults m_faults;

  //Limit Switches
  DigitalInput m_reverseLimitSwitch;
  DigitalInput m_forwardLimitSwitch;

  private boolean m_limitReached;

  //Soft Limits
  private final int CLIMBER_REVERSE_SOFT_LIMIT = 0;
  private final int CLIMBER_FORWARD_SOFT_LIMIT = 24300; 

  //Max Climber Speed
  private final double MAX_CLIMBER_SPEED = 1; //TODO: get proper max climber speed

  public Climber() {
    m_faults = new Faults();

    m_limitReached = false;

    m_climberMotor = new TalonSRX(CLIMER_SRX);
    m_climberMotor.configFactoryDefault();
    m_climberMotor.setInverted(false);

    //Limit Switches
    m_reverseLimitSwitch = new DigitalInput(REVERSE_LIMIT_SWITCH);
    m_forwardLimitSwitch = new DigitalInput(FORWARD_LIMIT_SWITCH);

    //Feedback
    m_climberMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_climberMotor.configForwardSoftLimitThreshold(CLIMBER_FORWARD_SOFT_LIMIT);
    m_climberMotor.configReverseSoftLimitThreshold(CLIMBER_REVERSE_SOFT_LIMIT);
    m_climberMotor.configForwardSoftLimitEnable(true);
    m_climberMotor.configReverseSoftLimitEnable(true);

    m_climberMotor.setSelectedSensorPosition(0);

    ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
    climberTab.addNumber("Encoder", this::getRelativeEncoder).withPosition(1, 1);
    climberTab.addBoolean("Forward Limit", this::forwardLimitSwitchTriggered).withPosition(2,1);
    climberTab.addBoolean("Reverse Limit", this::reverseLimitSwitchTriggered).withPosition(3,1);
    climberTab.addBoolean("Soft Forward Limit", this::getClimberSoftForwardLimit).withPosition(4,1);
    climberTab.addBoolean("Soft Reverse Limit", this::getClimberSoftReverseLimit).withPosition(5,1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_climberMotor.getFaults(m_faults);
  }

  public void setClimberMotor(double speed) {
    if (speed > 0) {
      if (forwardLimitSwitchTriggered()) {
        speed = 0; //The limit has been reached, stop immediately.
      } else {
        if (speed > MAX_CLIMBER_SPEED) {
          speed = MAX_CLIMBER_SPEED;
        }
      }
    } else {
      if (reverseLimitSwitchTriggered()) {
        speed = 0; //The limit has been reached, stop immediately.
      } else {
        if (speed < -MAX_CLIMBER_SPEED) {
          speed = -MAX_CLIMBER_SPEED;
        }
      }
    }
    m_climberMotor.set(ControlMode.PercentOutput, speed);
  }

  public void climberStopMotor() {
    m_climberMotor.set(ControlMode.PercentOutput, 0);
  }

  public void resetClimberMotorEncoder() {
    m_climberMotor.setSelectedSensorPosition(0);
    m_climberMotor.configReverseSoftLimitThreshold(CLIMBER_REVERSE_SOFT_LIMIT);
  }

  public void climberIsAtHighest() {
    m_climberMotor.setSelectedSensorPosition(CLIMBER_FORWARD_SOFT_LIMIT);
  }
  
  public boolean forwardLimitSwitchTriggered() {
    return !m_forwardLimitSwitch.get() || m_faults.ForwardSoftLimit;
  }

  public boolean reverseLimitSwitchTriggered() {
    return !m_reverseLimitSwitch.get() || m_faults.ReverseSoftLimit;
  }

  public boolean getClimberSoftForwardLimit() {
    return m_faults.ForwardSoftLimit;
  }

  public boolean getClimberSoftReverseLimit() {
    return m_faults.ReverseSoftLimit;
  }

  public double getRelativeEncoder() {
    return m_climberMotor.getSelectedSensorPosition();
  }
}