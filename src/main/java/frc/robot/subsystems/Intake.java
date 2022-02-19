// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.html.HTMLDocument.BlockElement;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Gains;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  /*self notes:
    needs 1 or 2 talons (with enconder), 2 limit switches (mechanical and software) (see remote limit switch ctre example), 
    pid commands for moving
    need to find
    see 2019 arm code for reference
    TODO should arm moving and grabbing ball be separate subsystems?
    TODO ^^i.e. do we want to do both (start the intake spinning while arm moving position) at the same time for efficiency
  */

  //motors
  TalonSRX m_armMotor; //calling up and down movement arm for lack of better term
  TalonSRX m_intakeMotor;
  
  //encoder?

  //positions
  public static final int ARM_POSITION_ZERO = 0; //intake fully vertical/up
  public static final int INTAKE_ARM_DOWN = 1000; //intake down to grab ball (currently has temporary value) TODO get this value
  //TODO Do we want a position somewhere in the middle?
  
  public static int MM_FEEDFORWARD; //PID (would this one also be motion magic?) TODO get this value
  
  //gains for intake arm
  private Gains m_intakeArmGains = Constants.kGains_IntakeArms;

  // /*
    // * kF = full forward value * duty-cycle (%) / runtime calculated target
    // (ticks,
    // * velocity units/100 ms) = 1023 * 100% / 1525 =
    // *
    // * ARM UP
    // * kF = 1023 * 0.5 / (250.5) = 2.0419161676646706586826347305389
    // *
    // * ARM DOWN kF =
    // *
    // */
  
  private static int kTimeoutMs = 5; //TODO determine value (current is from 2019)
  
  AnalogInput absoluteEncoder;

  //soft limits
  private final int ARM_REVERSE_SOFT_LIMIT = 0;
  private final int ARM_FORWARD_SOFT_LIMIT = INTAKE_ARM_DOWN;

  //max speed
  private final double MAX_ARM_SPEED = 0; //TODO set this
  private final double STANDARD_INTAKE_SPEED = 0; //^^^


  public Intake() {
    m_armMotor = new TalonSRX(Constants.INTAKE_ARM_TALON);
    m_intakeMotor = new TalonSRX(Constants.INTAKE_TALON);
    
    m_intakeMotor.configFactoryDefault();
    m_intakeMotor.setInverted(false);
    m_armMotor.configFactoryDefault();
    m_armMotor.setInverted(false); //TODO determine if either needs inverted
  
    //selected feedback
    m_armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_armMotor.configForwardSoftLimitThreshold(ARM_FORWARD_SOFT_LIMIT);
    m_armMotor.configReverseSoftLimitThreshold(ARM_REVERSE_SOFT_LIMIT);
    m_armMotor.configForwardSoftLimitEnable(true);
    m_armMotor.configReverseSoftLimitEnable(true);
    //mm
    m_armMotor.configMotionCruiseVelocity(0, kTimeoutMs);
    m_armMotor.configMotionAcceleration(0, kTimeoutMs);

    //current limits?
    m_armMotor.configPeakCurrentLimit(0);
    //m_armMotor.configContinuousCurrentLimit(amps);
    
    //limit switches
   //m_armMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, kTimeoutMs);
    m_armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, kTimeoutMs); //hopefully this is correct?

    //config PIDF values
    m_armMotor.config_kP(0, m_intakeArmGains.kP, 0);
    m_armMotor.config_kI(0, m_intakeArmGains.kI, 0);
    m_armMotor.config_kD(0, m_intakeArmGains.kD, 0);
    m_armMotor.config_kF(0, m_intakeArmGains.kF, 0);

    //config closed loop error
    m_armMotor.configAllowableClosedloopError(0, 10, kTimeoutMs); //TODO determine what tolerance needs to be
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //if(getArmLimit()){
      m_armMotor.getSelectedSensorPosition(0);
    //}
  }

  public void setMotor (double speed){
    if(Math.abs(speed) > MAX_ARM_SPEED){
      if(speed > 0){
        speed = -MAX_ARM_SPEED;
      } else{
        speed = MAX_ARM_SPEED;
      }
    }
    m_armMotor.set(ControlMode.PercentOutput, speed);
  }


  public  void configStartMM(double targetPosition){
    // if (targetPos > getRelativeEncoder()) {
            // forward slot
            m_armMotor.selectProfileSlot(0, 0);
            // armMotor.config_kP(0, kP, 0); // find values
            // armMotor.config_kI(0, kI, 0); // find values
            // armMotor.config_kD(0, kD, 0); // find values
            // armMotor.config_kF(0, kF, 0); // find values
            // armMotor.config_kF(0, 1, 0); // find values - auxiliary feed forward
        // } else {
        //     // backward slot
        //     armMotor.selectProfileSlot(1, 0);
        //     // armMotor.config_kP(1, kP, 0); // find values
        //     // armMotor.config_kI(1, kI, 0); // find values
        //     // armMotor.config_kD(1, kD, 0); // find values
        //     // armMotor.config_kF(1, kF, 0); // find values
        //     // armMotor.config_kF(1, 1, 0); // find values - auxiliary feed forward
        // }
  }

  public void moveMM(int targetPosition){
    switch(targetPosition){
      case ARM_POSITION_ZERO:
        m_armMotor.set(ControlMode.MotionMagic, targetPosition);
        break;
      case INTAKE_ARM_DOWN:
        m_armMotor.set(ControlMode.MotionMagic,  targetPosition, DemandType.ArbitraryFeedForward, MM_FEEDFORWARD); //basing off of 2019 arm
        break;
      default:
        //this shouldn't happen, if it does motors stay off
        break;
    }
  }

  public boolean onTarget_MM(double targetPosition){
    double tolerance = 10; //TODO
    double currentPosition = m_armMotor.getSelectedSensorPosition();
    return Math.abs(currentPosition - targetPosition) < tolerance;
  }

  //getter methods if needed? (would pheonix tuner be easier?)
  public double getIntakeArmPosition(){
    return m_armMotor.getSelectedSensorPosition();
  }

  public double getArmCurrent(){
    return m_armMotor.getStatorCurrent();
  }

  public double getRelativeEncoder(){
    return m_armMotor.getSelectedSensorPosition(0);
    }
  
  public double getAbsoluteEncoder(){
    return absoluteEncoder.getAverageVoltage();
  }

  public void intakeMotorOn(double speed){
    m_intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopIntakeMotor(){
    m_intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getIntakeMotorCurrent(){
    return m_intakeMotor.getStatorCurrent();
  }

  public void reset_arm_PIDF_values(double arm_kP, double kI, double kD, double kF) {
    m_armMotor.config_kP(0, arm_kP, kTimeoutMs);
    m_armMotor.config_kI(0, kI, kTimeoutMs);
    m_armMotor.config_kD(0, kD, kTimeoutMs);
    m_armMotor.config_kF(0, kF, kTimeoutMs);
  }
}
