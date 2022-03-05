// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static frc.robot.Constants.INTAKE_ARM_TALON;
import static frc.robot.Constants.INTAKE_TALON;
import static frc.robot.Constants.PID_PRIMARY;
import static frc.robot.Constants.kGains_IntakeDown;
import static frc.robot.Constants.kGains_IntakeHold;
import static frc.robot.Constants.kGains_IntakeUp;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Gains;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  //motors
  TalonSRX m_armMotor; //calling up and down movement arm for lack of better term
  TalonSRX m_intakeMotor;
  Faults m_faults;

  //positions
  public static final int INTAKE_ARM_RETRACT = 0; //intake fully vertical/up
  public static final int INTAKE_ARM_EXTEND = 1525; //intake down to grab ball (currently has temporary value)

  final double DOWN_FEEDFORWARD = .2;
  final double UP_FEEDFORWARD = -.3;
  final double HOLD_FEEDFORWARD = -.2; //PID (would this one also be motion magic?)
  final double MM_DONE_TOLERANCE = 10;
  
  //gains for intake arm
  private Gains m_intakeArmDownGains = kGains_IntakeDown;
  private Gains m_intakeArmUpGains = kGains_IntakeUp;
  private Gains m_intakeHoldPositionGains = kGains_IntakeHold;

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
  
  //soft limits
  private final int ARM_REVERSE_SOFT_LIMIT = 0;
  private final int ARM_FORWARD_SOFT_LIMIT = 1850;

  ShuffleboardTab m_intakeTab;
  NetworkTableEntry m_armMaxSpeed, m_armStandardSpeed, m_maxFF, m_minFF, m_forwardSoftLimit, m_armEncoder;

  private double MM_FEEDFORWARD = 0;
  private boolean m_is_on_target;

  private final int ARM_SLOT_DOWN = 0;
  private final int ARM_SLOT_UP   = 1;
  private final int ARM_SLOT_HOLD = 2;

  private final double INTAKE_WHEEL_SPEEDS = .8;

  public Intake() {
    m_faults = new Faults();
    m_armMotor = new TalonSRX(INTAKE_ARM_TALON);
    m_intakeMotor = new TalonSRX(INTAKE_TALON);
    
    m_intakeMotor.configFactoryDefault();
    m_intakeMotor.setInverted(false);
    m_armMotor.configFactoryDefault();
    m_armMotor.setInverted(true);
    m_armMotor.setSensorPhase(false);
  
    //selected feedback
    m_armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_armMotor.configForwardSoftLimitThreshold(ARM_FORWARD_SOFT_LIMIT);
    m_armMotor.configReverseSoftLimitThreshold(ARM_REVERSE_SOFT_LIMIT);
    m_armMotor.configForwardSoftLimitEnable(true);
    m_armMotor.configReverseSoftLimitEnable(true);
    // m_armMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    // m_armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    //mm
    m_armMotor.configMotionCruiseVelocity(300, 0);
    m_armMotor.configMotionAcceleration(300, 0);

    //current limits?
    //m_armMotor.configPeakCurrentLimit(0);
    //m_armMotor.configContinuousCurrentLimit(amps);
    
    //limit switches
   //m_armMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 0);
   // m_armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 0); //hopefully this is correct?

    //config PIDF values
    m_armMotor.config_kP(ARM_SLOT_DOWN, m_intakeArmDownGains.kP, 0);
    m_armMotor.config_kI(ARM_SLOT_DOWN, m_intakeArmDownGains.kI, 0);
    m_armMotor.config_kD(ARM_SLOT_DOWN, m_intakeArmDownGains.kD, 0);
    m_armMotor.config_kF(ARM_SLOT_DOWN, m_intakeArmDownGains.kF, 0);

    m_armMotor.config_kP(ARM_SLOT_UP, m_intakeArmUpGains.kP, 0);
    m_armMotor.config_kI(ARM_SLOT_UP, m_intakeArmUpGains.kI, 0);
    m_armMotor.config_kD(ARM_SLOT_UP, m_intakeArmUpGains.kD, 0);
    m_armMotor.config_kF(ARM_SLOT_UP, m_intakeArmUpGains.kF, 0);
    
    /*
     * CTRE documentation says "When using position closed loop, it is generally desired to use a kF of ‘0’. 
     * During this mode target position is multiplied by kF and added to motor output. 
     * If providing a feedforward is necessary, we recommend using the arbitrary 
     * feed forward term (4 param Set) to better implement this."
     * 
     * So this set of gains is put in place to be the position closed-loop used to keep our intake arm
     * in the place where we want it to be. So, no kF in these gains. 
     */
    m_armMotor.config_kP(ARM_SLOT_HOLD, m_intakeHoldPositionGains.kP, 0);
    m_armMotor.config_kI(ARM_SLOT_HOLD, m_intakeHoldPositionGains.kI, 0);
    m_armMotor.config_kD(ARM_SLOT_HOLD, m_intakeHoldPositionGains.kD, 0);
    m_armMotor.config_kF(ARM_SLOT_HOLD, m_intakeHoldPositionGains.kF, 0);

    //config closed loop error
    m_armMotor.configAllowableClosedloopError(0, 10, 0);

    ShuffleboardTab armMMTab = Shuffleboard.getTab("Arm MM Testing");
    armMMTab.addNumber("Encoder", this::getRelativeEncoder).withPosition(1, 1);
    armMMTab.addBoolean("Integrated Forward", this::getArmIntegratedForwardLimit).withPosition(2, 1);
    armMMTab.addBoolean("Integrated Reverse", this::getArmIntegratedReverseLimit).withPosition(3,1);
    armMMTab.addBoolean("Soft Forward", this::getArmSoftForwardLimit).withPosition(4, 1);
    armMMTab.addBoolean("Soft Reverse", this::getArmSoftReverseLimit).withPosition(5,1);

    holdMotorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //if(getArmLimit()){
      // m_armMotor.getSelectedSensorPosition(0);
    //}

    // Periodically grab the talon faults
    m_armMotor.getFaults(m_faults);
  }

  // public void setArmMotor (double speed){
  //   if(Math.abs(speed) > MAX_ARM_SPEED){
  //     if(speed > 0){
  //       speed = -MAX_ARM_SPEED;
  //     } else{
  //       speed = MAX_ARM_SPEED;
  //     }
  //   }
  //   m_armMotor.set(ControlMode.PercentOutput, speed);
  // }


  public  void configStartMM(int position){
      if (position > getRelativeEncoder()) {
          // forward slot
          // m_armMotor.config_kP(ARM_SLOT_DOWN, kP, 0); // find values
          // m_armMotor.config_kI(ARM_SLOT_DOWN, kI, 0); // find values
          // m_armMotor.config_kD(ARM_SLOT_DOWN, kD, 0); // find values
          // m_armMotor.config_kF(ARM_SLOT_DOWN, kF, 0); // find values
          // armMotor.config_kF(0, 1, 0); // find values - auxiliary feed forward
          MM_FEEDFORWARD = DOWN_FEEDFORWARD;
          m_armMotor.selectProfileSlot(ARM_SLOT_DOWN, PID_PRIMARY);
      } else {
          // backward slot
          // m_armMotor.config_kP(ARM_SLOT_UP, kP, 0); // find values
          // m_armMotor.config_kI(ARM_SLOT_UP, kI, 0); // find values
          // m_armMotor.config_kD(ARM_SLOT_UP, kD, 0); // find values
          // m_armMotor.config_kF(ARM_SLOT_UP, kF, 0); // find values
          // armMotor.config_kF(1, 1, 0); // find values - auxiliary feed forward
          MM_FEEDFORWARD = UP_FEEDFORWARD;
          m_armMotor.selectProfileSlot(ARM_SLOT_UP, PID_PRIMARY);

      }

  }

  public boolean moveMM(int targetPosition){
    int kMeasuredPosHorizontal = 900; //Position measured when arm is horizontal
    double kTicksPerDegree = 4096 / 360; //Sensor is 1:1 with arm rotation
    double currentPos = m_armMotor.getSelectedSensorPosition();
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    m_armMotor.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, MM_FEEDFORWARD * cosineScalar);
    m_is_on_target = Math.abs(currentPos - targetPosition) < MM_DONE_TOLERANCE;
    return m_is_on_target;
  }

  public boolean isOnTarget(){
    return m_is_on_target;
  }

  public double getMMError(){
    return m_armMotor.getClosedLoopError(0);
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
  
    public void intakeWheelsMotorOn(double speed){
      m_intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void collectBalls(){
      m_intakeMotor.set(ControlMode.PercentOutput, INTAKE_WHEEL_SPEEDS);
    }

  public void stopIntakeMotor(){
    m_intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getIntakeMotorCurrent(){
    return m_intakeMotor.getStatorCurrent();
  }

  public void reset_arm_PIDF_values(double arm_kP, double kI, double kD, double kF) {
    m_armMotor.config_kP(0, arm_kP, 0);
    m_armMotor.config_kI(0, kI, 0);
    m_armMotor.config_kD(0, kD, 0);
    m_armMotor.config_kF(0, kF, 0);
  }

  public void resetArmEncoder() {
    m_armMotor.setSelectedSensorPosition(0);
    m_armMotor.configReverseSoftLimitThreshold(ARM_REVERSE_SOFT_LIMIT);
  
  }

  public void setForwardLimit() {
    double pos = m_armMotor.getSelectedSensorPosition();
    m_armMotor.configForwardSoftLimitThreshold(pos);
  }

  public void holdMotorPosition(int position){
    int kMeasuredPosHorizontal = 900; //Position measured when arm is horizontal
    double kTicksPerDegree = 4096 / 360; //Sensor is 1:1 with arm rotation
    double degrees = (position - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    m_armMotor.selectProfileSlot(ARM_SLOT_HOLD, PID_PRIMARY);
    if( position == INTAKE_ARM_RETRACT){
      // No feedforward when resting at home
      m_armMotor.set(ControlMode.Position, position);
    }
    else if( position == INTAKE_ARM_EXTEND){
      m_armMotor.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, HOLD_FEEDFORWARD * cosineScalar);
    }
    else 
    {
      m_armMotor.set(ControlMode.PercentOutput, 0);
    }
  }

  public boolean getArmIntegratedForwardLimit() {
    return m_faults.ForwardLimitSwitch;
  }

  public boolean getArmIntegratedReverseLimit() {
    return m_faults.ReverseLimitSwitch;
  }

  public boolean getArmSoftForwardLimit() {
    return m_faults.ForwardSoftLimit;
  }

  public boolean getArmSoftReverseLimit() {
    return m_faults.ReverseSoftLimit;
  }
}
