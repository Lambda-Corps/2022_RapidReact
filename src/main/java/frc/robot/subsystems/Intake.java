// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
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

  //positions
  final int ARM_POSITION_ZERO = 0; //intake fully vertical/up
  final int INTAKE_ARM_DOWN = 1500; //intake down to grab ball (currently has temporary value) TODO get this value

  final double DOWN_FEEDFORWARD = .2;
  final double UP_FEEDFORWARD = -.3;
  final double HOLD_FEEDFORWARD = UP_FEEDFORWARD; //PID (would this one also be motion magic?) TODO get this value (current one is estimated)
  
  //gains for intake arm
  private Gains m_intakeArmDownGains = kGains_IntakeDown;
  private Gains m_intakeArmUpGains = kGains_IntakeUp;

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
  private final int ARM_REVERSE_SOFT_LIMIT = -20;
  private final int ARM_FORWARD_SOFT_LIMIT = 1850;

  //max speed
  private final double MAX_ARM_SPEED = 0; //TODO set this
  private final double STANDARD_INTAKE_SPEED = 0; //^^^

  ShuffleboardTab m_intakeTab;
  NetworkTableEntry m_armMaxSpeed, m_armStandardSpeed, m_maxFF, m_minFF, m_forwardSoftLimit, m_armEncoder;

  private double MM_FEEDFORWARD = 0;
  private boolean m_is_on_target;

  public Intake() {
    m_armMotor = new TalonSRX(INTAKE_ARM_TALON);
    m_intakeMotor = new TalonSRX(INTAKE_TALON);

    m_intakeTab = Shuffleboard.getTab("Intake");
    m_forwardSoftLimit = m_intakeTab.add("unusedname", 0)
                                    .withPosition(0, 0)
                                    .withSize(1, 1)
                                    .getEntry();
    m_armEncoder = m_intakeTab.add("Arm Encoder Position", 0).getEntry();
    
    m_intakeMotor.configFactoryDefault();
    m_intakeMotor.setInverted(false);
    m_armMotor.configFactoryDefault();
    m_armMotor.setInverted(true);
    m_armMotor.setSensorPhase(false);
  
    //selected feedback
    m_armMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    m_armMotor.configForwardSoftLimitThreshold(ARM_FORWARD_SOFT_LIMIT);
    m_armMotor.configReverseSoftLimitThreshold(ARM_REVERSE_SOFT_LIMIT);
    m_armMotor.configForwardSoftLimitEnable(false);
    m_armMotor.configReverseSoftLimitEnable(false);
    //mm
    m_armMotor.configMotionCruiseVelocity(100, 0);
    m_armMotor.configMotionAcceleration(100, 0);

    //current limits?
    //m_armMotor.configPeakCurrentLimit(0);
    //m_armMotor.configContinuousCurrentLimit(amps);
    
    //limit switches
   //m_armMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 0);
   // m_armMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, 0); //hopefully this is correct?

    //config PIDF values
    m_armMotor.config_kP(0, m_intakeArmDownGains.kP, 0);
    m_armMotor.config_kI(0, m_intakeArmDownGains.kI, 0);
    m_armMotor.config_kD(0, m_intakeArmDownGains.kD, 0);
    m_armMotor.config_kF(0, m_intakeArmDownGains.kF, 0);

    m_armMotor.config_kP(1, m_intakeArmUpGains.kP, 0);
    m_armMotor.config_kI(1, m_intakeArmUpGains.kI, 0);
    m_armMotor.config_kD(1, m_intakeArmUpGains.kD, 0);
    m_armMotor.config_kF(1, m_intakeArmUpGains.kF, 0);

    //config closed loop error
    m_armMotor.configAllowableClosedloopError(0, 10, 0); //TODO determine what tolerance needs to be

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //if(getArmLimit()){
      m_armMotor.getSelectedSensorPosition(0);
    //}
  }

  public void setArmMotor (double speed){
    if(Math.abs(speed) > MAX_ARM_SPEED){
      if(speed > 0){
        speed = -MAX_ARM_SPEED;
      } else{
        speed = MAX_ARM_SPEED;
      }
    }
    m_armMotor.set(ControlMode.PercentOutput, speed);
  }


  public  void configStartMM(int position, double kP, double kI, double kD, double kF){
      if (position > getRelativeEncoder()) {
          // forward slot
          m_armMotor.selectProfileSlot(0, 0);
          m_armMotor.config_kP(0, kP, 0); // find values
          m_armMotor.config_kI(0, kI, 0); // find values
          m_armMotor.config_kD(0, kD, 0); // find values
          m_armMotor.config_kF(0, kF, 0); // find values
          // armMotor.config_kF(0, 1, 0); // find values - auxiliary feed forward
          MM_FEEDFORWARD = DOWN_FEEDFORWARD;
      } else {
          // backward slot
          m_armMotor.selectProfileSlot(1, 0);
          m_armMotor.config_kP(1, kP, 0); // find values
          m_armMotor.config_kI(1, kI, 0); // find values
          m_armMotor.config_kD(1, kD, 0); // find values
          m_armMotor.config_kF(1, kF, 0); // find values
          // armMotor.config_kF(1, 1, 0); // find values - auxiliary feed forward
          MM_FEEDFORWARD = UP_FEEDFORWARD;
      }

  }

  public boolean moveMM(int targetPosition){    
    m_armMotor.set(ControlMode.MotionMagic, targetPosition, DemandType.ArbitraryFeedForward, MM_FEEDFORWARD);
    double tolerance = 20; //TODO
    double currentPosition = m_armMotor.getSelectedSensorPosition();
    m_is_on_target = Math.abs(currentPosition - targetPosition) < tolerance;
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

  public void holdMotorPosition(int position, double arbFF){
    if( position <= 75){
      // No feedforward when resting at home
      m_armMotor.set(ControlMode.Position, position);
      System.out.println("not setting arbFF");
    }
    else {
      // We are holding the arm in space, requires about -0.3 % measured
      m_armMotor.set(ControlMode.Position, position, DemandType.ArbitraryFeedForward, arbFF);
      System.out.println("setting Position-arbFF: " + position + "-" + arbFF);
    }
  }
}
