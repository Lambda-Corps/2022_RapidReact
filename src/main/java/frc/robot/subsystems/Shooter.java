// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Shooter extends SubsystemBase {
  public enum ShotDistance {
    Field_Wall, InitiationLine, FrontTarmack
  }
  private final WPI_TalonFX m_Shooter;
  private final TalonFXConfiguration shooterConfig;
  private int m_shooter_set_point;
    
  /** Creates a new Shooter. */
  public Shooter() {
    m_Shooter = new WPI_TalonFX(SHOOTER_FX);
    
    shooterConfig = new TalonFXConfiguration();
    // This is the configuration for the initation line shot
    shooterConfig.slot0.kF = kGains_InitiationLine.kF;
    shooterConfig.slot0.kP = kGains_InitiationLine.kP;
    shooterConfig.slot0.kI = kGains_InitiationLine.kI;
    shooterConfig.slot0.kD = kGains_InitiationLine.kD;
    shooterConfig.slot0.integralZone = kGains_InitiationLine.kIzone;
    shooterConfig.slot0.closedLoopPeakOutput = kGains_InitiationLine.kPeakOutput;

    // Configuration to shoot from up against the wall
    shooterConfig.slot1.kF = kGains_Field_Wall.kF;
    shooterConfig.slot1.kP = kGains_Field_Wall.kP;
    shooterConfig.slot1.kI = kGains_Field_Wall.kI;
    shooterConfig.slot1.kD = kGains_Field_Wall.kD;
    shooterConfig.slot1.integralZone = kGains_Field_Wall.kIzone;
    shooterConfig.slot1.closedLoopPeakOutput = kGains_Field_Wall.kPeakOutput;

    // Configuration to shoot from the front of the TARMACK
    shooterConfig.slot2.kF = kGains_FrontTarmack.kF;
    shooterConfig.slot2.kP = kGains_FrontTarmack.kP;
    shooterConfig.slot2.kI = kGains_FrontTarmack.kI;
    shooterConfig.slot2.kD = kGains_FrontTarmack.kD;
    shooterConfig.slot2.integralZone = kGains_FrontTarmack.kIzone;
    shooterConfig.slot2.closedLoopPeakOutput = kGains_FrontTarmack.kPeakOutput;
    
    // Setup the open loop limits so the drivers can't mess it up
    shooterConfig.peakOutputReverse = 0;
    shooterConfig.peakOutputForward = 1;
    shooterConfig.nominalOutputForward = 0;

    m_Shooter.configAllSettings(shooterConfig);

    m_Shooter.setInverted(false);
    m_Shooter.setSensorPhase(false);
    m_Shooter.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    // config bottom motor to follow top
    m_Shooter.setInverted(false);
    m_Shooter.follow(m_Shooter);
    m_shooter_set_point = SHOOTER_SETPOINT_LINE;
    // Default to the initiation line
  }
  public void stopMotors() {
    m_Shooter.set(ControlMode.PercentOutput, 0);
  }
  public void velocityPID(double m_setpoint){
    m_Shooter.set(ControlMode.Velocity, m_setpoint);
  }
  public void configureVelocityPID(double kp, double ki, double kd, double kf) {
    m_Shooter.selectProfileSlot(SHOOTER_SLOT_INITIATION_LINE, PID_PRIMARY);
    m_Shooter.config_kP(SHOOTER_SLOT_INITIATION_LINE, kp);
    m_Shooter.config_kI(SHOOTER_SLOT_INITIATION_LINE, ki);
    m_Shooter.config_kD(SHOOTER_SLOT_INITIATION_LINE, kd);
    m_Shooter.config_kF(SHOOTER_SLOT_INITIATION_LINE, kf);
  }
  public void setShotDistance(ShotDistance distance){
    switch(distance){
      case InitiationLine:
        // Config the shooter for initiation line
        m_Shooter.selectProfileSlot(SHOOTER_SLOT_INITIATION_LINE, PID_PRIMARY);
        // Set the setpoint
        m_shooter_set_point = SHOOTER_SETPOINT_LINE;
        // Set the pistons
        break;
      case Field_Wall:
        // Config the shooter for initiation line
        m_Shooter.selectProfileSlot(SHOOTER_SLOT_Field_Wall, PID_PRIMARY);
        // Set the setpoint
        m_shooter_set_point = SHOOTER_SETPOINT_WALL;
        // Set the pistons
        break;
      case FrontTarmack:
        // Config the shooter for initiation line
        m_Shooter.selectProfileSlot(SHOOTER_SLOT_FRONT_TARMACK, PID_PRIMARY);
        // Set the setpoint
        m_shooter_set_point = SHOOTER_SETPOINT_TARMACK;
        // Set the pistons
        break;
    }
    }
  @Override
  public void periodic() {




    // This method will be called once per scheduler run
  }
}
