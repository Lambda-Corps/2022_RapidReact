// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.PID_PRIMARY;
import static frc.robot.Constants.SHOOTER_FX;
import static frc.robot.Constants.kGains_CloseShot;
import static frc.robot.Constants.kGains_MidTarmac;
import static frc.robot.Constants.kGains_ShooterGains;
import static frc.robot.Constants.kGains_TarmacLine;
import static frc.robot.Constants.kSlot_CloseShot;
import static frc.robot.Constants.kSlot_MidTarmac;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  public static enum ShotDistance {
    MidTarmac, ClosestShot, TarmacLine
  }
  private final WPI_TalonFX m_Shooter;
  private final TalonFXConfiguration shooterConfig;
  private int m_shooter_set_point;

  //10% is 1918
  private final int SHOOTER_SETPOINT_TARMAC_LINE   = 8200, //40%
                    SHOOTER_SETPOINT_CLOSESHOT   = 6700,   //30%
                    SHOOTER_SETPOINT_MIDTARMAC = 6760;     //35%

  // Move balls that are stuck
  private final double SHOOTER_EJECT_SPEED = -.15;
  private final double SHOOTER_TOLERANCE = .03; // within 3% of the set point is good 

  // Keep track of whether we're shooting, either for LEDs or shooting sequences
  private boolean m_is_shooting;
  private int m_shooter_clock_count;
  private double m_closed_loop_error;

  NetworkTableEntry m_at_speed_entry;
    
  /** Creates a new Shooter. */
  public Shooter() {
    m_Shooter = new WPI_TalonFX(SHOOTER_FX);
    
    shooterConfig = new TalonFXConfiguration();
    // This is the configuration for the initation line shot
    shooterConfig.slot0.kF = kGains_ShooterGains.kF;
    shooterConfig.slot0.kP = kGains_ShooterGains.kP;
    shooterConfig.slot0.kI = kGains_ShooterGains.kI;
    shooterConfig.slot0.kD = kGains_ShooterGains.kD;
    shooterConfig.slot0.integralZone = kGains_ShooterGains.kIzone;
    shooterConfig.slot0.closedLoopPeakOutput = kGains_ShooterGains.kPeakOutput;

    // This is the configuration for the initation line shot
    shooterConfig.slot1.kF = kGains_CloseShot.kF;
    shooterConfig.slot1.kP = kGains_CloseShot.kP;
    shooterConfig.slot1.kI = kGains_CloseShot.kI;
    shooterConfig.slot1.kD = kGains_CloseShot.kD;
    shooterConfig.slot1.integralZone = kGains_CloseShot.kIzone;
    shooterConfig.slot1.closedLoopPeakOutput = kGains_CloseShot.kPeakOutput;

    // Configuration t2 shoot from up against the wall
    shooterConfig.slot2.kF = kGains_MidTarmac.kF;
    shooterConfig.slot2.kP = kGains_MidTarmac.kP;
    shooterConfig.slot2.kI = kGains_MidTarmac.kI;
    shooterConfig.slot2.kD = kGains_MidTarmac.kD;
    shooterConfig.slot2.integralZone = kGains_MidTarmac.kIzone;
    shooterConfig.slot2.closedLoopPeakOutput = kGains_MidTarmac.kPeakOutput;

    // Configuration to shoot from the front of the TARMACK
    shooterConfig.slot3.kF = kGains_TarmacLine.kF;
    shooterConfig.slot3.kP = kGains_TarmacLine.kP;
    shooterConfig.slot3.kI = kGains_TarmacLine.kI;
    shooterConfig.slot3.kD = kGains_TarmacLine.kD;
    shooterConfig.slot3.integralZone = kGains_TarmacLine.kIzone;
    shooterConfig.slot3.closedLoopPeakOutput = kGains_TarmacLine.kPeakOutput;
    
    // Setup the open loop limits so the drivers can't mess it up
    shooterConfig.peakOutputReverse = -.2;
    shooterConfig.peakOutputForward = 1;
    shooterConfig.nominalOutputForward = 0;
    shooterConfig.closedloopRamp = .5;
    
    shooterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    m_Shooter.configAllSettings(shooterConfig);
    m_Shooter.selectProfileSlot(kSlot_CloseShot, 0);

    m_Shooter.setInverted(true);

    m_shooter_set_point = SHOOTER_SETPOINT_CLOSESHOT;
    // Default to the initiation line

    m_is_shooting = false;
    m_shooter_clock_count = 0;

    m_at_speed_entry = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Shooter").getEntry("At Speed");
  }

  public void stopMotor() {
    m_is_shooting = false;
    m_shooter_clock_count = 0;
    m_Shooter.set(ControlMode.PercentOutput, 0);
  }

  public void velocityPID(double setpoint){
    m_Shooter.set(ControlMode.Velocity, m_shooter_set_point);
  }

  public void setProfileSlot(){
    m_Shooter.selectProfileSlot(kSlot_CloseShot, PID_PRIMARY);

  }
  public void configureVelocityPID(double kp, double ki, double kd, double kf) {
    m_Shooter.selectProfileSlot(kSlot_CloseShot, PID_PRIMARY);
    m_Shooter.config_kP(kSlot_CloseShot, kp);
    m_Shooter.config_kI(kSlot_CloseShot, ki);
    m_Shooter.config_kD(kSlot_CloseShot, kd);
    m_Shooter.config_kF(kSlot_CloseShot, kf);
  }
  public void setShotDistance(ShotDistance distance){
    switch(distance){
      case ClosestShot:
        // Config the shooter for close shot
        m_Shooter.selectProfileSlot(kSlot_CloseShot, PID_PRIMARY);
        // Set the setpoint
        m_shooter_set_point = SHOOTER_SETPOINT_CLOSESHOT ;
        // Set the pistons
        break;
      case MidTarmac:
        // Config the shooter for mid tarmac
        m_Shooter.selectProfileSlot(kSlot_MidTarmac, PID_PRIMARY);
        // Set the setpoint
        m_shooter_set_point = SHOOTER_SETPOINT_MIDTARMAC;
        // Set the pistons
        break;
      case TarmacLine:
        // Config the shooter for tarmac line
        m_Shooter.selectProfileSlot(SHOOTER_SETPOINT_TARMAC_LINE, PID_PRIMARY);
        // Set the setpoint
        m_shooter_set_point = SHOOTER_SETPOINT_TARMAC_LINE;
        // Set the pistons
        break;
    }
  }

  public void test_shooter_percent(double speed){
    m_Shooter.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_is_shooting){
      m_shooter_clock_count++;
    }

    m_closed_loop_error = m_Shooter.getClosedLoopError(PID_PRIMARY);
  }

  public void startVelocityPID(){
    m_is_shooting = true;
    m_Shooter.set(ControlMode.Velocity, m_shooter_set_point);
  }

  public boolean isUpToSpeed(){
    double currentLoopError = Math.abs(m_Shooter.getClosedLoopError());

    boolean atspeed = currentLoopError < (m_shooter_set_point * SHOOTER_TOLERANCE);
    m_at_speed_entry.setBoolean(atspeed);
    // The command was ending right as it started because the closed loop error
    // hadn't gotten far enough away from the inertia start.
    // Only return true if we've been spinning for at least half a second AND we
    // are spinning within the tolerance
    atspeed = atspeed && (m_shooter_clock_count >= 25);
    return atspeed ;
  }

  public double getClosedLoopError(){
    return m_closed_loop_error;
  }

  public void ejectBallsBackward() {
    m_Shooter.set(ControlMode.PercentOutput, SHOOTER_EJECT_SPEED);
  }

  public double getLoopCount(){
    return m_shooter_clock_count;
  }
}
