// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.INDEXER_INTAKE_BEAM;
import static frc.robot.Constants.INDEXER_INTAKE_MID;
import static frc.robot.Constants.INDEXER_INTAKE_SHOOTER;
import static frc.robot.Constants.INDEXER_SPEED;
import static frc.robot.Constants.INTAKE_INDEXER;
import static frc.robot.Constants.MID_INDEXER;
import static frc.robot.Constants.SHOOTER_INDEXER;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final double INTAKE_SHOOT_SPEED = .4; // measured testing
  private final double INTAKE_INTAKE_SPEED = .55;

  public enum StorageState {
    EMPTY, 
    BOTTOMONLY,
    MIDDLEONLY,
    TOPONLY,
    BOTTOMTWO,
    TOPTWO,
    TOPANDBOTTOM,
    FULL,
    TOO_FAR,
    PURGE // This is if the state machine cannot decide what state the indexer is in. If applied it will eject all cargo.
  }

  TalonSRX m_intakeIndex, m_midIndex, m_shooterIndex;
 private final NetworkTableEntry m_intakeIndexerEntry, m_midIndexerEntry, m_shooterIndexerEntry, m_BallCountEntry;
//  public ShuffleboardTab m_IntakeTab;

  private DigitalInput m_bottomBeam, m_topBeam, m_midbeam;
  private StorageState m_storageStatus;

  private boolean m_bottomBeamState, m_topBeamState, m_midBeamState, m_IntakingBalls;
  private double m_ball_count;

  /** Creates a new Indexer. */
  public Indexer() {
    m_intakeIndex = new TalonSRX(INTAKE_INDEXER);
    m_midIndex = new TalonSRX(MID_INDEXER);
    m_shooterIndex = new TalonSRX(SHOOTER_INDEXER);

    m_midIndex.setNeutralMode(NeutralMode.Brake);
    m_intakeIndex.setNeutralMode(NeutralMode.Brake);
    m_shooterIndex.setNeutralMode(NeutralMode.Brake);

    // m_bottomBeam = new DigitalInput(Constants.BEAM_BREAKER_RECEIVE_BOTTOM);
    // m_topBeam = new DigitalInput(Constants.BEAM_BREAKER_RECEIVE_TOP);
    m_bottomBeam = new DigitalInput(INDEXER_INTAKE_BEAM);
    m_midbeam = new DigitalInput(INDEXER_INTAKE_MID);
    m_topBeam = new DigitalInput(INDEXER_INTAKE_SHOOTER);

    m_storageStatus = StorageState.EMPTY;

    // Add a shuffleboard tab for any testing, tuning, or debugging, etc
    NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive");

    m_intakeIndexerEntry = driveTable.getEntry("IntakeBreak");
    m_midIndexerEntry = driveTable.getEntry("MidBreak");
    m_shooterIndexerEntry = driveTable.getEntry("ShootBreak");
    m_BallCountEntry = driveTable.getEntry("Ball Count");

    m_IntakingBalls = false;
    m_ball_count = 0;
  }

  @Override
  public void periodic() {
    boolean bottom = m_bottomBeam.get();
    boolean mid = m_midbeam.get();
    boolean top = m_topBeam.get();
    m_intakeIndexerEntry.setBoolean(bottom);
    m_midIndexerEntry.setBoolean(mid);
    m_shooterIndexerEntry.setBoolean(top);

    // Calculate How many balls we have
    if( !bottom && !top ){
      // Should mean two balls
      m_ball_count = 2;
    }
    else if ( (!bottom && top )|| (bottom && !top) ) {
      m_ball_count = 1;
    }
    else {
      // Probably some malfunction or none
      m_ball_count = 0;
    }
    m_BallCountEntry.setNumber(m_ball_count);
  }

  public void checkIndexState() {
    // Real State Machine
    m_bottomBeamState = m_bottomBeam.get();
    m_midBeamState = m_midbeam.get();
    m_topBeamState = m_topBeam.get();
  
    if(!m_topBeamState){
      m_storageStatus = StorageState.TOO_FAR;
    } else if( !m_midBeamState && !m_bottomBeamState ) {
      m_storageStatus = StorageState.FULL;
    } else if( !m_bottomBeamState && m_midBeamState ){
      m_storageStatus = StorageState.BOTTOMONLY;
    } else if( !m_midBeamState && m_bottomBeamState ){
      m_storageStatus = StorageState.TOPONLY;
    } else if (m_midBeamState && m_bottomBeamState){
      m_storageStatus = StorageState.EMPTY;
    } else {
      // who knows what happened, try and purge
      m_storageStatus = StorageState.PURGE;
    }
  }

  public void resolveIndexer() {
    // Set the default speeds to off.
    double intake_speed, mid_speed, shooter_speed;
    intake_speed = mid_speed = shooter_speed = 0;

    switch(m_storageStatus) {
      case FULL:
      case EMPTY:
      case TOPONLY:
        break;
      case BOTTOMONLY:
        intake_speed = 2*INDEXER_SPEED;
        mid_speed = 2*INDEXER_SPEED;
        break;
      case TOO_FAR:
        mid_speed = -3*INTAKE_SHOOT_SPEED;
        shooter_speed = -3*INTAKE_SHOOT_SPEED;
        intake_speed = -INDEXER_SPEED;
        break;
      case PURGE:
        intake_speed = -INTAKE_SHOOT_SPEED;
        mid_speed = -INTAKE_SHOOT_SPEED;
        shooter_speed = -INTAKE_SHOOT_SPEED;
        break;
      default:
        break;
    }

    if(m_IntakingBalls){
      intake_speed = INTAKE_INTAKE_SPEED;
    }
    m_intakeIndex.set(ControlMode.PercentOutput, intake_speed);
    m_midIndex.set(ControlMode.PercentOutput, mid_speed);
    m_shooterIndex.set(ControlMode.PercentOutput, shooter_speed);
  }

  /*
   * Drive the index motors at independent speeds.  Designed to be called from a specific 
   * test command to tune the indexer while running.
   */
  public void testIndexDriving(double intake, double mid, double shooter){
    m_intakeIndex.set(ControlMode.PercentOutput, intake);
    m_midIndex.set(ControlMode.PercentOutput, mid);
    m_shooterIndex.set(ControlMode.PercentOutput, shooter);
  }

  public boolean getBeams(){
    return m_bottomBeam.get() && m_midbeam.get() && m_topBeam.get();
  }
  
  public void shootBalls(){
    m_intakeIndex.set(ControlMode.PercentOutput, INTAKE_SHOOT_SPEED + 0.1);
    m_midIndex.set(ControlMode.PercentOutput, INTAKE_SHOOT_SPEED);
    m_shooterIndex.set(ControlMode.PercentOutput, INTAKE_SHOOT_SPEED);
  }

  public void ejectBallsBackward(){
    m_intakeIndex.set(ControlMode.PercentOutput, -INTAKE_SHOOT_SPEED);
    m_midIndex.set(ControlMode.PercentOutput, -INTAKE_SHOOT_SPEED);
    m_shooterIndex.set(ControlMode.PercentOutput, -INTAKE_SHOOT_SPEED);
  }

  public void setIntakingBallsTrue(){
    m_IntakingBalls = true;
  }

  public void setIntakingBallsFalse(){
    m_IntakingBalls = false;
  }

  public boolean isFull(){
    return m_ball_count == 2;
  }

  public void stopMotors(){
    m_intakeIndex.set(ControlMode.PercentOutput, 0);
    m_midIndex.set(ControlMode.PercentOutput, 0);
    m_shooterIndex.set(ControlMode.PercentOutput, 0);
  }

  public boolean isEmpty(){
    return m_ball_count == 0;
  }
}
