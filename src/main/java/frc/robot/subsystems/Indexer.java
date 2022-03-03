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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final double INTAKE_SHOOT_SPEED = .27; // measured testing

  public enum StorageState {
    EMPTY, 
    BOTTOMONLY,
    MIDDLEONLY,
    TOPONLY,
    BOTTOMTWO,
    TOPTWO,
    TOPANDBOTTOM,
    FULL,
    PURGE // This is if the state machine cannot decide what state the indexer is in. If applied it will eject all cargo.
  }

  TalonSRX m_intakeIndex, m_midIndex, m_shooterIndex;
//  private final NetworkTableEntry m_intakeIndexerEntry, m_midIndexerEntry, m_shooterIndexerEntry;
//  public ShuffleboardTab m_IntakeTab;

  private DigitalInput m_bottomBeam, m_topBeam, m_midbeam;
  private StorageState m_storageStatus;

  private boolean m_bottomBeamState, m_topBeamState, m_midBeamState;

  /** Creates a new Indexer. */
  public Indexer() {
    m_intakeIndex = new TalonSRX(INTAKE_INDEXER);
    m_midIndex = new TalonSRX(MID_INDEXER);
    m_shooterIndex = new TalonSRX(SHOOTER_INDEXER);

    // m_bottomBeam = new DigitalInput(Constants.BEAM_BREAKER_RECEIVE_BOTTOM);
    // m_topBeam = new DigitalInput(Constants.BEAM_BREAKER_RECEIVE_TOP);
    m_bottomBeam = new DigitalInput(INDEXER_INTAKE_BEAM);
    m_midbeam = new DigitalInput(INDEXER_INTAKE_MID);
    m_topBeam = new DigitalInput(INDEXER_INTAKE_SHOOTER);

    m_storageStatus = StorageState.EMPTY;

    // Add a shuffleboard tab for any testing, tuning, or debugging, etc
    ShuffleboardTab tab = Shuffleboard.getTab("Indexer");
    tab.addBoolean("Shooter Beam", m_topBeam::get);
    tab.addBoolean("Mid Beam", m_midbeam::get);
    tab.addBoolean("Intake Beam", m_bottomBeam::get);
  }

  @Override
  public void periodic() {}

  public void checkIndexState() {
    // Real State Machine
    m_bottomBeamState = m_bottomBeam.get();
    m_midBeamState = m_midbeam.get();
    m_topBeamState = m_topBeam.get();
  
    if (!m_bottomBeamState && !m_topBeamState && !m_midBeamState) {
      m_storageStatus = StorageState.EMPTY;
    }else if (!m_bottomBeamState && m_topBeamState && m_midBeamState) {
      m_storageStatus = StorageState.BOTTOMONLY;
    }else if (!m_bottomBeamState && m_topBeamState && !m_midBeamState) { 
      m_storageStatus = StorageState.BOTTOMTWO;
    }else if (!m_bottomBeamState && !m_topBeamState && m_midBeamState) {
      m_storageStatus = StorageState.TOPANDBOTTOM;
    }else if (m_bottomBeamState && !m_topBeamState && !m_midBeamState) {
      m_storageStatus = StorageState.TOPTWO; 
    }else if (!m_bottomBeamState && m_topBeamState && m_midBeamState) {
      m_storageStatus = StorageState.BOTTOMONLY;
    }else if (m_bottomBeamState && m_topBeamState && !m_midBeamState) {
      m_storageStatus = StorageState.MIDDLEONLY;
    }else if (m_bottomBeamState && !m_topBeamState && m_midBeamState) {
      m_storageStatus = StorageState.TOPONLY;
    }
  }

  public void resolveIndexer(boolean forceSpinBottom) {
    switch(m_storageStatus) {
      case EMPTY:
        if (forceSpinBottom) {
          m_intakeIndex.set(ControlMode.PercentOutput, 0.8);
        }else {
          m_intakeIndex.set(ControlMode.PercentOutput, 0);
        }
        m_midIndex.set(ControlMode.PercentOutput, 0);
        m_shooterIndex.set(ControlMode.PercentOutput, 0);
        break;
      case BOTTOMONLY:
      case BOTTOMTWO:
        m_intakeIndex.set(ControlMode.PercentOutput, INDEXER_SPEED);
        m_midIndex.set(ControlMode.PercentOutput, INDEXER_SPEED);
        m_shooterIndex.set(ControlMode.PercentOutput, 0);
        break;
      case MIDDLEONLY:
        m_intakeIndex.set(ControlMode.PercentOutput, 0);
        m_midIndex.set(ControlMode.PercentOutput, INDEXER_SPEED);
        m_shooterIndex.set(ControlMode.PercentOutput, 0);
        break;
      case TOPTWO:
      case TOPONLY:
      case TOPANDBOTTOM:
      case FULL:
      default:
        m_intakeIndex.set(ControlMode.PercentOutput, 0);
        m_midIndex.set(ControlMode.PercentOutput, 0);
        m_shooterIndex.set(ControlMode.PercentOutput, 0);
        break;
    }
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
    m_intakeIndex.set(ControlMode.PercentOutput, INTAKE_SHOOT_SPEED);
    m_midIndex.set(ControlMode.PercentOutput, INTAKE_SHOOT_SPEED);
    m_shooterIndex.set(ControlMode.PercentOutput, INTAKE_SHOOT_SPEED);
  }

  public void ejectBallsBackward(){
    m_intakeIndex.set(ControlMode.PercentOutput, -INTAKE_SHOOT_SPEED);
    m_midIndex.set(ControlMode.PercentOutput, -INTAKE_SHOOT_SPEED);
    m_shooterIndex.set(ControlMode.PercentOutput, -INTAKE_SHOOT_SPEED);
  }
}
