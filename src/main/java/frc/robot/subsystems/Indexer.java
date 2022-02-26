// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */

  public enum StorageState {
    EMPTY, 
    BOTTOMONLY,
    TOPONLY, 
    FULL,
    PURGE // This is if the state machine cannot decide what state the indexer is in. If applied it will eject all cargo.
  }

  TalonSRX m_intakeIndex, m_midIndex, m_shooterIndex;
//  private final NetworkTableEntry m_intakeIndexerEntry, m_midIndexerEntry, m_shooterIndexerEntry;
//  public ShuffleboardTab m_IntakeTab;

  private DigitalInput m_bottomBeam, m_topBeam;
  private StorageState m_storageStatus;

  private boolean m_bottomBeamState, m_topBeamState;

  private NetworkTableEntry m_intake_entry, m_shooter_entry;
  
  public Indexer() {
    m_intakeIndex = new TalonSRX(INTAKE_INDEXER);
    m_midIndex = new TalonSRX(MID_INDEXER);
    m_shooterIndex = new TalonSRX(SHOOTER_INDEXER);

    // m_bottomBeam = new DigitalInput(Constants.BEAM_BREAKER_RECEIVE_BOTTOM);
    // m_topBeam = new DigitalInput(Constants.BEAM_BREAKER_RECEIVE_TOP);

    m_storageStatus = StorageState.EMPTY;

    // Add a shuffleboard tab for any testing, tuning, or debugging, etc
    ShuffleboardTab tab = Shuffleboard.getTab("Indexer");

    // TODO -- Remove these after tuning and beam break sensors are in
    m_intake_entry = tab.add("Intake Beam", 0).withPosition(0, 3).getEntry();
    m_shooter_entry = tab.add("Shooter Beam", 0).withPosition(1,3).getEntry();
  }

  @Override
  public void periodic() {}

  // Command to be called from the indexer default command that will automate the state of the balls in
  // the indexer.
  public void check_and_resolve_indexer(){
    checkIndexState();
    resolveIndexer();
  }

  private void checkIndexState() {
    // Real State Machine
    // m_bottomBeamState = m_bottomBeam.get();
    // m_topBeamState = m_topBeam.get();

//=========================================================================
    // For testing ONLY; TODO Redact when beambreaks are in
    m_bottomBeamState = m_intake_entry.getDouble(0) == 1 ? true : false;
    m_topBeamState = m_shooter_entry.getDouble(0) == 1 ? true : false;
//=========================================================================

    if (!m_bottomBeamState && !m_topBeamState) {
      m_storageStatus = StorageState.EMPTY;
    }else if (m_bottomBeamState && m_topBeamState) {
      m_storageStatus = StorageState.FULL;
    }else if (m_bottomBeamState && !m_topBeamState) {
      m_storageStatus = StorageState.BOTTOMONLY;
    }else if (!m_bottomBeamState && m_topBeamState) {
    m_storageStatus = StorageState.TOPONLY;
    }
  }

  private void resolveIndexer() {
    if (m_storageStatus == StorageState.EMPTY) {
      m_intakeIndex.set(ControlMode.PercentOutput, 0);
      m_midIndex.set(ControlMode.PercentOutput, 0);
      m_shooterIndex.set(ControlMode.PercentOutput, 0);
    }else if (m_storageStatus == StorageState.FULL) {
      m_intakeIndex.set(ControlMode.PercentOutput, 0);
      m_midIndex.set(ControlMode.PercentOutput, 0);
      m_shooterIndex.set(ControlMode.PercentOutput, 0);
    }else if (m_storageStatus == StorageState.TOPONLY) {
      m_intakeIndex.set(ControlMode.PercentOutput, 0);
      m_midIndex.set(ControlMode.PercentOutput, 0);
      m_shooterIndex.set(ControlMode.PercentOutput, 0);
    }else if (m_storageStatus == StorageState.BOTTOMONLY) {
      m_intakeIndex.set(ControlMode.PercentOutput, INDEXER_SPEED);
      m_midIndex.set(ControlMode.PercentOutput, INDEXER_SPEED);
      m_shooterIndex.set(ControlMode.PercentOutput, 0);
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
}
