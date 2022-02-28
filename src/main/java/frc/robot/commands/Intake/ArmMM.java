// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ArmMM extends CommandBase {
  /** Creates a new ArmMM. */
  Intake m_intake;
  int m_targetPosition;
  public final ShuffleboardTab armMMTab;
  private double m_arm_kP, m_kI, m_kD, m_kF;
  private NetworkTableEntry m_armkPEntry, m_kIEntry, m_kDEntry, m_targetPosEntry, m_kFEntry, m_arbFFEntry, m_MMError;
  private double m_start_time;
  private double m_upFF, m_downFF;
  private boolean m_isFinished;
  private int m_count_done;

  public ArmMM(Intake intake, int target) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    addRequirements(m_intake);
    m_targetPosition = target;
    armMMTab = Shuffleboard.getTab("Arm MM Testing");
    m_armkPEntry = armMMTab.add("kP_turn", 0 ).withPosition(1, 0).getEntry();
    m_kIEntry = armMMTab.add("kI", 0 ).withPosition(2, 0).getEntry();
    m_kDEntry = armMMTab.add("kD", 0 ).withPosition(3, 0).getEntry();
    m_kFEntry = armMMTab.add("kF", 0 ).withPosition(0, 0).getEntry();
    armMMTab.addNumber("MM Error", m_intake::getMMError).withPosition(4, 0);
    armMMTab.addBoolean("MM On Target", m_intake::isOnTarget).withPosition(4, 1);
    //m_iterationEntry = turnMMTab.add("stable iteration before finishing", 5 ).withPosition(0, 1).getEntry();
    m_targetPosEntry = armMMTab.add("target position", 0).withPosition(4, 0).getEntry();
    armMMTab.addNumber("Encoder", m_intake::getRelativeEncoder).withPosition(1, 1);
    //m_countokEntry = turnMMTab.add("count_ok", 0).getEntry();
    m_arbFFEntry = armMMTab.add("Arbitrary Feedforward", 0).withPosition(7, 0).getEntry();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isFinished = false;
    m_count_done = 0;
    m_targetPosition = (int) m_targetPosEntry.getDouble(0.0);
    m_kF = m_kFEntry.getDouble(0.0);
    m_arm_kP = m_armkPEntry.getDouble(0.0);
    m_kI = m_kIEntry.getDouble(0.0);
    m_kD = m_kDEntry.getDouble(0.0);
    m_downFF = m_arbFFEntry.getDouble(0.0);
    m_intake.configStartMM(m_targetPosition, m_arm_kP, m_kI, m_kD, m_kF);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_isFinished = m_intake.moveMM(m_targetPosition);
    if(m_isFinished){
      m_count_done++;
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.holdMotorPosition(m_targetPosition, m_downFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_count_done >= 3;
  }
}
