// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ArmMM extends CommandBase {
  /** Creates a new ArmMM. */
  Intake m_intake;
  int m_targetPosition;
  // public final ShuffleboardTab armMMTab;
  // private double m_arm_kP, m_kI, m_kD, m_kF;
  // private NetworkTableEntry m_armkPEntry, m_kIEntry, m_kDEntry, m_targetPosEntry, m_kFEntry, m_arbFFEntry, m_MMError;
  // private double m_start_time;
  // private double m_upFF, m_downFF;
  private boolean m_isFinished;
  private int m_count_done;
  private Timer m_timer;

  public ArmMM(Intake intake, int target_position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_timer = new Timer();
    m_timer.start();

    addRequirements(m_intake);
    // m_targetPosition = target_position;
    m_targetPosition = Intake.INTAKE_ARM_EXTEND;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_isFinished = false;
    m_count_done = 0;
    // m_targetPosition = (int) m_targetPosEntry.getDouble(0.0);
    // m_kF = m_kFEntry.getDouble(0.0);
    // m_arm_kP = m_armkPEntry.getDouble(0.0);
    // m_kI = m_kIEntry.getDouble(0.0);
    // m_kD = m_kDEntry.getDouble(0.0);
    // m_downFF = m_arbFFEntry.getDouble(0.0);
    // m_intake.configStartMM(m_targetPosition, m_arm_kP, m_kI, m_kD, m_kF);
    m_intake.configStartMM(m_targetPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_isFinished = m_intake.moveMM(m_targetPosition);
    if(m_isFinished){
      m_count_done++;
    }
    else{
      m_count_done = 0;
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.holdMotorPosition(m_targetPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_count_done >= 10 || m_timer.hasElapsed(1);
  }

}
