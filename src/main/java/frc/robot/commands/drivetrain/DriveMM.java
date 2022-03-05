// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveMM extends CommandBase {
  /** Creates a new DriveMM. */
  DriveTrain m_driveTrain;
  ShuffleboardTab driveMMTab;
  double m_targetPosition;
  double m_targetTicks;
  int count;
  double m_start_time;
  double m_drive_kP, m_kI, m_kD, m_kF;
  // NetworkTableEntry m_drivekPEntry, m_kIEntry, m_kDEntry, m_kFEntry, m_targetPosEntry, m_targetTicksEntry, m_iterationEntry, m_drivedurationEntry, m_countokEntry;
  //the number of times motion magic is on target before the command finishes
  int STABLE_ITERATIONS_BEFORE_FINISHED = 5;
  public DriveMM(DriveTrain driveTrain, double targetInches) {
    m_driveTrain = driveTrain;
    m_targetPosition = targetInches;
    // driveMMTab = Shuffleboard.getTab("Drive MM Testing");
    // m_drivekPEntry = driveMMTab.add("kP_drive", 0 ).withPosition(1, 0).getEntry();
    // m_kIEntry = driveMMTab.add("kI", 0 ).withPosition(2, 0).getEntry();
    // m_kDEntry = driveMMTab.add("kD", 0 ).withPosition(3, 0).getEntry();
    // m_kFEntry = driveMMTab.add("kF", 0 ).withPosition(0, 0).getEntry();
    // m_iterationEntry = driveMMTab.add("stable iteration before finishing", 5 ).withPosition(0, 1).getEntry();
    // m_targetPosEntry = driveMMTab.add("target position", 0).withPosition(4, 0).getEntry();
    // m_targetTicksEntry = driveMMTab.add("target ticks", 0).getEntry();
    // driveMMTab.addNumber("Left Encoder", m_driveTrain::getLeftEncoderValue).withPosition(1, 1);
    // driveMMTab.addNumber("Right Encoder", m_driveTrain::getRightEncoderValue).withPosition(2,1);
    // m_drivedurationEntry = driveMMTab.add("drive duration", 0).withPosition(6, 0).getEntry();
    // m_countokEntry = driveMMTab.add("count_ok", 0).getEntry();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.disableMotorSafety();
    // m_kF = //m_kFEntry.getDouble(0.003699);
    // m_drive_kP = //m_drivekPEntry.getDouble(0.0);
    // m_kI = //m_kIEntry.getDouble(0.0);
    // m_kD = //m_kDEntry.getDouble(0.0);
    // STABLE_ITERATIONS_BEFORE_FINISHED = (int) m_iterationEntry.getDouble(5.0);
    // m_targetPosition = //m_targetPosEntry.getDouble(0.0);
    m_targetTicks = m_targetPosition * 1108.23;
    // m_targetTicksEntry.forceSetDouble((int) m_targetTicks);
    count = 0;
    m_driveTrain.setEncodersToZero();
    //m_driveTrain.reset_drive_PID_values(m_drive_kP, m_kI, m_kD);
    m_driveTrain.motion_magic_start_config_drive(m_targetTicks >= 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_driveTrain.motionMagicDrive(m_targetTicks)){
      count++;
    } else {
      count = 0;
    }
    //m_countokEntry.setDouble(count);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.teleop_drive(0, 0);
    m_driveTrain.enableMotorSafety();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= STABLE_ITERATIONS_BEFORE_FINISHED;
  }
}
