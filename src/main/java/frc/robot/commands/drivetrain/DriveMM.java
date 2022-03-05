// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  NetworkTableEntry m_drivekPEntry, m_kIEntry, m_kDEntry, m_kFEntry, m_targetPosEntry, m_targetTicksEntry, m_iterationEntry, m_drivedurationEntry, m_countokEntry;
  //the number of times motion magic is on target before the command finishes
  int STABLE_ITERATIONS_BEFORE_FINISHED = 5;
  public DriveMM(DriveTrain driveTrain, double targetInches) {
    m_driveTrain = driveTrain;
    m_targetPosition = targetInches;
    NetworkTable driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive MM Testing");

    m_drivekPEntry = driveTab.getEntry("kP_drive");
    m_kIEntry = driveTab.getEntry("kI");
    m_kDEntry = driveTab.getEntry("kD");
    m_kFEntry = driveTab.getEntry("kF");
    m_iterationEntry = driveTab.getEntry("stable iteration before finishing");
    m_targetPosEntry = driveTab.getEntry("target position");
    m_targetTicksEntry = driveTab.getEntry("target ticks");
    m_drivedurationEntry = driveTab.getEntry("drive duration");
    m_countokEntry = driveTab.getEntry("count_ok");
    m_kIEntry = driveTab.getEntry("kI");
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.disableMotorSafety();
    m_kF = m_kFEntry.getDouble(0.003699);
    m_drive_kP = m_drivekPEntry.getDouble(0.0);
    m_kI = m_kIEntry.getDouble(0.0);
    m_kD = m_kDEntry.getDouble(0.0);
    STABLE_ITERATIONS_BEFORE_FINISHED = (int) m_iterationEntry.getDouble(5.0);
    //m_targetPosition = m_targetPosEntry.getDouble(0.0);
    m_targetTicks = m_targetPosition * 1108.23;
    m_targetTicksEntry.forceSetDouble((int) m_targetTicks);
    count = 0;
    m_driveTrain.setEncodersToZero();
    m_driveTrain.reset_drive_PID_values(m_drive_kP, m_kI, m_kD);
    m_driveTrain.motion_magic_start_config_drive(m_targetTicks >= 0);
    m_drive_kP = 0.3;
    m_kF = 0.1;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_driveTrain.motionMagicDrive(m_targetTicks)){
      count++;
    } else {
      count = 0;
    }
    m_countokEntry.setDouble(count);
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
