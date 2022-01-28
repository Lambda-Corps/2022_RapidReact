// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Moveforwarddistance extends CommandBase {
  boolean m_isdone;
  DriveTrain m_driveTrain;
  private double m_target_position;
  private double m_target_position_ticks;
  /** Creates a new Moveforwarddistance. */
  public Moveforwarddistance(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isdone = false;
    double current_position = m_driveTrain.getLeftEncoderValue();
    m_target_position = 108;
    m_target_position_ticks = current_position + (m_target_position * 1086.49774484);
    SmartDashboard.putNumber("targetposition", m_target_position_ticks);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.teleop_drive(.15, 0);
    m_isdone = m_target_position_ticks <= m_driveTrain.getLeftEncoderValue() ? true : false;
    if (m_isdone) {
      m_driveTrain.teleop_drive(0, 0);
      SmartDashboard.putNumber("positionafter", m_driveTrain.getLeftEncoderValue());
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isdone;
  }
}
