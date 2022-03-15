// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnGyroWithPID extends CommandBase {
  DriveTrain m_drivetrain;
  double m_target_angle, m_current_angle, m_setpoint_angle;

  /** Creates a new TurnGyroWithPID. */
  public TurnGyroWithPID(DriveTrain dt, double angle) {
    m_drivetrain = dt;
    m_target_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_current_angle = m_drivetrain.getHeading();
    m_setpoint_angle = m_current_angle + m_target_angle;
    m_drivetrain.resetGyroPIDController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.driveGyroPID(m_setpoint_angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.teleop_drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.atGyroPIDSetpoint();
  }
}
