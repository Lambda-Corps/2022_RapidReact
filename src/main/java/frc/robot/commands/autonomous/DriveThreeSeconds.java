// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drivetrain.DriveMM;
import frc.robot.subsystems.DriveTrain;

public class DriveThreeSeconds extends CommandBase {
  /** Creates a new DriveThreeSeconds. */
  DriveTrain m_drive_train;
  Timer m_timer;
  double currTime;
  public DriveThreeSeconds(DriveTrain driveTrain) {
    m_drive_train = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive_train);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive_train.tank_drive_straight(-0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive_train.teleop_drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(3);
  }
}
