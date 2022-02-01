// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.*;

public class DriveForSecondsFromShuffleboard extends CommandBase {
  Timer m_timer;
  DriveTrain m_driveTrain;
  ShuffleboardTab m_dTab = Shuffleboard.getTab("Default Drive Tab");
  NetworkTableEntry m_elapsed_entry, m_speed_entry, m_right_distance, m_inches_off;
  private double m_elapsed, m_speed;
  /** Creates a new DriveFiveSeconds. */
  public DriveForSecondsFromShuffleboard(DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveTrain = dt;

    addRequirements(m_driveTrain);
    m_elapsed_entry = m_dTab.add("Elapsed", 0).withPosition(4, 0).withSize(1, 1).getEntry();
    m_speed_entry = m_dTab.add("Speed", 0).withPosition(5, 0).withSize(1, 1).getEntry();
    m_right_distance = m_dTab.add("R Distance in Inches", 0).withPosition(6, 0).withSize(1, 1).getEntry();
    m_inches_off = m_dTab.add("Inches_off", 0).withPosition(7, 0).withSize(1, 1).getEntry();

    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elapsed = m_elapsed_entry.getDouble(0);
    m_speed = m_speed_entry.getDouble(0);

    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.teleop_drive(m_speed, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.teleop_drive(0, 0);

    double rightenc = m_driveTrain.getRightEncoderValue();
    double leftenc = m_driveTrain.getLeftEncoderValue();

    m_right_distance.forceSetDouble(rightenc / kEncoderTicksPerInch);
    double off = Math.abs(Math.abs(rightenc) - Math.abs(leftenc));
    m_inches_off.forceSetDouble(off/kEncoderTicksPerInch);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_elapsed);
  }
}
