// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static frc.robot.Constants.kEncoderTicksPerDegree;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  DriveTrain m_driveTrain;
  double tolerance;
  double m_arc_length_ticks;
  int STABLE_ITERATIONS_BEFORE_FINISHED = 5;
  double m_count = 0;

  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveTrain driveTrain, double angle) {
    m_driveTrain = driveTrain;
    m_arc_length_ticks = angle * kEncoderTicksPerDegree;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //arclengthDegrees = m_arclengthEntry.getDouble(0);
    //see 2020 or 2019 code for explanation on these calculations
    m_count = 0;
    m_driveTrain.motionMagicStartConfigsTurn((m_arc_length_ticks < 0), m_arc_length_ticks);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_driveTrain.feedWatchdog(); is this needed?
    if (m_driveTrain.motionMagicTurn(m_arc_length_ticks)){
      m_count++;
    } else {
      m_count = 0;
    }
    // m_countokEntry.setDouble(count);

    // if (Math.abs(arclengthDegrees - currentAngle) < tolerance){
    //   m_driveTrain.teleop_drive(0, 0);
    //   isDone = true;
    // } else if (currentAngle < arclengthDegrees){
    //   m_driveTrain.teleop_drive(0, speed);
    // } else if (currentAngle > arclengthDegrees){
    //   m_driveTrain.teleop_drive(0, -speed);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_driveTrain.motionMagicEndConfigsTurn();
    //double drive_duration = Timer.getFPGATimestamp() - m_start_time;
    //m_drivedurationEntry.setDouble(drive_duration);
    m_driveTrain.teleop_drive(0, 0);
    m_driveTrain.motion_magic_end_config_turn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_count >= STABLE_ITERATIONS_BEFORE_FINISHED;
  }
}
