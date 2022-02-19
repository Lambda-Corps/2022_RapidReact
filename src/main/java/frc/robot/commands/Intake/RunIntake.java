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

public class RunIntake extends CommandBase {
  /** Creates a new RunIntake. */
  Intake m_intake;
  ShuffleboardTab intakeTab;
  double m_start_time, m_curr_time, desiredTime;
  double speed;
  NetworkTableEntry m_speedEntry, m_durationEntry;
  public RunIntake(Intake intake) {
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
    intakeTab = Shuffleboard.getTab("Intake Testing");
    m_speedEntry = intakeTab.add("Intake Indexer Speed", 0).withPosition(2, 2).getEntry();
    m_durationEntry = intakeTab.add("Intake duration", 0).withPosition(2, 1).getEntry();
    m_start_time = Timer.getFPGATimestamp();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = m_speedEntry.getDouble(0.0);
    if(speed > 0.39){
      speed = 0.39;
    }
    desiredTime = m_durationEntry.getDouble(0.0);
    m_intake.intakeMotorOn(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_curr_time = Timer.getFPGATimestamp() - m_start_time;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_curr_time > desiredTime;
  }
}
