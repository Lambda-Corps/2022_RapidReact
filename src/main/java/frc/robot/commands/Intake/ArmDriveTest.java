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

public class ArmDriveTest extends CommandBase {
  /** Creates a new ArmUpTest. */
  Intake m_intake;

  ShuffleboardTab m_forwardTestTab;
  NetworkTableEntry m_driveSpeed, m_driveTimer;
  Timer timer;
  public ArmDriveTest(Intake intake) {
    m_forwardTestTab = Shuffleboard.getTab("Arm Drive Testing");
    m_driveSpeed = m_forwardTestTab.add("Drive Speed", 0).getEntry();
    m_driveTimer = m_forwardTestTab.add("Drive Timer", 0).getEntry();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setMotor(m_driveSpeed.getDouble(0));
    m_driveTimer.forceSetDouble(timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setMotor(0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
