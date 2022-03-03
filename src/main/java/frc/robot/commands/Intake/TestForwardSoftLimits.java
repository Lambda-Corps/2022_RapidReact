// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class TestForwardSoftLimits extends CommandBase {
  /** Creates a new TestSoftLimits. */
  Intake m_intake;
  ShuffleboardTab m_intakeTab;
  NetworkTableEntry m_encoderPos_entry;
  public TestForwardSoftLimits(Intake intake) {
    m_intake = intake;
    m_encoderPos_entry = NetworkTableInstance.getDefault().getTable("Intake").getEntry("Arm Encoder Position");

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_encoderPos = m_encoderPos_entry.getDouble(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
