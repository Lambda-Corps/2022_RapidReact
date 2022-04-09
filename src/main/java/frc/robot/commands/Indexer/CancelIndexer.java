// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CancelIndexer extends InstantCommand {
  Indexer m_indexer;
  Intake m_intake;
  public CancelIndexer(Indexer indexer, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_indexer = indexer;
    m_intake = intake;
    addRequirements(m_indexer, m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexer.stopMotors();
    m_intake.stopIntakeMotor();
  }
}
