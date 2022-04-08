// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.combined;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LEDsubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StopShooterAndIndexerMotors extends InstantCommand {
  Shooter m_shooter;
  Indexer m_indexer;
  LEDsubsystem m_LEDsubsystem;
  public StopShooterAndIndexerMotors(Shooter shooter, Indexer indexer, LEDsubsystem ledsubsystem) {
    m_shooter = shooter;
    m_indexer = indexer;
    m_LEDsubsystem = ledsubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_indexer, m_LEDsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.stopMotor();
    m_indexer.stopMotors();
    m_LEDsubsystem.shooterActive(0);
  }
}
