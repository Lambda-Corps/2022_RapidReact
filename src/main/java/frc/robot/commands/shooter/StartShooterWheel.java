// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LEDsubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StartShooterWheel extends InstantCommand {
  Shooter m_shooter;
  LEDsubsystem m_LEDsubsystem;
  public StartShooterWheel(Shooter shooter, LEDsubsystem ledsubsystem) {
    m_shooter = shooter;
    m_LEDsubsystem = ledsubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_LEDsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LEDsubsystem.blackout();
    m_LEDsubsystem.shooterActive(1);
    m_shooter.startVelocityPID();
  }
}
