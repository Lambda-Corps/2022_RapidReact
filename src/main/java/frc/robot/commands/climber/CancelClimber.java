// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LEDsubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CancelClimber extends InstantCommand {
  Climber m_climber;
  LEDsubsystem m_LEDsubsystem;
  public CancelClimber(Climber climber, LEDsubsystem ledsubsystem) {
    m_climber = climber;
    m_LEDsubsystem = ledsubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber, ledsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.climberStopMotor();
    m_LEDsubsystem.setClimbInProgress(0);
    m_LEDsubsystem.resetClimberLEDInformation(2);
  }
}
