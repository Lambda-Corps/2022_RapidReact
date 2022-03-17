// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LEDsubsystem;

public class LowerClimber extends CommandBase {
  /** Creates a new LowerClimber. */

  Climber m_climber;
  LEDsubsystem m_LEDsubsystem;

  public LowerClimber(Climber climber, LEDsubsystem ledsubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    m_climber = climber;
    m_LEDsubsystem = ledsubsystem;

    addRequirements(climber, ledsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LEDsubsystem.resetClimberLEDInformation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.setClimberMotor(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.climberStopMotor();
    if (m_climber.reverseLimitSwitchTriggered()) {
      if (m_LEDsubsystem.checkClimbInProgress()) {
        m_LEDsubsystem.climbFinished();
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.reverseLimitSwitchTriggered();
  }
}
