// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDsubsystem;
import frc.robot.subsystems.Climber;

public class LowBarLower extends CommandBase {
  /** Creates a new LowBarLower. */

  Climber m_climber;
  Timer m_timer;
  LEDsubsystem m_LEDsubsystem;

  public LowBarLower(Climber climber, LEDsubsystem ledsubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_LEDsubsystem = ledsubsystem;

    addRequirements(climber, ledsubsystem);

    m_timer = new Timer();
    m_timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_LEDsubsystem.resetClimberLEDInformation(2);
    if (m_LEDsubsystem.checkDriverSignalActive()) {
      m_LEDsubsystem.blackout();
      m_LEDsubsystem.resetDriverSignal();
    }
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
    // m_LEDsubsystem.updateClimberLEDInformation(0);
    if (m_LEDsubsystem.checkClimbInProgress()) {
      m_LEDsubsystem.climbFinished();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(5);
  }
}
