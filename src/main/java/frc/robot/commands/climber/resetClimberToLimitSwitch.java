// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class resetClimberToLimitSwitch extends CommandBase {
  /** Creates a new resetClimberToLimitSwitch. */

  Climber m_climber;

  public resetClimberToLimitSwitch(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_climber = climber;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!m_climber.reverseLimitSwitchTriggered() || !m_climber.forwardLimitSwitchTriggered()) {
      m_climber.resetClimberPositionEncoder();
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
    m_climber.resetClimberMotorEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.reverseLimitSwitchTriggered();
  }
}
