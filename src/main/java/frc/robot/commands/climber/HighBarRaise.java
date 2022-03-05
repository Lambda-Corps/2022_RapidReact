// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class HighBarRaise extends CommandBase {
  /** Creates a new HighBarRaise. */

  private final XboxController m_driverController;

  Climber m_climber;

  public HighBarRaise(Climber climber, XboxController driverController) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_climber = climber;
    m_driverController = driverController;

    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_climber.reverseLimitSwitchTriggered() && m_climber.getRelativeEncoder() != 0) {
      m_climber.resetClimberMotorEncoder();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.setClimberMotor(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.climberStopMotor();
    m_driverController.setRumble(GenericHID.RumbleType.kRightRumble, 1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.forwardLimitSwitchTriggered();
  }
}