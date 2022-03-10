// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ResetArmLimitAndEncoder extends CommandBase {
  private final Intake m_intake;
  private boolean m_isDone;

  /** Creates a new ResetArmLimitAndEncoder. */
  public ResetArmLimitAndEncoder(Intake intake) {
    m_intake = intake;
    m_isDone = false;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isDone = false;
    m_intake.disableArmMotorSoftLimits();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_isDone = m_intake.driveMotorToLimitSwitch();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.holdMotorPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isDone;
  }
}
