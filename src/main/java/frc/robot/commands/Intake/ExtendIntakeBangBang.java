// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ExtendIntakeBangBang extends CommandBase {
  /** Creates a new ExtendIntakeBangBang. */
  Intake m_intake;
  boolean m_finished;
  int m_targetPos;
  public ExtendIntakeBangBang(Intake intake, int targetPosition) { //drops arm to given position without holding it
    // Use addRequirements() here to declare subsystem dependencies.\       ^ then gravity will do the rest (strap holds position)
    m_intake = intake;
    m_targetPos = targetPosition;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_finished = false;
    m_intake.turnOnArmMotor(0.6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_finished = m_intake.getIntakeArmPosition() >= (m_targetPos / 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.turnOffArmMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
