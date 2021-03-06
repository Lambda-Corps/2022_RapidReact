// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class ExtendIntakeBangBang extends CommandBase {
  /** Creates a new ExtendIntakeBangBang. */
  Intake m_intake;
  boolean m_finished;
  int m_targetPos;
  boolean m_hold;
  Timer m_timer;
  public ExtendIntakeBangBang(Intake intake, int targetPosition) { //drops arm to given position without holding it
    // Use addRequirements() here to declare subsystem dependencies.\       ^ then gravity will do the rest (strap holds position)
    m_intake = intake;
    m_targetPos = targetPosition;
    m_hold = true;
    m_timer = new Timer();
    m_timer.start();
    addRequirements(m_intake);
  }

  public ExtendIntakeBangBang(Intake intake, int targetPosition, boolean hold) { //drops arm to given position without holding it
    // Use addRequirements() here to declare subsystem dependencies.\       ^ then gravity will do the rest (strap holds position)
    m_intake = intake;
    m_targetPos = targetPosition;
    m_hold = hold;
    m_timer = new Timer();
    m_timer.reset();
    m_timer.start();
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_finished = false;
    m_intake.turnOnArmMotor(0.45);
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_finished = m_intake.getIntakeArmPosition() >= (m_targetPos / 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!m_hold) {
      m_intake.turnOffArmMotor();
    } else {
      m_intake.holdMotorPosition(m_targetPos);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return m_finished;
  //   if (m_hold) {
  //     return m_finished;
  //   } else {
  //     return m_timer.hasElapsed(0.75);
  //   }
    return m_finished;
  }
}
