// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class EjectBalls extends CommandBase {
  Indexer m_Indexer;
  Shooter m_shooter;
  Intake m_intake;
  /** Creates a new EjectBalls. */
  public EjectBalls(Indexer indexer, Shooter shooter, Intake intake) {
    m_Indexer = indexer;
    m_shooter = shooter;
    m_intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Indexer, m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_Indexer.ejectBallsBackward();
    m_shooter.ejectBallsBackward();
    m_intake.ejectBalls();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Indexer.stopMotors();
    m_shooter.stopMotor();
    m_intake.stopIntakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
