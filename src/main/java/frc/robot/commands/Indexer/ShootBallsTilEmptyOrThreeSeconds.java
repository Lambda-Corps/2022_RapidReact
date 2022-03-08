// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class ShootBallsTilEmptyOrThreeSeconds extends CommandBase {

  Indexer m_indexer;
  Timer m_timer;

  /** Creates a new Index_Balls_into_FlyWheel. */
  public ShootBallsTilEmptyOrThreeSeconds(Indexer indexer) {
    m_indexer = indexer;
    m_timer = new Timer();
    m_timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexer.shootBalls();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean done = m_indexer.isEmpty();
    if(m_timer.hasElapsed(3)){
      done = true;
    }
  
    return done;
  }
}
