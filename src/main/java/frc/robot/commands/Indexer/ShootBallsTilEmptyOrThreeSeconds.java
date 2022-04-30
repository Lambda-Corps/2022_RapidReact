// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LEDsubsystem;

public class ShootBallsTilEmptyOrThreeSeconds extends CommandBase {

  Indexer m_indexer;
  Shooter m_shooter;
  Timer m_timer;
  int m_emptycount;
  LEDsubsystem m_LEDsubsystem;

  /** Creates a new Index_Balls_into_FlyWheel. */
  public ShootBallsTilEmptyOrThreeSeconds(Indexer indexer, Shooter shooter, LEDsubsystem ledsubsystem) {
    m_indexer = indexer;
    m_shooter = shooter;
    m_LEDsubsystem = ledsubsystem;
    m_timer = new Timer();
    m_timer.start();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_indexer, m_LEDsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_emptycount = 0;
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_LEDsubsystem.blackout();
    //m_LEDsubsystem.shooterActive(1);
    m_indexer.shootBalls();
    if(m_indexer.isEmpty()){
      m_emptycount++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stopMotors();
    m_shooter.stopMotor();
    //m_LEDsubsystem.shooterActive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean done = false;
    // If we've been empty for more than .4 seconds, or 
    // the timer elapsed, end the command
    if(m_timer.hasElapsed(3) || m_emptycount >= 20){
      done = true;
    }
  
    return done;
  }
}
