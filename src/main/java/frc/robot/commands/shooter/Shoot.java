// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LEDsubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotDistance;

public class Shoot extends CommandBase {
  private double m_setpoint, m_indexerDelay, m_runTime;
  private Timer cmdTimer;

  private final Shooter m_shooter;
  private final Indexer m_indexer;
  private final ShotDistance m_distance;

  LEDsubsystem m_LEDsubsystem;

  // private final NetworkTableEntry m_kpEntry, m_kiEntry, m_kdEntry, m_kfEntry, m_spEntry,m_indexerSpeedEntry, m_indexderDelayEntry, m_runTimeEntry;
  /**
   * Creates a new PIDTuningCommand.
   */
  /** Creates a new ShooterPIDTuning. */
  public Shoot(Shooter shooter, Indexer indexer, LEDsubsystem ledsubsystem, ShotDistance distance) {
    m_shooter = shooter;
    m_indexer = indexer;
    m_distance = distance;
    m_LEDsubsystem = ledsubsystem;
    m_runTime = 6;
    m_indexerDelay = 3;
    cmdTimer = new Timer(); // used to delay the conveyor if necessary
    cmdTimer.start();
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_indexer, m_LEDsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        // Grab the relevant values for the PID control from Shuffleboard and set the 
    m_shooter.setShotDistance(m_distance);
    m_shooter.setProfileSlot();
    cmdTimer.reset();
    cmdTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_LEDsubsystem.shooterActive(1);
    // Drive the shooter motors, as well as the conveyor to start the 
    m_shooter.velocityPID(m_setpoint);
    if( cmdTimer.hasElapsed(m_indexerDelay)){
      // Turn on the conveyor motor after the delay has been meet.
      m_indexer.shootBalls();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_LEDsubsystem.shooterActive(0);
    m_shooter.stopMotor();
    m_indexer.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmdTimer.hasElapsed(m_runTime);
  }
}
