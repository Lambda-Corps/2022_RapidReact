// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class CollectBalls extends CommandBase {
  private final Intake m_intake;
  private final Indexer m_indexer;
  private double m_count_done;
  /** Creates a new CollectBalls. */
  public CollectBalls(Intake intake, Indexer indexer) {
    m_intake = intake;
    m_indexer = indexer;
    m_count_done = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexer.setIntakingBallsTrue();
    m_count_done = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run the intake wheels
    m_intake.collectBalls();
    // Have indexer check and resolve
    m_indexer.checkIndexState();
    m_indexer.resolveIndexer();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      System.out.println("Interrupted the CollectBalls");
    }
    m_indexer.setIntakingBallsFalse();
    m_indexer.stopMotors();
    m_intake.stopIntakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_indexer.isFull()){
      m_count_done++;
    }
    if(m_count_done >= 25){
      return true;
    }
    else{
      return false;
    }
  }
}
