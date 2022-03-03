// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;

public class VisionAlign extends CommandBase {
  /** Creates a new VisionAlign. */
  Vision m_vision;
  NetworkTableEntry isWorking, timeElasped;
  ShuffleboardTab m_visionTab;
  Timer timer;

  private String working;
  private double elapsed;

  public VisionAlign(Vision vision) {
    m_visionTab = Shuffleboard.getTab("Vision");
    m_vision = vision;
    isWorking = m_visionTab.add("Command Running", "Not Running").getEntry();
    timeElasped = m_visionTab.add("Time Elapsed", 0).getEntry();

    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    working = "Running Command";
    isWorking.setString(working);
    
    int targetIndex = 4;
    m_vision.targetMode(targetIndex);

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elapsed = timer.get();
    timeElasped.setDouble(elapsed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_vision.setTeamPipeline();
    working = "Not Running";
    isWorking.setString(working);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (elapsed >= 5.0);
  }
}
