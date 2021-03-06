// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class TestClimberUp extends CommandBase {
  /** Creates a new TestClimber. */

  Climber m_climber;
  NetworkTableEntry m_raiseSpeed;

  double m_speed;

  public TestClimberUp(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;

    addRequirements(m_climber);

    NetworkTable climberTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Climber");
    //m_lowerSpeed = -1; 
    m_raiseSpeed = climberTab.getEntry("ClimberUpSpeed");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //if (!m_climber.forwardLimitSwitchTriggered()) {
    //  m_climber.climberIsAtHighest();
    //}

    m_speed = m_raiseSpeed.getDouble(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.setClimberMotor(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.climberStopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climber.forwardLimitSwitchTriggered();
  }
}
