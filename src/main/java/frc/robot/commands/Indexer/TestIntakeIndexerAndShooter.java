// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TestIntakeIndexerAndShooter extends CommandBase {
  private final Indexer m_indexer;
  private final Intake m_intake;
  private final Shooter m_shooter;

  NetworkTableEntry m_in_speed_entry, m_mid_speed_entry, m_shoot_speed_entry, m_duration_entry, m_beamIndex_entry, m_beamShoot_entry, m_intakeWheels_entry, m_shotPower;

  double m_in_speed, m_mid_speed, m_shoot_speed, m_duration, m_intakeWheelSpeed, m_flywheelSpeed;

  Timer m_timer;
  /** Creates a new TestIndexerCommand. */
  public TestIntakeIndexerAndShooter(Indexer indexer, Intake intake, Shooter shooter) {
    m_indexer = indexer;
    m_intake = intake;
    m_shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_indexer, m_intake, m_shooter);

    // setup shuffleboard testing, tab created in subsystem
    ShuffleboardTab intab = Shuffleboard.getTab("Combined Test");

    m_in_speed_entry = intab.add("Index Entry Speed", 0).withPosition(0, 0)
                                .withSize(1, 1)
                                .getEntry();

    m_mid_speed_entry = intab.add("Mid Speed", 0).withPosition(1, 0)
                                .withSize(1, 1)
                                .getEntry();
    
    m_shoot_speed_entry = intab.add("Shooter Speed", 0).withPosition(2, 0)
                                .withSize(1, 1)
                                .getEntry();

    m_duration_entry = intab.add("Duration", 0).withPosition(3, 0)
                                .withSize(1, 1)
                                .getEntry();

    m_intakeWheels_entry = intab.add("Intake Wheel Speed", 0).withPosition(4, 0)
                                .withSize(1, 1)
                                .getEntry();

    m_shotPower = intab.add("Flywheel Speed", 0).withPosition(5, 0)
                                .withSize(1, 1)
                                .getEntry();
    intab.addNumber("test speed", this::getFlywheelSpeed).withPosition(6,0);

    m_timer = new Timer();
    m_timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Grab the Shuffleboard values
    m_in_speed = m_in_speed_entry.getDouble(0);
    m_mid_speed = m_mid_speed_entry.getDouble(0);
    m_shoot_speed = m_shoot_speed_entry.getDouble(0);
    m_duration = m_duration_entry.getDouble(0);
    m_intakeWheelSpeed = m_intakeWheels_entry.getDouble(0);
    m_flywheelSpeed = m_shotPower.getDouble(0);

    // Reset the clock
    m_timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.intakeWheelsMotorOn(m_intakeWheelSpeed);
    m_shooter.test_shooter_percent(m_flywheelSpeed);
    m_indexer.checkIndexState();
    m_indexer.resolveIndexer(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.testIndexDriving(0, 0, 0);
    m_intake.stopIntakeMotor();
    m_shooter.test_shooter_percent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Command shoot end if the values were 0, or the timer has elapsed.
    return m_timer.hasElapsed(m_duration) || 
           (m_in_speed == 0 && m_mid_speed == 0 && m_shoot_speed == 0);
  }

  public double getFlywheelSpeed() {
    return m_flywheelSpeed;
  }
}
