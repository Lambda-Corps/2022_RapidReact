// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShooterPIDTuning extends CommandBase {
  private double m_setpoint, m_tolerance, m_indexerSpeed, m_indexerDelay, m_Intake, m_Indexer, m_runTime;
  private Timer cmdTimer;

  private final Shooter m_shooter;
  private final Indexer m_indexer;
  private final ShuffleboardTab m_myTab;
  private final NetworkTableEntry m_kpEntry, m_kiEntry, m_kdEntry, m_kfEntry, m_spEntry,m_indexerSpeedEntry, m_indexderDelayEntry, m_runTimeEntry;
  /**
   * Creates a new PIDTuningCommand.
   */
  /** Creates a new ShooterPIDTuning. */
  public ShooterPIDTuning(Shooter shooter, Indexer indexer) {
    m_shooter = shooter;
    m_indexer = indexer;

    cmdTimer = new Timer(); // used to delay the conveyor if necessary

    // This sets up all the shuffleboard components needed for testing on the PID Tuning Tab
    m_myTab = Shuffleboard.getTab("ShooterPID");

    // Get the rest of shuffleboard commands
    // The general process to tune the velocity PID is as follows:
    // 1) Determine (with Phoenix Tuner) what is the RPM for the desired shot,
    //    note the PercentOutput required to generate that speed.
    //    this is your kF value you will use for the rest of the tuning.
    // 2) Start with a small kP, and 0 for kI and kD values.  A starting kP 
    //    of 1 is reasonable.
    // 3) If the kP value is too large there will be some semi-violent-ish 
    //    recovery on the motor (which you can see visually, or in the graph
    //    on Phoenix Tuner), if the kP value is too small, the second and
    //    subsequent shots will not be the proper trajectory (meaning the motor
    //    output is not recovering quickly enough with the PID). Increase/Decrease
    //    by doubling (or halving) it's value until the subsequent shot trajectories
    //    match the first.
    // 4) If for some reason the recover from kP looks "jerky" or not smooth, then 
    //    try introducing the kD value.  Typically a starting kD value is 5x - 10x kP.
    //
    //    For a velocity PID, it's unlikely you'll need anything but kF and kD.
    m_kfEntry = m_myTab.add("kF", 0 ).withPosition(0, 0).getEntry();
    m_kpEntry = m_myTab.add("kP", 0).withPosition(1, 0).getEntry();
    m_kiEntry = m_myTab.add("kI", 0 ).withPosition(2, 0).getEntry();
    m_kdEntry = m_myTab.add("kD", 0 ).withPosition(3, 0).getEntry();
    m_spEntry = m_myTab.add("Set Point", 0 ).withPosition(4, 0).getEntry();
    m_indexerSpeedEntry = m_myTab.add("Indexer Speed", .8).withPosition(0, 1).getEntry();
    m_indexderDelayEntry = m_myTab.add("Indexer Delay", 2).withPosition(1, 1).getEntry();
    m_runTimeEntry = m_myTab.add("runtime", 0).withPosition(2, 1).getEntry();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter, m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        // Grab the relevant values for the PID control from Shuffleboard and set the 
    // motor controller configuration accordingly
    double kp = m_kpEntry.getDouble(.0999);
    double ki = m_kiEntry.getDouble(0);
    double kd = m_kdEntry.getDouble(0);
    double kf = m_kfEntry.getDouble(.0534);
    m_setpoint = m_spEntry.getDouble(0);
    m_indexerSpeed = m_indexerSpeedEntry.getDouble(0);
    m_indexerDelay = m_indexderDelayEntry.getDouble(2.0);
    m_runTime = m_runTimeEntry.getDouble(0);
    m_Indexer = m_indexerSpeedEntry.getDouble(0);
    m_shooter.configureVelocityPID(kp, ki, kd, kf);

    cmdTimer.reset();
    cmdTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive the shooter motors, as well as the conveyor to start the 
    m_shooter.velocityPID(m_setpoint, m_tolerance);
    if( cmdTimer.hasElapsed(m_indexerDelay)){
      // Turn on the conveyor motor after the delay has been meet.
      m_indexer.testIndexDriving(m_indexerSpeed, m_indexerSpeed, m_indexerSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO finish
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO finish
    return false;
  }
}
