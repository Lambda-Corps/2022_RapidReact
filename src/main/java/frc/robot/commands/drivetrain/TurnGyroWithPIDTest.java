// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnGyroWithPIDTest extends CommandBase {
  DriveTrain m_drivetrain;
  double m_target_angle, m_current_angle, m_setpoint_angle, m_start_time;

  private NetworkTableEntry m_turnkPEntry, m_kIEntry, m_kDEntry, m_arclengthEntry, m_drivedurationEntry, m_speed_entry;
  
  /** Creates a new TurnGyroWithPID. */
  public TurnGyroWithPIDTest(DriveTrain dt, double angle) {
    m_drivetrain = dt;
    m_target_angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);

    NetworkTable driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Testing");
    m_turnkPEntry = driveTab.getEntry("kP");
    m_kIEntry = driveTab.getEntry("kI");
    m_kDEntry = driveTab.getEntry("kD");
    m_arclengthEntry = driveTab.getEntry("Tgt. Degrees");
    m_drivedurationEntry = driveTab.getEntry("Run Time");
    m_speed_entry = driveTab.getEntry("Drive Max");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_current_angle = m_drivetrain.getRawAngle();
    m_setpoint_angle = m_current_angle + m_arclengthEntry.getDouble(0);
    m_drivetrain.resetGyroPIDController(m_turnkPEntry.getDouble(0), m_kIEntry.getDouble(0), m_kDEntry.getDouble(0));
    m_start_time = Timer.getFPGATimestamp();
    m_drivetrain.resetGyroPIDTurnSpeed(m_speed_entry.getDouble(0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.driveGyroPID(m_setpoint_angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double drive_duration = Timer.getFPGATimestamp() - m_start_time;
    m_drivedurationEntry.setDouble(drive_duration);
    m_drivetrain.teleop_drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.atGyroPIDSetpoint();
  }
}
