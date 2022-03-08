// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static frc.robot.Constants.kEncoderTicksPerDegree;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngleTest extends CommandBase {
  DriveTrain m_driveTrain;
  public static double currentAngle;
  double m_target_angle;
  double tolerance;
  double speed;
  int count;
  private double m_start_time;
  double m_arcLengthticks;
  int STABLE_ITERATIONS_BEFORE_FINISHED = 5;
  // public final ShuffleboardTab turnMMTab;
  private double m_turn_kP, m_kI, m_kD;
  private NetworkTableEntry m_turnkPEntry, m_kIEntry, m_kDEntry, m_arclengthEntry, m_iterationEntry, m_drivedurationEntry, m_arclengthticksEntry;
  /** Creates a new TurnToAngle. */
  public TurnToAngleTest(DriveTrain driveTrain, double angle) {
    m_driveTrain = driveTrain;
    m_target_angle = angle;
    NetworkTable driveTab = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive Testing");
    m_turnkPEntry = driveTab.getEntry("kP");
    m_kIEntry = driveTab.getEntry("kI");
    m_kDEntry = driveTab.getEntry("kD");
    //m_kFEntry = driveTab.getEntry("kF");
    m_iterationEntry = driveTab.getEntry("Finish Iterations");
    m_arclengthEntry = driveTab.getEntry("Tgt. Degrees");
    m_arclengthticksEntry = driveTab.getEntry("Tgt. Ticks");
    m_drivedurationEntry = driveTab.getEntry("Run Time");
    m_kIEntry = driveTab.getEntry("kI");
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arcLengthticks = m_arclengthEntry.getDouble(0) * kEncoderTicksPerDegree;
    m_arclengthticksEntry.setDouble(m_arcLengthticks);
    m_turn_kP = m_turnkPEntry.getDouble(0.0);
    m_kI = m_kIEntry.getDouble(0.0);
    m_kD = m_kDEntry.getDouble(0.0);
    STABLE_ITERATIONS_BEFORE_FINISHED = (int) m_iterationEntry.getDouble(5.0);
    m_start_time = Timer.getFPGATimestamp();
    count = 0;
    m_driveTrain.reset_turn_PID_values(m_turn_kP, m_kI, m_kD);
    m_driveTrain.motionMagicStartConfigsTurn((m_arcLengthticks < 0), m_arcLengthticks);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_driveTrain.feedWatchdog(); is this needed?
    if (m_driveTrain.motionMagicTurn(m_arcLengthticks)){
      count++;
    } else {
      count = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_driveTrain.motionMagicEndConfigsTurn();
    double drive_duration = Timer.getFPGATimestamp() - m_start_time;
    m_drivedurationEntry.setDouble(drive_duration);
    m_driveTrain.teleop_drive(0, 0);
    m_driveTrain.motion_magic_end_config_turn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= STABLE_ITERATIONS_BEFORE_FINISHED;
  }
}
