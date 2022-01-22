// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.default_commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  DriveTrain m_driveTrain;
  public static double currentAngle;
  double arclengthDegrees;
  boolean isDone = false;
  double tolerance;
  double speed;
  int count;
  private double m_start_time;
  int arclengthTicks;
  int STABLE_ITERATIONS_BEFORE_FINISHED = 5;
  public final ShuffleboardTab turnMMTab;
  private double m_turn_kP, m_kI, m_kD, m_kF;
  private NetworkTableEntry m_turnkPEntry, m_kIEntry, m_kDEntry, m_arclengthEntry, m_iterationEntry, m_drivedurationEntry, m_countokEntry, m_arclengthticksEntry, m_kFEntry;
  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveTrain driveTrain, double speed, double angle) {
    m_driveTrain = driveTrain;
    arclengthDegrees = angle;
    this.speed = speed;
    turnMMTab = Shuffleboard.getTab("Turn MM Testing");
    m_turnkPEntry = turnMMTab.add("kP_turn", 0 ).withPosition(1, 0).getEntry();
    m_kIEntry = turnMMTab.add("kI", 0 ).withPosition(2, 0).getEntry();
    m_kDEntry = turnMMTab.add("kD", 0 ).withPosition(3, 0).getEntry();
    m_kFEntry = turnMMTab.add("kF", 0 ).withPosition(0, 0).getEntry();
    m_iterationEntry = turnMMTab.add("stable iteration before finishing", 5 ).withPosition(0, 1).getEntry();
    m_arclengthEntry = turnMMTab.add("target position", 0).withPosition(4, 0).getEntry();
    m_arclengthticksEntry = turnMMTab.add("target ticks", 0).getEntry();
    turnMMTab.addNumber("Left Encoder", m_driveTrain::getLeftEncoderValue).withPosition(1, 1);
    turnMMTab.addNumber("Right Encoder", m_driveTrain::getRightEncoderValue).withPosition(2,1);
    m_drivedurationEntry = turnMMTab.add("drive duration", 0).withPosition(6, 0).getEntry();
    m_countokEntry = turnMMTab.add("count_ok", 0).getEntry();
    SmartDashboard.putNumber("Target Angle", arclengthDegrees);
    SmartDashboard.putNumber("Speed", this.speed);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arclengthDegrees = m_arclengthEntry.getDouble(0);
    //see 2020 or 2019 code for explanation on these calculations
    arclengthTicks = (int) (arclengthDegrees * 1074.67 * 0.2291); 
    // ^^^ arc length in ticks = degrees to turn * ticks per 1 inch * degrees per 1 inch
    m_arclengthticksEntry.forceSetDouble(arclengthTicks);
    m_kF = m_kFEntry.getDouble(0.0129904762);
    m_turn_kP = m_turnkPEntry.getDouble(0.0);
    m_kI = m_kIEntry.getDouble(0.0);
    m_kD = m_kDEntry.getDouble(0.0);
    STABLE_ITERATIONS_BEFORE_FINISHED = (int) m_iterationEntry.getDouble(5.0);
    m_start_time = Timer.getFPGATimestamp();
    count = 0;
    m_driveTrain.reset_turn_PID_values(m_turn_kP, m_kI, m_kD);
    m_driveTrain.zeroSensors();
    m_driveTrain.motionMagicStartConfigsTurn();
    arclengthDegrees = SmartDashboard.getNumber("Arc Length in Degrees", 0);
    speed = SmartDashboard.getNumber("Speed", 0);
    currentAngle = m_driveTrain.m_gyro.getAngle();
    // arclengthDegrees = currentAngle + arclengthDegrees;
    m_driveTrain.disableMotorSafety();
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_driveTrain.feedWatchdog(); is this needed?
    if (m_driveTrain.motionMagicTurn(arclengthTicks)){
      count++;
    } else {
      count = 0;
    }
    m_countokEntry.setDouble(count);

    currentAngle = m_driveTrain.m_gyro.getAngle();
    // if (Math.abs(arclengthDegrees - currentAngle) < tolerance){
    //   m_driveTrain.teleop_drive(0, 0);
    //   isDone = true;
    // } else if (currentAngle < arclengthDegrees){
    //   m_driveTrain.teleop_drive(0, speed);
    // } else if (currentAngle > arclengthDegrees){
    //   m_driveTrain.teleop_drive(0, -speed);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_driveTrain.motionMagicEndConfigsTurn();
    double drive_duration = Timer.getFPGATimestamp() - m_start_time;
    m_drivedurationEntry.setDouble(drive_duration);
    m_driveTrain.teleop_drive(0, 0);
    m_driveTrain.motion_magic_end_config_turn();
    m_driveTrain.enableMotorSafety();
    isDone = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= STABLE_ITERATIONS_BEFORE_FINISHED;
  }
}
