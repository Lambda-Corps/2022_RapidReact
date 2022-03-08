// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import static frc.robot.Constants.kEncoderTicksPerInch;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveMM extends CommandBase {
  /** Creates a new DriveMM. */
  DriveTrain m_driveTrain;
  double m_targetPosition;
  int count;
  //the number of times motion magic is on target before the command finishes
  int STABLE_ITERATIONS_BEFORE_FINISHED = 5;
  public DriveMM(DriveTrain driveTrain, double targetInches) {
    m_driveTrain = driveTrain;
    // Distance in inches to ticks conversion is:
    /*
		   wheelRotations = positionMeters/(2 * Math.PI * kWheelRadiusInches);
	   	 motorRotations = wheelRotations * kSensorGearRatio
		   sensorCounts =   motorRotations * kCountsPerRev
     */
    m_targetPosition = targetInches * kEncoderTicksPerInch;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
    m_driveTrain.motion_magic_start_config_drive(m_targetPosition >= 0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_driveTrain.motionMagicDrive(m_targetPosition)){
      count++;
    } else {
      count = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.teleop_drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= STABLE_ITERATIONS_BEFORE_FINISHED;
  }
}
