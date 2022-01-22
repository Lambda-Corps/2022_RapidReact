// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.default_commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.*;

public class DriveTrainDefaultCommand extends CommandBase {
  // Reference to the constructed drive train from RobotContainer to be 
  // used to drive our robot
  private final DriveTrain m_driveTrain;
  private final XboxController m_driverController;
  /**
   * Creates a new DefaultDriveTrainCommand.
   */
  public DriveTrainDefaultCommand(DriveTrain driveTrain, XboxController driverController) {
    m_driveTrain = driveTrain;
    m_driverController = driverController;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Axises are inverted, negate them so positive is forward
    double right = m_driverController.getRawAxis(DRIVER_RIGHT_AXIS); // Right X
    double left  = -m_driverController.getRawAxis(DRIVER_LEFT_AXIS); // Left Y

    m_driveTrain.teleop_drive(left, right);
    // m_driveTrain.curvature_drive_imp(0.5, 0.5, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}