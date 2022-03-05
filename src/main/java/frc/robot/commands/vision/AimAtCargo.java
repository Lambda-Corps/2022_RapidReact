// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import static frc.robot.Constants.*;

public class AimAtCargo extends CommandBase {
  /** Creates a new AimAtCargo. */
  Vision m_vision;
  DriveTrain m_driveTrain;
  XboxController m_drive_remote;

  public AimAtCargo(Vision vision, DriveTrain driveTrain, XboxController driverRemote) {
    m_driveTrain = driveTrain;
    m_vision = vision;
    m_drive_remote = driverRemote;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision, driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetVisionPidController();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = m_drive_remote.getRawAxis(DRIVER_RIGHT_AXIS);
    m_driveTrain.cargoAim(m_vision.getCargoTargetYaw(), forward);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
