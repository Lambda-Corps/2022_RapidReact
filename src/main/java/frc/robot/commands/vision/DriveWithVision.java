package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

public class DriveWithVision extends CommandBase {
  /** Creates a new VisionAlign. */
  DriveTrain m_driveTrain;
  Vision m_vision;
  double m_targetDistance;

  public DriveWithVision(DriveTrain driveTrain, Vision vision, double targetDistance) {
    m_vision = vision;
    m_driveTrain = driveTrain;

    m_targetDistance = targetDistance;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetVisionPidController();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] range = m_vision.getHubTargetRange();
    m_driveTrain.visionDrive(range[0], range[1], m_targetDistance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.teleop_drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
