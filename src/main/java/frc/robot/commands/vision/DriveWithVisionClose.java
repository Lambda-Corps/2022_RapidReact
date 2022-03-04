package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import static frc.robot.Constants.*;

public class DriveWithVisionClose extends CommandBase {
  /** Creates a new VisionAlign. */
  DriveTrain m_driveTrain;
  Vision m_vision;

  public DriveWithVisionClose(DriveTrain driveTrain, Vision vision) {
    m_vision = new Vision();
    m_driveTrain = new DriveTrain();

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
    double[] range = m_vision.getTargetRange();
    m_driveTrain.visionDrive(range[0], range[1], TAREGET_DISTANCE_CLOSE);
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
