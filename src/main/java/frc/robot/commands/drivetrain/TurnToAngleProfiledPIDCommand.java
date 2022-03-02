// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngleProfiledPIDCommand extends ProfiledPIDCommand {
  /** Creates a new TurnToAngleProfiledPIDCommand. */
  public TurnToAngleProfiledPIDCommand(double targetAngle, DriveTrain drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            NetworkTableInstance.getDefault().getTable("Shuffleboard").getEntry("Turn MM Testing/kP").getDouble(0),
            NetworkTableInstance.getDefault().getTable("Shuffleboard").getEntry("Turn MM Testing/kI").getDouble(0),
            NetworkTableInstance.getDefault().getTable("Shuffleboard").getEntry("Turn MM Testing/kD").getDouble(0),
            // The motion profile constraints
            new TrapezoidProfile.Constraints(5, 10)),
        // This should return the measurement
        drivetrain::getHeading,
        // This should return the goal (can also be a constant)
        // () -> new TrapezoidProfile.State(),
        targetAngle,
        // This uses the output
        (output, setpoint) -> drivetrain.teleop_drive(0, output),
        drivetrain
    );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController().setTolerance(100, 300);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
