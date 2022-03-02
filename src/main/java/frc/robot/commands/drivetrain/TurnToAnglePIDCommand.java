// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAnglePIDCommand extends PIDCommand {
  double m_calculated_out;
  double m_target_angle;
  DriveTrain m_DriveTrain;
  NetworkTableEntry m_entry_target_degrees;
  /** Creates a new TurnToAnglePIDCommand. */
  public TurnToAnglePIDCommand(double targetAngle, DriveTrain drivetrain) {
    super(
        // The controller that the command will use
        new PIDController(.005,0,0),
        // This should return the measurement
        drivetrain::getHeading,
        // This should return the setpoint (can also be a constant)
        targetAngle,
        // This uses the output
        output ->  drivetrain.teleop_drive(0, output),
        drivetrain
    );
    m_target_angle = targetAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(5, 10);
    getController().enableContinuousInput(-180, 180);

    Shuffleboard.getTab("Turn MM Testing").addNumber("PIDOutput", this::getOutput).withPosition(5, 1);
    Shuffleboard.getTab("Turn MM Testing").addNumber("SetPoint", this::getSetPoint).withPosition(6, 1);
    m_entry_target_degrees = Shuffleboard.getTab("Turn MM Testing").add("Target Degrees", targetAngle).withPosition(3,1).withSize(1, 1).getEntry();
    m_DriveTrain = drivetrain;
  }

  @Override
  public void initialize(){
    double targetSetPoint = m_entry_target_degrees.getDouble(m_target_angle);
    m_controller.reset();
    m_controller.setSetpoint(m_DriveTrain.getHeading() + targetSetPoint);
  }
  @Override
  public void execute() {
    m_useOutput.accept(
        m_controller.calculate(m_measurement.getAsDouble(), m_setpoint.getAsDouble()));
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  public double getOutput(){
    return m_controller.calculate( m_measurement.getAsDouble(), m_setpoint.getAsDouble() );
  }
  public double getSetPoint(){
    return m_controller.getSetpoint();
  }
}
