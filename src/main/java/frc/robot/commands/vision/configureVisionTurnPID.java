// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class configureVisionTurnPID extends InstantCommand {
  DriveTrain dT;

  public configureVisionTurnPID(DriveTrain driveTrain) {
    dT = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Vision");
    dT.configureVisionTurnController(table.getEntry("Turn kP").getDouble(0), 
                                      table.getEntry("Turn kI").getDouble(0), 
                                      table.getEntry("Turn kD").getDouble(0));
  }
}
