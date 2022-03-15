// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetExtendLimit extends InstantCommand {
  Intake m_intake;
  NetworkTableEntry m_extend_entry;
  public SetExtendLimit(Intake intake) {
    m_intake = intake;
    m_extend_entry = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Intake").getEntry("Extend Limit");
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double limit = m_extend_entry.getDouble(0);

    Intake.INTAKE_ARM_EXTEND = (int) limit;
  }
}
