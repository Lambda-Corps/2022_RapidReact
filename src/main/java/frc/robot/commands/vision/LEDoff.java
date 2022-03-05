// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LEDoff extends InstantCommand {
  Vision m_vision;

  public LEDoff(Vision vision) {
    m_vision = vision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
  }

  @Override
  public void initialize() {
    m_vision.setLEDoff();
  }
}