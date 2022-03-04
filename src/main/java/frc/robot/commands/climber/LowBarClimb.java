// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.climber.LowBarRaise;
import frc.robot.commands.climber.LowBarLower;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LowBarClimb extends SequentialCommandGroup {
  /** Creates a new LowBarClimb. */

  Climber m_climber;

  public LowBarClimb(Climber climber, XboxController driverController) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_climber = climber;

    addCommands(
      new LowBarRaise(climber, driverController),
      new WaitCommand(2),
      new LowBarLower(climber)
    );
  }
}
