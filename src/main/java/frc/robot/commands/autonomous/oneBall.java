// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveMM;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class oneBall extends SequentialCommandGroup {
  /** Creates a new oneBall. */
  DriveTrain m_drive_train;
  public oneBall(DriveTrain driveTrain) {
    m_drive_train = driveTrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //STARTING CONDITIONS: 
    //ball placed inside indexer, robot placed 19 inches away from hub fender
    addCommands(
      //shoot command
      new DriveMM(m_drive_train, 25)
    );
  }
}
