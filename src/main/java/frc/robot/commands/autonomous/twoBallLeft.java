// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveMM;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class twoBallLeft extends SequentialCommandGroup {
  /** Creates a new twoBallRight. */
  DriveTrain m_drive_train;
  public twoBallLeft(DriveTrain driveTrain) {
    m_drive_train = driveTrain;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //STARTING CONDITIONS: 1 ball in robot, front of the robot at edge of tarmac, directly in front of the ball
    //                       on the left side of the tarmac
    addCommands(
      new DriveMM(m_drive_train, 40.44), //drive to the ball
      new TurnToAngle(m_drive_train, 180), //turn around
      new DriveMM(m_drive_train, 40.44),
      new DriveMM(m_drive_train, 75), //drive up to fender, may need lowered a little
      new TurnToAngle(m_drive_train, -30) //angle to be perpendicular to the hub fender
      //shoot command
    );
  }
}
