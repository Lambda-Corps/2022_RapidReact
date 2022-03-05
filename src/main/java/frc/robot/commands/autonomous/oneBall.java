// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveMM;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.Auto_Shooting_Sequence;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotDistance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class oneBall extends SequentialCommandGroup {
  /** Creates a new oneBall. */
  DriveTrain m_drive_train;
  Shooter m_shooter;
  Intake m_intake;
  Indexer m_indexer;
  public oneBall(DriveTrain driveTrain, Shooter shooter, Intake intake, Indexer indexer) {
    m_drive_train = driveTrain;
    m_shooter = shooter;
    m_intake = intake;
    m_indexer = indexer;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //STARTING CONDITIONS: 
    //ball placed inside indexer, robot placed 19 inches away from hub fender
    addCommands(
      new Shoot(m_shooter, m_indexer, ShotDistance.ClosestShot),
      new TurnToAngle(m_drive_train, 350),
      new DriveMM(m_drive_train, -50)//DriveMM(m_drive_train, 50)
    );
  }
}
