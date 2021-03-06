// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.DropIntakeAndCollectBalls;
import frc.robot.commands.Intake.ResetArmLimitAndEncoder;
import frc.robot.commands.drivetrain.DriveMM;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.shooter.Auto_Shooting_Sequence;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotDistance;
import frc.robot.subsystems.LEDsubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class fourBall extends SequentialCommandGroup {
  /** Creates a new fourBall. */
  DriveTrain m_drive_train;
  Shooter m_shooter;
  Intake m_intake;
  Indexer m_indexer;
  LEDsubsystem m_LEDsubsystem;
  public fourBall(DriveTrain driveTrain, Shooter shooter, Intake intake, Indexer indexer, LEDsubsystem ledsubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_drive_train = driveTrain;
    m_shooter = shooter;
    m_intake = intake;
    m_indexer = indexer;
    m_LEDsubsystem = ledsubsystem;
    //STARTING CONDITIONS: 
    addCommands(
      new DriveMM(m_drive_train, 40.44), //drive to the ball
      new DropIntakeAndCollectBalls(m_intake, m_indexer),
      new ResetArmLimitAndEncoder(m_intake),
      new TurnToAngle(m_drive_train, 180), //turn around
      new DriveMM(m_drive_train, 115.44), //drive up to fender, may need lowered a little
      new TurnToAngle(m_drive_train, 30), //angle to be perpendicular to the hub fender
      new Auto_Shooting_Sequence(m_shooter, m_intake, m_indexer, m_LEDsubsystem, ShotDistance.ClosestShot),
      new TurnToAngle(m_drive_train, -230), //turn to go towards terminal need to measure this still
      new DriveMM(m_drive_train, 300),  // need to measure this still
      new ResetArmLimitAndEncoder(m_intake),
      new TurnToAngle(m_drive_train, 230),
      new DriveMM(m_drive_train, 300),
      new Auto_Shooting_Sequence(m_shooter, m_intake, m_indexer, m_LEDsubsystem, ShotDistance.ClosestShot)
    );
  }
}
