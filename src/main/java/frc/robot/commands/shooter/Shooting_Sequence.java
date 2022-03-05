// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.Indexer.ShootBallsTilEmptyOrThreeSeconds;
import frc.robot.commands.combined.StopShooterAndIndexerMotors;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotDistance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shooting_Sequence extends SequentialCommandGroup {
  /** Creates a new Shooting_Sequence. */
  Shooter m_shooter;
  Intake m_intake;
  Indexer m_indexer;
  ShotDistance m_distance;
  public Shooting_Sequence(Shooter shooter, Intake intake, Indexer indexer, ShotDistance distance) {
    m_shooter = shooter;
    m_intake = intake;
    m_indexer = indexer;
    m_distance = distance;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetShooterDistance(m_shooter, distance),
      new StartShooterWheel(m_shooter),
      new WaitUntilCommand(m_shooter::isUpToSpeed),
      new ShootBallsTilEmptyOrThreeSeconds(m_indexer),
      new StopShooterAndIndexerMotors(m_shooter, m_indexer)
    );
  }
}
