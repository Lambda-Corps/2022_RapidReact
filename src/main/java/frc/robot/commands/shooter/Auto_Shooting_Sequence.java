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
import frc.robot.subsystems.LEDsubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Shooting_Sequence extends SequentialCommandGroup {
  /** Creates a new Shooting_Sequence. */

  LEDsubsystem m_LEDsubsystem;

  public Auto_Shooting_Sequence(Shooter shooter, Intake intake, Indexer indexer, LEDsubsystem ledsubsystem, ShotDistance distance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    m_LEDsubsystem = ledsubsystem;

    addCommands(
      new SetShooterDistance(shooter, distance),
      new StartShooterWheel(shooter),
      new WaitUntilCommand(shooter::isUpToSpeed),
      new ShootBallsTilEmptyOrThreeSeconds(indexer, shooter, m_LEDsubsystem),
      new StopShooterAndIndexerMotors(shooter, indexer)  
    );
  }
}
