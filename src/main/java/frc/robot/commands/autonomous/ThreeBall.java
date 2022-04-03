// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.CollectBalls;
import frc.robot.commands.Intake.ExtendIntakeBangBang;
import frc.robot.commands.Intake.ResetArmLimitAndEncoder;
import frc.robot.commands.drivetrain.DriveMM;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.shooter.SetShooterDistance;
import frc.robot.commands.shooter.Shooting_Sequence;
import frc.robot.commands.shooter.StartShooterWheel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotDistance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBall extends SequentialCommandGroup {
  /** Creates a new ThreeBall. */
  public ThreeBall(DriveTrain m_driveTrain, Shooter m_shooter, Intake m_intake, Indexer m_indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(new ExtendIntakeBangBang(m_intake, Intake.INTAKE_ARM_EXTEND).withTimeout(2).andThen(new CollectBalls(m_intake, m_indexer).withTimeout(3)), new WaitCommand(.75).andThen(new DriveMM(m_driveTrain, 45.44))),
      new ParallelCommandGroup(new ResetArmLimitAndEncoder(m_intake), new WaitCommand(.6).andThen(new SetShooterDistance(m_shooter, ShotDistance.TarmacLine)).andThen( new StartShooterWheel(m_shooter))),
      new TurnToAngle(m_driveTrain, 201).withTimeout(4), //turn around
      //new TurnToAngle(m_drive_train, 30), //angle to be perpendicular to the hub fender
      new Shooting_Sequence(m_shooter, m_intake, m_indexer, ShotDistance.TarmacLine), //Shoot grabbed ball and preload ball
      new ParallelCommandGroup(new TurnToAngle(m_driveTrain, -85).andThen(new DriveMM(m_driveTrain, 90)), new ExtendIntakeBangBang(m_intake, 1700).andThen(new CollectBalls(m_intake, m_indexer).withTimeout(3.5))),
      new ParallelCommandGroup(new ResetArmLimitAndEncoder(m_intake), new TurnToAngle(m_driveTrain, 118), new StartShooterWheel(m_shooter)),
      new Shooting_Sequence(m_shooter, m_intake, m_indexer, ShotDistance.TarmacLine)
    );
  }
}
