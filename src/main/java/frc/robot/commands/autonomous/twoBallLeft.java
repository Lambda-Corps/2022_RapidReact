// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intake.ArmMM;
import frc.robot.commands.Intake.CollectBalls;
import frc.robot.commands.Intake.ResetArmLimitAndEncoder;
import frc.robot.commands.drivetrain.DriveMM;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.Shooting_Sequence;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotDistance;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class twoBallLeft extends SequentialCommandGroup {
  /** Creates a new twoBallRight. */
  DriveTrain m_drive_train;
  Shooter m_shooter;
  Intake m_intake;
  Indexer m_indexer;
  public twoBallLeft(DriveTrain driveTrain, Shooter shooter, Intake intake, Indexer indexer) {
    m_drive_train = driveTrain;
    m_shooter = shooter;
    m_intake = intake;
    m_indexer = indexer;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //STARTING CONDITIONS: 1 ball in robot, front of the robot at edge of tarmac, directly in front of the ball
    //                       on the left side of the tarmac
    addCommands(
      new ArmMM(m_intake, Intake.INTAKE_ARM_EXTEND),
      new ParallelCommandGroup(new CollectBalls(m_intake, m_indexer).withTimeout(5), new WaitCommand(.75).andThen(new DriveMM(m_drive_train, 45.44))),
      new ResetArmLimitAndEncoder(m_intake),
      new TurnToAngle(m_drive_train, 192), //turn around
      new DriveMM(m_drive_train, 60), //drive up to fender, may need lowered a little
      //new TurnToAngle(m_drive_train, 30), //angle to be perpendicular to the hub fender
      new Shooting_Sequence(m_shooter, m_intake, m_indexer, ShotDistance.ClosestShot)
    );
  }
}
