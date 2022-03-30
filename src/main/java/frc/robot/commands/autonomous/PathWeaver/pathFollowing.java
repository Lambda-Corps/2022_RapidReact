// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.PathWeaver;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class pathFollowing extends SequentialCommandGroup {
  /** Creates a new pathFollowing. */
  public pathFollowing(DriveTrain m_driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                ksVolts,
                kvVoltSecondsPerMeter,
                kaVoltSecondsSquaredPerMeter),
            kDriveKinematics,
            10);


    // Create config for trajectory

    TrajectoryConfig config =
        new TrajectoryConfig(
            kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    Trajectory trajectory = new Trajectory();
    //Trajectory trajectory2 = new Trajectory();
    //String path = "pathplanner/generatedJSON/testpath.json";
    // String path = "paths/11First.wpilib.json";
    // //String path2 = "paths/11Second.wpilib.json";
    // try{
    //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
    //   trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    //   //Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(path2);
    //   //trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
    // }
    // // A default trajectory to follow.  All units in meters.
    // catch(IOException ex){
      //DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
      trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 3, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            //List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            List.of(new Translation2d(1,3), new Translation2d(2,3)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 3, new Rotation2d(0)),
            // Pass config
            config);
    //}

    addCommands(
      new RamseteCommand(
      trajectory,
      m_driveTrain::getPose,
      new RamseteController(kRamseteB, kRamseteZeta),
      new SimpleMotorFeedforward(
          ksVolts,
          kvVoltSecondsPerMeter,
          kaVoltSecondsSquaredPerMeter),
          kDriveKinematics,
          m_driveTrain::getWheelSpeeds,
          new PIDController(kPDriveVel, 0, 0),
          new PIDController(kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          m_driveTrain::tankDriveVolts,
          m_driveTrain)
          
      /* new RamseteCommand(
      trajectory2,
      m_driveTrain::getPose,
      new RamseteController(kRamseteB, kRamseteZeta),
      new SimpleMotorFeedforward(
          ksVolts,
          kvVoltSecondsPerMeter,
          kaVoltSecondsSquaredPerMeter),
          kDriveKinematics,
          m_driveTrain::getWheelSpeeds,
          new PIDController(kPDriveVel, 0, 0),
          new PIDController(kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          m_driveTrain::tankDriveVolts,
          m_driveTrain) */
          
          

          );
          
    m_driveTrain.resetOdometry(trajectory.getInitialPose());
    // m_driveTrain.resetOdometry(trajectory2.getInitialPose());
  }
}
