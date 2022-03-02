// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.Intake.ArmDriveTest;
import frc.robot.commands.Indexer.TestIntakeIndexerAndShooter;
import frc.robot.commands.Intake.ArmMM;
import frc.robot.commands.Intake.ResetIntakeArmEncoder;
import frc.robot.commands.Intake.SetArm;
import frc.robot.commands.Intake.SetForwardLimit;
import frc.robot.commands.default_commands.DriveTrainDefaultCommand;
import frc.robot.commands.default_commands.IndexerDefaultCommand;
import frc.robot.commands.drivetrain.DriveForSecondsFromShuffleboard;
import frc.robot.commands.drivetrain.DriveMM;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.drivetrain.TurnToAnglePIDCommand;
import frc.robot.commands.drivetrain.TurnToAngleProfiledPIDCommand;
import frc.robot.commands.shooter.ShooterPIDTuning;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DriveTrain m_driveTrain;
  Indexer m_indexer;
  Intake m_intake;
  Shooter m_shooter;

  // OI
  XboxController m_driver_controller;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driver_controller = new XboxController(0);
    m_driveTrain = new DriveTrain();
    m_indexer = new Indexer();
    m_intake = new Intake();
    m_shooter = new Shooter();

    m_driveTrain.setDefaultCommand(new DriveTrainDefaultCommand(m_driveTrain, m_driver_controller));
    m_indexer.setDefaultCommand(new IndexerDefaultCommand(m_indexer));

    Shuffleboard.getTab("Default Drive Tab").add("DriveForSeconds", new DriveForSecondsFromShuffleboard(m_driveTrain))
                                            .withPosition(4, 1)
                                            .withSize(2, 1);         
    Shuffleboard.getTab("Drive MM Testing").add(new DriveMM(m_driveTrain, 0));
    Shuffleboard.getTab("Combined Test").add(new TestIntakeIndexerAndShooter(m_indexer, m_intake, m_shooter)).withPosition(0, 1).withSize(2, 1);
    Shuffleboard.getTab("Combined Test").add(new SetForwardLimit(m_intake)).withPosition(0, 3).withSize(2, 1);
    Shuffleboard.getTab("Arm Drive Testing").add("Test Arm Drive", new ArmDriveTest(m_intake)).withPosition(0, 1);
    Shuffleboard.getTab("Arm MM Testing").add("ReSet Intake Arm", new SetArm(m_intake)).withPosition(0, 3).withSize(2, 1);
    Shuffleboard.getTab("Arm MM Testing").add("Extend Intake", new ArmMM(m_intake, Intake.INTAKE_ARM_EXTEND)).withPosition(0, 0).withSize(2,1);
    Shuffleboard.getTab("Arm MM Testing").add("Retract Intake", new ArmMM(m_intake, Intake.INTAKE_ARM_RETRACT)).withPosition(2,0).withSize(2,1);
    Shuffleboard.getTab("Arm MM Testing").add(new ResetIntakeArmEncoder(m_intake)).withPosition(0, 2).withSize(2, 1);
    Shuffleboard.getTab("ShooterPID").add("Shoot" , new ShooterPIDTuning(m_shooter, m_indexer)).withPosition(0, 3);
    Shuffleboard.getTab("Turn MM Testing").add("Turn MM", new TurnToAngle(m_driveTrain, 0)).withPosition(0, 3).withSize(2, 1);
    Shuffleboard.getTab("Turn MM Testing").add("Turn90DegreesGyroPID", new TurnToAnglePIDCommand(90, m_driveTrain)).withPosition(2, 3).withSize(2, 1);
    Shuffleboard.getTab("Turn MM Testing").add("Turn90DegreesGyroProfiledPID ", new TurnToAngleProfiledPIDCommand(90, m_driveTrain)).withPosition(4,3).withSize(2,1);
    // SmartDashboard.getNumber("Target Angle", 0);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PrintCommand("Auto");
  }
  
  
}