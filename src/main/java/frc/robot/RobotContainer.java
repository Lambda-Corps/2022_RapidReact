// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.default_commands.DriveTrainDefaultCommand;
import frc.robot.commands.DriveForSecondsFromShuffleboard;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DriveTrain m_driveTrain;

  // OI
  XboxController m_driver_controller;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driver_controller = new XboxController(0);
    m_driveTrain = new DriveTrain();

    m_driveTrain.setDefaultCommand(new DriveTrainDefaultCommand(m_driveTrain, m_driver_controller));
    Shuffleboard.getTab("Default Drive Tab").add("DriveForSeconds", new DriveForSecondsFromShuffleboard(m_driveTrain))
                                            .withPosition(4, 1)
                                            .withSize(2, 1);
                                            
    // SmartDashboard.putData("Turn To Angle", new TurnToAngle(m_driveTrain, 0.2, 90));
    Shuffleboard.getTab("Turn MM Testing").add(new TurnToAngle(m_driveTrain, 0.2, 90));
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
