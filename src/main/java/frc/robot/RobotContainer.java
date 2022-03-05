// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Indexer.EjectBalls;
import frc.robot.commands.Indexer.TestIntakeIndexerAndShooter;
import frc.robot.commands.Intake.ArmMM;
import frc.robot.commands.Intake.CollectBalls;
import frc.robot.commands.Intake.DropIntakeAndCollectBalls;
import frc.robot.commands.Intake.ResetIntakeArmEncoder;
import frc.robot.commands.Intake.SetArm;
import frc.robot.commands.Intake.SetForwardLimit;
import frc.robot.commands.default_commands.DriveTrainDefaultCommand;
import frc.robot.commands.default_commands.IndexerDefaultCommand;
import frc.robot.commands.drivetrain.DriveForSecondsFromShuffleboard;
import frc.robot.commands.drivetrain.DriveMM;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShooterPIDTuning;
import frc.robot.commands.vision.AimAtCargo;
import frc.robot.commands.vision.DriveWithVision;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDsubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.utils.GamepadAxisButton;

import static frc.robot.Constants.*;

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
  LEDsubsystem m_ledsubsystem;
  Vision m_vision;

  // OI
  XboxController m_driver_controller, m_partner_controller;
  JoystickButton m_d_a, m_d_b, m_d_rb, m_d_lb, m_d_rs, m_d_ls, m_p_a, m_p_b, m_p_rb, m_d_sel, m_p_start, m_d_y;
  GamepadAxisButton m_d_rt;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driver_controller = new XboxController(0);
    m_partner_controller = new XboxController(1);
    m_driveTrain = new DriveTrain();
    m_indexer = new Indexer();
    m_intake = new Intake();
    m_shooter = new Shooter();
    m_ledsubsystem = new LEDsubsystem();
    m_vision = new Vision();

    // Joystick Buttons
    m_d_a = new JoystickButton(m_driver_controller, XboxController.Button.kA.value);
    m_d_b = new JoystickButton(m_driver_controller, XboxController.Button.kB.value);
    m_d_y = new JoystickButton(m_driver_controller, XboxController.Button.kY.value);
    m_d_rb = new JoystickButton(m_driver_controller, XboxController.Button.kRightBumper.value);
    m_d_lb = new JoystickButton(m_driver_controller, XboxController.Button.kLeftBumper.value);
    m_d_ls = new JoystickButton(m_driver_controller, XboxController.Button.kLeftStick.value);
    m_d_rs = new JoystickButton(m_driver_controller, XboxController.Button.kRightStick.value);
    m_d_rt = new GamepadAxisButton(m_driver_controller, 3);
    m_d_sel = new JoystickButton(m_driver_controller, XboxController.Button.kBack.value);
    m_p_a = new JoystickButton(m_partner_controller, XboxController.Button.kA.value);
    m_p_b = new JoystickButton(m_partner_controller, XboxController.Button.kB.value);
    m_p_rb = new JoystickButton(m_partner_controller, XboxController.Button.kRightBumper.value);
    m_p_start = new JoystickButton(m_partner_controller, XboxController.Button.kStart.value);

    m_driveTrain.setDefaultCommand(new DriveTrainDefaultCommand(m_driveTrain, m_driver_controller));
    m_indexer.setDefaultCommand(new IndexerDefaultCommand(m_indexer));
    
    // Build up the driver's heads up display
    buildShuffleboard();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver Bindings
    m_d_a.whileHeld(new DriveWithVision(m_driveTrain, m_vision, TARGET_DISTANCE_CLOSE));
    m_d_b.whileHeld(new DriveWithVision(m_driveTrain, m_vision, TARGET_DISTANCE_FAR));
    m_d_rt.whileHeld(new AimAtCargo(m_vision, m_driveTrain, m_driver_controller));
    m_d_rb.whenHeld(new PrintCommand("Driving Inverted"));
    m_d_lb.whenPressed(new DropIntakeAndCollectBalls(m_intake, m_indexer));
    m_d_lb.whenReleased(new ArmMM(m_intake, Intake.INTAKE_ARM_RETRACT));
    m_d_rs.whenPressed(new PrintCommand("Climber Up"));
    m_d_ls.whenPressed(new PrintCommand("Climber Down"));
    m_d_y.whenPressed(new PrintCommand("Climber Cancelled"));

    // Partner Bindings
    m_p_a.whenPressed(new Shoot/*Long Shot*/()); 
    m_p_b.whenPressed(new Shoot/*Short Shot*/());
    m_p_rb.whileHeld(new EjectBalls(m_indexer));
    m_p_start.whenPressed(new ResetIntakeArmEncoder(m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new PrintCommand("Auto");
  }
  
  private void buildShuffleboard(){
    buildDriverTab();

    Shuffleboard.getTab("Drive MM Testing").add(new DriveMM(m_driveTrain, 0));
    Shuffleboard.getTab("Combined Test").add(new TestIntakeIndexerAndShooter(m_indexer, m_intake, m_shooter)).withPosition(0, 1).withSize(2, 1);
    Shuffleboard.getTab("Combined Test").add(new SetForwardLimit(m_intake)).withPosition(0, 3).withSize(2, 1);
    Shuffleboard.getTab("Arm MM Testing").add("ReSet Intake Arm", new SetArm(m_intake)).withPosition(0, 3).withSize(2, 1);
    Shuffleboard.getTab("Intake").add("Extend Intake", new ArmMM(m_intake, Intake.INTAKE_ARM_EXTEND)).withPosition(0, 2).withSize(2,1);
    Shuffleboard.getTab("Intake").add("Retract Intake", new ArmMM(m_intake, Intake.INTAKE_ARM_RETRACT)).withPosition(2,2).withSize(2,1);
    Shuffleboard.getTab("Arm MM Testing").add(new ResetIntakeArmEncoder(m_intake)).withPosition(0, 2).withSize(2, 1);
    Shuffleboard.getTab("ShooterPID").add("Shoot" , new ShooterPIDTuning(m_shooter, m_indexer)).withPosition(0, 3);
    Shuffleboard.getTab("Turn MM Testing").add("Turn MM", new TurnToAngle(m_driveTrain, 0)).withPosition(0, 3).withSize(2, 1);
    Shuffleboard.getTab("Intake").add(new CollectBalls(m_intake, m_indexer)).withPosition(0, 1).withSize(2, 1);
    Shuffleboard.getTab("Intake").add(new DropIntakeAndCollectBalls(m_intake, m_indexer)).withPosition(2, 1).withSize(2, 1);
    Shuffleboard.getTab("Intake").add(new EjectBalls(m_indexer)).withPosition(0, 3).withSize(2, 1);
  }

  private void buildDriverTab(){
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

    // The drive tab is roughly 9 x 5 (columns x rows)
    // Camera can be 4 x 4, gyro 
    driveTab.add("Cargo Cam", new HttpCamera("Cargo Photon", "http://10.50.24.11:5800"))
                                            .withWidget(BuiltInWidgets.kCameraStream).withPosition(0, 0)
                                            .withSize(4, 4);
    // Add heading and outputs to the driver views
    driveTab.add("Gyro", m_driveTrain.getGyro()).withPosition(4, 0).withWidget(BuiltInWidgets.kGyro);
    driveTab.add("Left Output", 0).withSize(1, 1).withPosition(4, 2).withWidget(BuiltInWidgets.kDial)
                                  .withProperties(Map.of("Min", -1, "Max", 1));
    driveTab.add("Right Output", 0).withSize(1, 1).withPosition(5, 2).withWidget(BuiltInWidgets.kDial)
                                  .withProperties(Map.of("Min", -1, "Max", 1));

    // Add vision cues below the camera stream block
    driveTab.add("HighTarget", false).withSize(1, 1).withPosition(0, 4).withWidget(BuiltInWidgets.kBooleanBox);
    driveTab.add("BallTarget", false).withSize(1, 1).withPosition(1, 4).withWidget(BuiltInWidgets.kBooleanBox);
    driveTab.add("Pipeline",0).withSize(1, 1).withPosition(2, 4).withWidget(BuiltInWidgets.kDial)
                              .withProperties(Map.of("Min", 0, "Max", 2));
    driveTab.add("Distance", 0).withSize(1, 1).withPosition(3, 4);

    // Add Intake Sensors and Ball Count
    driveTab.add("Ball Count",0).withSize(1, 1).withPosition(6, 0).withWidget(BuiltInWidgets.kDial)
                              .withProperties(Map.of("Min", 0, "Max", 2));
    driveTab.add("ShootBreak", false).withSize(1, 1).withPosition(7, 0).withWidget(BuiltInWidgets.kBooleanBox);
    driveTab.add("MidBreak", false).withSize(1, 1).withPosition(8, 0).withWidget(BuiltInWidgets.kBooleanBox);
    driveTab.add("IntakeBreak", false).withSize(1, 1).withPosition(9, 0).withWidget(BuiltInWidgets.kBooleanBox);
    // Add Intake Limits
    driveTab.add("Int. Fwd Hard", false).withSize(1, 1).withPosition(6, 1).withWidget(BuiltInWidgets.kBooleanBox);
    driveTab.add("Int. Fwd Soft", false).withSize(1, 1).withPosition(7, 1).withWidget(BuiltInWidgets.kBooleanBox);
    driveTab.add("Int. Rev Hard", false).withSize(1, 1).withPosition(8, 1).withWidget(BuiltInWidgets.kBooleanBox);
    driveTab.add("Int. Rev Soft", false).withSize(1, 1).withPosition(9, 1).withWidget(BuiltInWidgets.kBooleanBox);
    // // Climber Limits
    driveTab.add("Clm. Fwd Hard", false).withSize(1, 1).withPosition(6, 2).withWidget(BuiltInWidgets.kBooleanBox);
    driveTab.add("Clm. Fwd Soft", false).withSize(1, 1).withPosition(7, 2).withWidget(BuiltInWidgets.kBooleanBox);
    driveTab.add("Clm. Rev Hard", false).withSize(1, 1).withPosition(8, 2).withWidget(BuiltInWidgets.kBooleanBox);
    driveTab.add("Clm. Rev Soft", false).withSize(1, 1).withPosition(9, 2).withWidget(BuiltInWidgets.kBooleanBox);

    // Field
    driveTab.add("Field", m_driveTrain.getField()).withPosition(6, 3).withSize(4, 2).withWidget(BuiltInWidgets.kField);



  }

  public void buildDriverTestTab(){
    
  }
}