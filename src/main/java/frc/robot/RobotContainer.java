// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.TARGET_DISTANCE_CLOSE;
import static frc.robot.Constants.TARGET_DISTANCE_FAR;

import java.util.Map;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Indexer.CancelIndexer;
import frc.robot.commands.Indexer.EjectBalls;
import frc.robot.commands.Indexer.ShootBallsTilEmptyOrThreeSeconds;
import frc.robot.commands.Intake.ArmMM;
import frc.robot.commands.Intake.DropIntakeAndCollectBalls;
import frc.robot.commands.Intake.ResetArmLimitAndEncoder;
import frc.robot.commands.Intake.ResetIntakeArmEncoder;
import frc.robot.commands.Intake.SetExtendLimit;
import frc.robot.commands.Intake.TurnOffIntakeArm;
import frc.robot.commands.autonomous.oneBall;
import frc.robot.commands.autonomous.twoBallLeft;
import frc.robot.commands.autonomous.twoBallRight;
import frc.robot.commands.climber.CancelClimber;
import frc.robot.commands.climber.HighBarClimb;
import frc.robot.commands.climber.HighBarRaise;
import frc.robot.commands.climber.LowBarClimb;
import frc.robot.commands.climber.LowBarRaise;
import frc.robot.commands.climber.LowerClimber;
import frc.robot.commands.climber.TestClimberDown;
import frc.robot.commands.climber.TestClimberUp;
import frc.robot.commands.climber.DriveClimbertoReverseHardLimit;
import frc.robot.commands.combined.StopShooterAndIndexerMotors;
import frc.robot.commands.default_commands.DriveTrainDefaultCommand;
import frc.robot.commands.default_commands.IndexerDefaultCommand;
import frc.robot.commands.drivetrain.DriveMM;
import frc.robot.commands.drivetrain.DriveMMTest;
import frc.robot.commands.drivetrain.TurnToAngle;
import frc.robot.commands.drivetrain.TurnToAngleTest;
import frc.robot.commands.drivetrain.UpdateDriveLimiters;
import frc.robot.commands.shooter.CancelShooter;
import frc.robot.commands.shooter.SetShooterDistance;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.Shooting_Sequence;
import frc.robot.commands.shooter.StartShooterWheel;
import frc.robot.commands.vision.AimAtCargo;
import frc.robot.commands.vision.DriveWithVision;
import frc.robot.commands.vision.LEDoff;
import frc.robot.commands.vision.LEDon;
import frc.robot.commands.vision.configureVisionCargoPID;
import frc.robot.commands.vision.configureVisionDrivePID;
import frc.robot.commands.vision.configureVisionTurnPID;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDsubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.ShotDistance;
import frc.robot.subsystems.Vision;
import frc.robot.utils.GamepadAxisButton;

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
  Climber m_climber;

  // OI
  XboxController m_driver_controller, m_partner_controller;
  JoystickButton m_d_a, m_d_b, m_d_rb, m_d_lb, m_d_rs, m_d_ls, m_p_a, m_p_b, m_p_rb, m_d_sel, m_d_strt, m_p_x, m_d_y, m_p_y,
                 m_p_lb, m_p_rs, m_p_ls, m_p_start, m_p_sel;
  //auto chooser
  private SendableChooser<Command> m_auto_chooser;
  GamepadAxisButton m_d_rt, m_d_lt;
  POVButton m_d_up, m_d_right, m_d_down, m_d_left;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // All the subsystems here
    m_driver_controller = new XboxController(0);
    m_partner_controller = new XboxController(1);
    m_driveTrain = new DriveTrain();
    m_indexer = new Indexer();
    m_intake = new Intake();
    m_shooter = new Shooter();
    m_ledsubsystem = new LEDsubsystem();
    m_vision = new Vision();
    m_climber = new Climber();

    // Joystick Buttons
    m_d_a = new JoystickButton(m_driver_controller, XboxController.Button.kA.value);
    m_d_b = new JoystickButton(m_driver_controller, XboxController.Button.kB.value);
    m_d_y = new JoystickButton(m_driver_controller, XboxController.Button.kY.value);
    m_d_rb = new JoystickButton(m_driver_controller, XboxController.Button.kRightBumper.value);
    m_d_lb = new JoystickButton(m_driver_controller, XboxController.Button.kLeftBumper.value);
    m_d_ls = new JoystickButton(m_driver_controller, XboxController.Button.kLeftStick.value);
    m_d_rs = new JoystickButton(m_driver_controller, XboxController.Button.kRightStick.value);
    m_d_rt = new GamepadAxisButton(m_driver_controller, 3);
    m_d_lt = new GamepadAxisButton(m_driver_controller, 2);
    m_d_sel = new JoystickButton(m_driver_controller, XboxController.Button.kBack.value);
    m_d_strt = new JoystickButton(m_driver_controller, XboxController.Button.kStart.value);

    m_p_a = new JoystickButton(m_partner_controller, XboxController.Button.kA.value);
    m_p_b = new JoystickButton(m_partner_controller, XboxController.Button.kB.value);
    m_p_y = new JoystickButton(m_partner_controller, XboxController.Button.kY.value);
    m_p_rb = new JoystickButton(m_partner_controller, XboxController.Button.kRightBumper.value);
    m_p_start = new JoystickButton(m_partner_controller, XboxController.Button.kStart.value);
    m_p_sel = new JoystickButton(m_partner_controller, XboxController.Button.kBack.value);
    m_p_rs = new JoystickButton(m_partner_controller, XboxController.Button.kRightStick.value);

    // POV Buttons
    m_d_up = new POVButton(m_driver_controller, 0);
    m_d_right = new POVButton(m_driver_controller, 90);
    m_d_down = new POVButton(m_driver_controller, 180);
    m_d_left = new POVButton(m_driver_controller, 270);

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
    m_d_a.whenPressed(new LEDon(m_vision));
    m_d_b.whenPressed(new LEDon(m_vision));
    m_d_a.whileHeld(new DriveWithVision(m_driveTrain, m_vision, TARGET_DISTANCE_CLOSE));
    m_d_b.whileHeld(new DriveWithVision(m_driveTrain, m_vision, TARGET_DISTANCE_FAR));
    m_d_a.whenReleased(new LEDoff(m_vision));
    m_d_b.whenReleased(new LEDoff(m_vision));
    m_d_rt.whileHeld(new AimAtCargo(m_vision, m_driveTrain, m_driver_controller));
    m_d_lt.whenPressed(new CancelIndexer(m_indexer));
    m_d_rb.whenHeld(new PrintCommand("Driving Inverted"));
    m_d_lb.whenPressed(new DropIntakeAndCollectBalls(m_intake, m_indexer));
    m_d_lb.whenReleased(new ArmMM(m_intake, Intake.INTAKE_ARM_RETRACT));
    m_d_rs.whenPressed(new HighBarClimb(m_climber, m_ledsubsystem, m_driver_controller));
    m_d_ls.whenPressed(new LowBarClimb(m_climber, m_ledsubsystem, m_driver_controller));
    m_d_sel.whenPressed(new CancelClimber(m_climber, m_ledsubsystem));

    // Driver POV Bindings
    m_d_up.whenPressed(new CancelClimber(m_climber, m_ledsubsystem));
    m_d_right.whenPressed(new HighBarRaise(m_climber, m_ledsubsystem, m_driver_controller));
    m_d_left.whenPressed(new LowBarRaise(m_climber, m_ledsubsystem, m_driver_controller));
    m_d_down.whileHeld(new LowerClimber(m_climber, m_ledsubsystem));

    // Partner Bindings
    m_p_rb.whileHeld(new EjectBalls(m_indexer, m_shooter));
    m_p_start.whenPressed(new TurnOffIntakeArm(m_intake));
    m_p_sel.whenPressed(new ResetArmLimitAndEncoder(m_intake));
    m_p_a.whenPressed(new Shoot(m_shooter, m_indexer, ShotDistance.ClosestShot));
    m_p_b.whenPressed(new Shooting_Sequence(m_shooter, m_intake, m_indexer, ShotDistance.MidTarmac));
    m_p_y.whenPressed(new Shoot(m_shooter, m_indexer, ShotDistance.TarmacLine));
    m_p_sel.whenPressed(new CancelShooter(m_shooter));
    m_p_rs.whenPressed(new TestClimberDown(m_climber));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_auto_chooser.getSelected();
  }
  
  private void buildShuffleboard(){
    buildDriverTab();
    buildDriverTestTab();
    buildShooterTab();
    buildIntakeTestTab();
    buildClimberTestTab();
    // buildVisionTab();

    // Shuffleboard.getTab("Combined Test").add(new TestIntakeIndexerAndShooter(m_indexer, m_intake, m_shooter)).withPosition(0, 1).withSize(2, 1);
    // Shuffleboard.getTab("Combined Test").add(new SetForwardLimit(m_intake)).withPosition(0, 3).withSize(2, 1);
    // Shuffleboard.getTab("Arm MM Testing").add("ReSet Intake Arm", new SetArm(m_intake)).withPosition(0, 3).withSize(2, 1);
    // Shuffleboard.getTab("Intake").add("Extend Intake", new ArmMM(m_intake, Intake.INTAKE_ARM_EXTEND)).withPosition(0, 2).withSize(2,1);
    // Shuffleboard.getTab("Intake").add("Retract Intake", new ArmMM(m_intake, Intake.INTAKE_ARM_RETRACT)).withPosition(2,2).withSize(2,1);
    // Shuffleboard.getTab("Arm MM Testing").add(new ResetIntakeArmEncoder(m_intake)).withPosition(0, 2).withSize(2, 1);
    // Shuffleboard.getTab("ShooterPID").add("Shoot" , new ShooterPIDTuning(m_shooter, m_indexer)).withPosition(0, 3);
    // Shuffleboard.getTab("Turn MM Testing").add("Turn MM", new TurnToAngle(m_driveTrain, 0)).withPosition(0, 3).withSize(2, 1);
    // Shuffleboard.getTab("Intake").add(new CollectBalls(m_intake, m_indexer)).withPosition(0, 1).withSize(2, 1);
    // Shuffleboard.getTab("Intake").add(new DropIntakeAndCollectBalls(m_intake, m_indexer)).withPosition(2, 1).withSize(2, 1);
    // Shuffleboard.getTab("Intake").add(new EjectBalls(m_indexer)).withPosition(0, 3).withSize(2, 1);
    
  }

  private void buildDriverTab(){
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

    // The drive tab is roughly 9 x 5 (columns x rows)
    // Camera can be 4 x 4, gyro 
    // driveTab.add("Cargo Cam", new HttpCamera("Cargo Photon", "http://10.50.24.11:5800"))
    //                                         .withWidget(BuiltInWidgets.kCameraStream).withPosition(0, 0)
    //                                         .withSize(4, 4);
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
    driveTab.add("Ball Count Test", 0).withSize(1, 1).withPosition(7, 0);  
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
    // driveTab.add("Field", m_driveTrain.getField()).withPosition(6, 3).withSize(4, 2).withWidget(BuiltInWidgets.kField);

    // //auto chooser
    m_auto_chooser = new SendableChooser<Command>();
    m_auto_chooser.addOption("1 ball", new oneBall(m_driveTrain, m_shooter, m_intake, m_indexer));
    m_auto_chooser.addOption("Left Tarmac, 2 ball", new twoBallLeft(m_driveTrain, m_shooter, m_intake, m_indexer));
    m_auto_chooser.addOption("Right Tarmac, 2 ball", new twoBallRight(m_driveTrain, m_shooter, m_intake, m_indexer));
    //m_auto_chooser.addOption("Bottom Left Tarmac, 4 ball", new fourBall(m_driveTrain, m_shooter, m_intake, m_indexer));
    driveTab.add("Autonomous Chooser", m_auto_chooser).withWidget(BuiltInWidgets.kComboBoxChooser).withPosition(4, 4).withSize(2, 1);
  }

  public void buildDriverTestTab(){
    ShuffleboardTab driveMMTab = Shuffleboard.getTab("Drive Testing");
    // Configuration Values on row 1
    driveMMTab.add("kF", 0.1 )              .withPosition(0, 0).getEntry();
    driveMMTab.add("kP", 0.3 )              .withPosition(1, 0).getEntry();
    driveMMTab.add("kI", 0 )                .withPosition(2, 0).getEntry();
    driveMMTab.add("kD", 0 )                .withPosition(3, 0).getEntry();
    driveMMTab.add("Tgt. Inches", 0)        .withPosition(4, 0).getEntry();
    driveMMTab.add("Tgt. Degrees", 0)       .withPosition(5, 0).getEntry();
    driveMMTab.add("Finish Iterations", 5 ) .withPosition(6, 0).getEntry();

    // Result Values on row 2
    driveMMTab.add("Tgt. Ticks", 0)                                          .withPosition(0, 1);
    driveMMTab.addNumber("Left Encoder", m_driveTrain::getLeftEncoderValue)  .withPosition(1, 1);
    driveMMTab.addNumber("Right Encoder", m_driveTrain::getRightEncoderValue).withPosition(2, 1);
    driveMMTab.addNumber("Gyro Read", m_driveTrain::getRawAngle)             .withPosition(3, 1);
    driveMMTab.add("Run Time", 0)                                            .withPosition(4, 1);
    driveMMTab.addNumber("Left SP", m_driveTrain::getLeftSetPoint).withPosition(5, 1).withSize(1, 1);
    driveMMTab.addNumber("Right SP", m_driveTrain::getRightSetPoint).withPosition(6, 1).withSize(1, 1);
   
    // Drive limiters on row 3
    driveMMTab.add("Forward Limiter", 2.5).withPosition(0, 2);
    driveMMTab.add("Rotation Limiter", 2.5).withPosition(1, 2);
    driveMMTab.add("Drive Max", .7).withPosition(2, 2);
    driveMMTab.add("Update Limits", new UpdateDriveLimiters(m_driveTrain)).withPosition(3, 2).withSize(2, 1);

    // Drive commands on row 4
    driveMMTab.add("Drive MM 100", new DriveMM(m_driveTrain, 100))        .withPosition(0, 3).withSize(2, 1);
    driveMMTab.add("Drive MM -100", new DriveMM(m_driveTrain, -100))      .withPosition(2, 3).withSize(2, 1);
    driveMMTab.add("Drive MM Test", new DriveMMTest(m_driveTrain, 0))     .withPosition(4, 3).withSize(2, 1);

    // Turn commands on row 5
    driveMMTab.add("Turn MM 90", new TurnToAngle(m_driveTrain, 90))          .withPosition(0, 4).withSize(2, 1);
    driveMMTab.add("Turn MM -90", new TurnToAngle(m_driveTrain, -90))        .withPosition(2, 4).withSize(2, 1);
    driveMMTab.add("Turn MM Test", new TurnToAngleTest(m_driveTrain, 0))     .withPosition(4, 4).withSize(2, 1);
  }

  public void buildShooterTab(){
    ShuffleboardTab driveTab = Shuffleboard.getTab("Shooter");
    driveTab.add("SetShotDistanceCloseShot", new SetShooterDistance(m_shooter, ShotDistance.ClosestShot)).withPosition(0, 0).withSize(2, 1);
    driveTab.add("StarShooterWheel", new StartShooterWheel(m_shooter)).withPosition(2, 0).withSize(2, 1);
    driveTab.add("WaitUntilCommand", new WaitUntilCommand(m_shooter::isUpToSpeed)).withPosition(4, 0).withSize(2, 1);
    driveTab.add("ShootBallsUntilEmpty", new ShootBallsTilEmptyOrThreeSeconds(m_indexer, m_shooter)).withPosition(6, 0).withSize(2, 1);
    driveTab.add("StopShooter", new StopShooterAndIndexerMotors(m_shooter, m_indexer)).withPosition(8, 0).withSize(2, 1);

    driveTab.addBoolean("Is Up To Speed", m_shooter::isUpToSpeed).withPosition(0, 2).withSize(1, 1);
    driveTab.addNumber("Closed Loop Error", m_shooter::getClosedLoopError).withPosition(1, 2).withSize(1, 1);
  }

  private void buildIntakeTestTab(){
    ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
    intakeTab.add("ResetDriveSpeed", -.5)                  .withPosition(0, 0).withSize(1, 1);
    intakeTab.add("Extend Limit", Intake.INTAKE_ARM_EXTEND).withPosition(3, 0).withSize(1, 1);

    intakeTab.add("Intake Fwd Limit", 1)                            .withPosition(1, 1).withSize(1, 1).withWidget(BuiltInWidgets.kBooleanBox);
    intakeTab.add("Intake Rev Limit", 0)                            .withPosition(2, 1).withSize(1, 1).withWidget(BuiltInWidgets.kBooleanBox);
    intakeTab.addNumber("Arm Encoder", m_intake::getRelativeEncoder).withPosition(3, 1).withSize(1, 1);
    
    intakeTab.add("RetractIntakeArm", new ArmMM(m_intake, Intake.INTAKE_ARM_RETRACT)).withPosition(0, 2).withSize(2, 1);
    intakeTab.add("ExtendIntakeArm", new ArmMM(m_intake, Intake.INTAKE_ARM_EXTEND))  .withPosition(2, 2).withSize(2, 1);
    intakeTab.add("Reset Extend Limit", new SetExtendLimit(m_intake))                .withPosition(4, 2).withSize(2, 1);

    intakeTab.add("ResetArmLimitAndEncoder", new ResetArmLimitAndEncoder(m_intake)).withPosition(0, 3).withSize(2, 1);
    intakeTab.add("TurnOffIntakeArm", new TurnOffIntakeArm(m_intake))              .withPosition(2, 3).withSize(2, 1);

  }

  private void buildClimberTestTab(){
    ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
    // Testing Information
    climberTab.add("ClimberDownSpeed", -.7).withPosition(0, 0).withSize(1, 1);
    climberTab.add("ClimberUpSpeed", 1)    .withPosition(1, 0).withSize(1, 1);

    climberTab.addNumber("Encoder", m_climber::getRelativeEncoder)                    .withPosition(1, 1);
    climberTab.addBoolean("Forward Limit", m_climber::forwardLimitSwitchTriggered)    .withPosition(2, 1);
    climberTab.addBoolean("Reverse Limit", m_climber::reverseLimitSwitchTriggered)    .withPosition(3, 1);
    climberTab.addBoolean("Soft Forward Limit", m_climber::getClimberSoftForwardLimit).withPosition(4, 1);
    climberTab.addBoolean("Soft Reverse Limit", m_climber::getClimberSoftReverseLimit).withPosition(5, 1);
    
    // High bar raise
    climberTab.add("Raise to MidRung", new HighBarRaise(m_climber, m_ledsubsystem, m_driver_controller)).withPosition(0, 2).withSize(2, 1);
    climberTab.add("Raise to LowBar", new LowBarRaise(m_climber, m_ledsubsystem, m_driver_controller))  .withPosition(2, 2).withSize(2, 1);
    climberTab.add("Cancel Climber", new CancelClimber(m_climber, m_ledsubsystem))                      .withPosition(4, 2).withSize(2, 1);
    // climberTab.add("Lower Climber", new LowerToLimitOrTime(m_climber))                  .withPosition(6, 2).withSize(2, 1);
    
    climberTab.add("TestClimbDown", new TestClimberDown(m_climber))                     .withPosition(0, 3).withSize(2, 1);
    climberTab.add("TestClimbUp", new TestClimberUp(m_climber))                         .withPosition(2, 3).withSize(2, 1);
    climberTab.add("Reset to LowerLimit", new DriveClimbertoReverseHardLimit(m_climber)).withPosition(4, 3).withSize(2, 1);
  }

  private void buildVisionTab() {
    ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");

    visionTab.addNumber("Distance to Target", m_vision::getHubTargetRangeIndex0).withPosition(1, 0);
    visionTab.addNumber("Target Yaw",         m_vision::getHubTargetRangeIndex1).withPosition(1, 1);

    visionTab.add("LED on", new LEDon(m_vision))  .withPosition(0, 0);
    visionTab.add("LED off", new LEDoff(m_vision)).withPosition(0, 1);
    visionTab.add("Condigure Vision Drive", new configureVisionDrivePID(m_driveTrain)).withPosition(0, 2);
    visionTab.add("Configure Turn Turn", new configureVisionTurnPID(m_driveTrain))    .withPosition(0, 3);
    visionTab.add("Configure Cargo controllre", new configureVisionCargoPID(m_driveTrain)).withPosition(0, 4);

    visionTab.add("forward drive speed", 0);
    visionTab.add("Turn speed", 0);
    visionTab.add("Cargo Yaw", 0);

    visionTab.add("Drive kP", 0.4);
    visionTab.add("Drive kD", 0);
    visionTab.add("Drive kI", 0);
    visionTab.add("Drive kF", 0);
    visionTab.add("Turn kP", 0.02);
    visionTab.add("Turn kD", 0);
    visionTab.add("Turn kI", 0);
    visionTab.add("Turn kF", 0);
    visionTab.add("Cargo kP,", 0.011);
  }
}