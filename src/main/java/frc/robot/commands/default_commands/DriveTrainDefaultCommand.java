// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.default_commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

import static frc.robot.Constants.*;

public class DriveTrainDefaultCommand extends CommandBase {
    // Reference to the constructed drive train from RobotContainer to be 
    // used to drive our robot
    private final DriveTrain m_driveTrain;
    private final XboxController m_driverController;
    public final ShuffleboardTab m_driveTab;
    private NetworkTableEntry m_leftVal, m_rightVal;
    /**
     * Creates a new DefaultDriveTrainCommand.
     */
    public DriveTrainDefaultCommand(DriveTrain driveTrain, XboxController driverController) {
        m_driveTrain = driveTrain;
        m_driverController = driverController;

        m_driveTab = Shuffleboard.getTab("Default Drive Tab");
        m_leftVal = m_driveTab.add("Left Drive Value", 0).getEntry();
        m_rightVal = m_driveTab.add("Right Drive Value", 0).getEntry();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
        public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double turn, forward;
        if(Robot.isSimulation()){
            turn = m_driverController.getRawAxis(0); // Right X
            forward  = -m_driverController.getRawAxis(DRIVER_LEFT_AXIS); // Left Y
        }
        else {
            // Axises are inverted, negate them so positive is forward
            turn = m_driverController.getRawAxis(DRIVER_RIGHT_AXIS); // Right X
            forward  = -m_driverController.getRawAxis(DRIVER_LEFT_AXIS); // Left Y
        }
        

        m_leftVal.forceSetDouble(forward);
        m_rightVal.forceSetDouble(turn);

        m_driveTrain.teleop_drive(forward, turn);
        SmartDashboard.putNumber("Forward", forward);
        SmartDashboard.putNumber("Turn", turn);
        // m_driveTrain.curvature_drive_imp(0.5, 0.5, false);
    }

    // Called once the command ends or is interrupted.
    @Override
        public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}