// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.miscellaneous;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.music.Orchestra;
// import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
// import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
// import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.subsystems.DriveTrain;
import static frc.robot.Constants.*;

public class Play_Eye_of_the_tiger extends CommandBase {
  /** Creates a new Play_Eye_of_the_tiger. */

  Orchestra m_Orchestra;
  DriveTrain m_driveTrain;

  WPI_TalonFX m_Right_Talon_Leader;

  public Play_Eye_of_the_tiger(Orchestra orchestra) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Right_Talon_Leader = new WPI_TalonFX(RIGHT_TALON_LEADER);

    m_Orchestra = orchestra;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Orchestra.addInstrument(m_Right_Talon_Leader);
    m_Orchestra.loadMusic("Mario.chrp");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_Orchestra.isPlaying()){
      m_Orchestra.play();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
