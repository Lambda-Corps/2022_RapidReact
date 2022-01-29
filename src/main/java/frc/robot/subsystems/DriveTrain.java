// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TurnToAngle;
import edu.wpi.first.math.MathUtil;

import static frc.robot.Constants.*;

public class DriveTrain extends SubsystemBase {
  // TalonFX's for the drivetrain
  // Right side is inverted here to drive forward
  WPI_TalonFX m_left_leader, m_right_leader;

  // Variables to hold the invert types for the talons
  TalonFXInvertType m_left_invert, m_right_invert;

  // Gyro 
  public AHRS m_gyro;

  // Safety Drive to have motor watchdog turn robot off if we lose comms
  DifferentialDrive m_safety_drive;

  // Curvature Drive member variables
  private double m_quickStopThreshold = .2;
  private double m_quickStopAlpha = .1;
  private double m_quickStopAccumulator;
  private double m_deadband = .1;

  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    m_gyro = new AHRS(SPI.Port.kMXP);
    m_left_leader = new WPI_TalonFX(LEFT_TALON_LEADER);
    m_right_leader = new WPI_TalonFX(RIGHT_TALON_LEADER);

    m_safety_drive = new DifferentialDrive(m_left_leader, m_right_leader);

    /** Invert Directions for Left and Right */
    m_left_invert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
    m_right_invert = TalonFXInvertType.Clockwise; //Same as invert = "true"
    
    /** Config Objects for motor controllers */
    TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration _rightConfig = new TalonFXConfiguration();

    /* Set Neutral Mode */
		m_left_leader.setNeutralMode(NeutralMode.Brake);
		m_right_leader.setNeutralMode(NeutralMode.Brake);

		/* Configure output */
		m_left_leader.setInverted(m_left_invert);
		m_right_leader.setInverted(m_right_invert);
  
    /* Configure the left Talon's selected sensor as integrated sensor */
		/* 
		 * Currently, in order to use a product-specific FeedbackDevice in configAll objects,
		 * you have to call toFeedbackType. This is a workaround until a product-specific
		 * FeedbackDevice is implemented for configSensorTerm
		 */
		_leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Feedback Source

		/* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
		_rightConfig.remoteFilter1.remoteSensorDeviceID = m_left_leader.getDeviceID(); //Device ID of Remote Source
		_rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type
		
		/* Setup difference signal to be used for turn when performing Drive Straight with encoders */
		setRobotTurnConfigs(m_right_invert, _rightConfig);

		/* Config the neutral deadband. */
		_leftConfig.neutralDeadband = kNeutralDeadband;
		_rightConfig.neutralDeadband = kNeutralDeadband;

		/* max out the peak output (for all modes).  However you can
		 * limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_leftConfig.peakOutputForward = +1.0;
		_leftConfig.peakOutputReverse = -1.0;
		_rightConfig.peakOutputForward = +1.0;
		_rightConfig.peakOutputReverse = -1.0;

		/* FPID Gains for turn servo */
		/* FPID for Distance */
		_rightConfig.slot1.kF = kGains_Turning.kF;
		_rightConfig.slot1.kP = kGains_Turning.kP;
		_rightConfig.slot1.kI = kGains_Turning.kI;
		_rightConfig.slot1.kD = kGains_Turning.kD;
		_rightConfig.slot1.integralZone = kGains_Turning.kIzone;
		_rightConfig.slot1.closedLoopPeakOutput = kGains_Turning.kPeakOutput;
			
		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		_rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
    _rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;
    
    /* APPLY the config settings */
		m_left_leader.configAllSettings(_leftConfig);
		m_right_leader.configAllSettings(_rightConfig);
		
		/* Set status frame periods */
		m_right_leader.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, kTimeoutMs);
		m_right_leader.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, kTimeoutMs);
		m_left_leader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, kTimeoutMs);		//Used remotely by right Talon, speed up

		zeroSensors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
	SmartDashboard.putNumber("Current Angle", TurnToAngle.currentAngle);
  }

  /* Zero all sensors used */
	public void zeroSensors() {
		m_left_leader.getSensorCollection().setIntegratedSensorPosition(0.0, kTimeoutMs);
		m_right_leader.getSensorCollection().setIntegratedSensorPosition(0.0, kTimeoutMs);
	}
	
	/** Deadband 5 percent, used on the gamepad */
	private double deadband(double value) {
		/* Upper deadband */
		if (value >= kControllerDeadband) {
      return value;
    }
		
		/* Lower deadband */
		if (value <= -kControllerDeadband) {
      return value;
    }

		/* Outside deadband */
		return 0;
  }
  
  /** Make sure the input to the set command is 1.0 >= x >= -1.0 **/
	private double clamp_drive(double value) {
		/* Upper deadband */
		if (value >= 1.0) {
      return 1.0;
    }

		/* Lower deadband */
		if (value <= -1.0) {
      return -1.0;
    }

		/* Outside deadband */
		return value;
  }

  public void teleop_drive(double forward, double turn){
    // forward = deadband(forward);
    // turn = deadband(turn);

    // forward = clamp_drive(forward);
    // turn = clamp_drive(turn);

    // // If the yaw value is zero, we should be driving straight with encoder correction,
    // // otherwise, drive with the input values corrected for deadband
    // if( turn == 0 ){
    //   /* Determine which slot affects which PID */
    //   m_right_leader.selectProfileSlot(kSlot_Turning, PID_TURN);
    //   double _targetAngle = m_right_leader.getSelectedSensorPosition(1);
			
	// 		/* Configured for percentOutput with Auxiliary PID on Integrated Sensors' Difference */
	// 		m_right_leader.set(ControlMode.PercentOutput, forward, DemandType.AuxPID, _targetAngle);
	// 		m_left_leader.follow(m_right_leader, FollowerType.AuxOutput1);
    // }
    // else {
    //   curvature_drive_imp(forward, turn, forward == 0 ? true : false);
    // }

    // m_safety_drive.feed();

	m_safety_drive.curvatureDrive(forward, turn, true);

  }

  private void curvature_drive_imp(double xSpeed, double zRotation, boolean isQuickTurn) {
    double angularPower;
    boolean overPower;

    if (isQuickTurn) {
      if (Math.abs(xSpeed) < m_quickStopThreshold) {
        m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
            + m_quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
      }
      overPower = true;
      angularPower = zRotation;
    } else {
      overPower = false;
      angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

      if (m_quickStopAccumulator > 1) {
        m_quickStopAccumulator -= 1;
      } else if (m_quickStopAccumulator < -1) {
        m_quickStopAccumulator += 1;
      } else {
        m_quickStopAccumulator = 0.0;
      }
    }

    double leftMotorOutput = xSpeed + angularPower;
    double rightMotorOutput = xSpeed - angularPower;

    // If rotation is overpowered, reduce both outputs to within acceptable range
    if (overPower) {
      if (leftMotorOutput > 1.0) {
        rightMotorOutput -= leftMotorOutput - 1.0;
        leftMotorOutput = 1.0;
      } else if (rightMotorOutput > 1.0) {
        leftMotorOutput -= rightMotorOutput - 1.0;
        rightMotorOutput = 1.0;
      } else if (leftMotorOutput < -1.0) {
        rightMotorOutput -= leftMotorOutput + 1.0;
        leftMotorOutput = -1.0;
      } else if (rightMotorOutput < -1.0) {
        leftMotorOutput -= rightMotorOutput + 1.0;
        rightMotorOutput = -1.0;
      }
    }

    // Normalize the wheel speeds
    double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
    if (maxMagnitude > 1.0) {
      leftMotorOutput /= maxMagnitude;
      rightMotorOutput /= maxMagnitude;
    }

    m_left_leader.set(ControlMode.PercentOutput, leftMotorOutput);
    m_right_leader.set(ControlMode.PercentOutput, rightMotorOutput);
  }
  	public boolean motionMagicTurn(int arcTicks){
		  double tolerance = 10; //TODO determine if this works or if we need it higher
		  m_left_leader.set(ControlMode.MotionMagic, arcTicks);
		  m_right_leader.set(ControlMode.MotionMagic, -arcTicks);
		  int currentLeftPos = (int) Math.abs(m_left_leader.getSelectedSensorPosition());
		  int currentRightPos = (int) Math.abs(m_right_leader.getSelectedSensorPosition());
		  int targetTicks = Math.abs(arcTicks);
		return (targetTicks - currentLeftPos) < tolerance && (targetTicks - currentRightPos) < tolerance;
	  }
	public void motionMagicStartConfigsTurn(){
		m_left_leader.selectProfileSlot(kSlot_Turning, PID_PRIMARY);
		m_right_leader.selectProfileSlot(kSlot_Turning, PID_PRIMARY);
		m_left_leader.configMotionCruiseVelocity(3000, kTimeoutMs);
		m_left_leader.configMotionAcceleration(3000, kTimeoutMs);
		m_right_leader.configMotionCruiseVelocity(3000, kTimeoutMs);
		m_right_leader.configMotionAcceleration(3000, kTimeoutMs);
	}
	public void disableMotorSafety(){
		m_safety_drive.setSafetyEnabled(false);
	  }
	
	public void enableMotorSafety(){
		m_safety_drive.setSafetyEnabled(true);
	  }
	public void feedWatchdog(){
		m_safety_drive.feed();
	  }
	  public void motion_magic_end_config_turn(){
		m_left_leader.configMotionCruiseVelocity(2100, kTimeoutMs);
		m_left_leader.configMotionAcceleration(500, kTimeoutMs);
		m_right_leader.configMotionCruiseVelocity(2100, kTimeoutMs);
		m_right_leader.configMotionAcceleration(500, kTimeoutMs);
	  }
	public double getLeftEncoderValue(){
		return m_left_leader.getSelectedSensorPosition();
	  }
	
	public double getRightEncoderValue(){
		return m_right_leader.getSelectedSensorPosition();
	  }
	  public void reset_turn_PID_values(double kP, double kI, double kD) {
		m_left_leader.config_kP(kSlot_Turning, kP);
		m_left_leader.config_kI(kSlot_Turning, kI);
		m_left_leader.config_kD(kSlot_Turning, kD);
		
		m_right_leader.config_kP(kSlot_Turning, kP);
		m_right_leader.config_kI(kSlot_Turning, kI);
		m_right_leader.config_kD(kSlot_Turning, kD);
	  }
  //////////////// NOTE ////////////////
  // The setRobotTurnConfigs method is taken directly from CTRE's example code,
  // and should be usable without modification.
  // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/DriveStraight_AuxIntegratedSensor

  /** 
	 * Determines if SensorSum or SensorDiff should be used 
	 * for combining left/right sensors into Robot Distance.  
	 * 
	 * Assumes Aux Position is set as Remote Sensor 0.  
	 * 
	 * configAllSettings must still be called on the master config
	 * after this function modifies the config values. 
	 * 
	 * @param masterInvertType Invert of the Master Talon
	 * @param masterConfig Configuration object to fill
	 */
	void setRobotTurnConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
		/**
		 * Determine if we need a Sum or Difference.
		 * 
		 * The auxiliary Talon FX will always be positive
		 * in the forward direction because it's a selected sensor
		 * over the CAN bus.
		 * 
		 * The master's native integrated sensor may not always be positive when forward because
		 * sensor phase is only applied to *Selected Sensors*, not native
		 * sensor sources.  And we need the native to be combined with the 
		 * aux (other side's) distance into a single robot heading.
		 */

		/* THIS FUNCTION should not need to be modified. 
		   This setup will work regardless of whether the master
		   is on the Right or Left side since it only deals with
		   heading magnitude.  */

		/* Check if we're inverted */
		if (masterInvertType == TalonFXInvertType.Clockwise){
			/* 
				If master is inverted, that means the integrated sensor
				will be negative in the forward direction.
				If master is inverted, the final sum/diff result will also be inverted.
				This is how Talon FX corrects the sensor phase when inverting 
				the motor direction.  This inversion applies to the *Selected Sensor*,
				not the native value.
				Will a sensor sum or difference give us a positive heading?
				Remember the Master is one side of your drivetrain distance and 
				Auxiliary is the other side's distance.
					Phase | Term 0   |   Term 1  | Result
				Sum:  -1 *((-)Master + (+)Aux   )| OK - magnitude will cancel each other out
				Diff: -1 *((-)Master - (+)Aux   )| NOT OK - magnitude increases with forward distance.
				Diff: -1 *((+)Aux    - (-)Master)| NOT OK - magnitude decreases with forward distance
			*/

			masterConfig.sum0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			masterConfig.sum1Term = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();   //Aux Selected Sensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1

			/*
				PID Polarity
				With the sensor phasing taken care of, we now need to determine if the PID polarity is in the correct direction
				This is important because if the PID polarity is incorrect, we will run away while trying to correct
				Will inverting the polarity give us a positive counterclockwise heading?
				If we're moving counterclockwise(+), and the master is on the right side and inverted,
				it will have a negative velocity and the auxiliary will have a negative velocity
				 heading = right + left
				 heading = (-) + (-)
				 heading = (-)
				Let's assume a setpoint of 0 heading.
				This produces a positive error, in order to cancel the error, the right master needs to
				drive backwards. This means the PID polarity needs to be inverted to handle this
				
				Conversely, if we're moving counterclwise(+), and the master is on the left side and inverted,
				it will have a positive velocity and the auxiliary will have a positive velocity.
				 heading = right + left
				 heading = (+) + (+)
				 heading = (+)
				Let's assume a setpoint of 0 heading.
				This produces a negative error, in order to cancel the error, the left master needs to
				drive forwards. This means the PID polarity needs to be inverted to handle this
			*/
			masterConfig.auxPIDPolarity = true;
		} else {
			/* Master is not inverted, both sides are positive so we can diff them. */
			masterConfig.diff0Term = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();    //Aux Selected Sensor
			masterConfig.diff1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Sum0 + Sum1
			/* With current diff terms, a counterclockwise rotation results in negative heading with a right master */
			masterConfig.auxPIDPolarity = true;
		}
		/**
		 * Heading units should be scaled to ~4000 per 360 deg, due to the following limitations...
		 * - Target param for aux PID1 is 18bits with a range of [-131072,+131072] units.
		 * - Target for aux PID1 in motion profile is 14bits with a range of [-8192,+8192] units.
		 *  ... so at 3600 units per 360', that ensures 0.1 degree precision in firmware closed-loop
		 *  and motion profile trajectory points can range +-2 rotations.
		 */
		masterConfig.auxiliaryPID.selectedFeedbackCoefficient = kTurnTravelUnitsPerRotation / kEncoderUnitsPerRotation;
	}
}
