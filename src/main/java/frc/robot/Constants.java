// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Robot Electronics Map

<<<<<<< HEAD
    /////////////// CAN Bus IDs  ////////////////
    public final static int RIGHT_TALON_LEADER = 1;
    public final static int LEFT_TALON_LEADER = 2;
=======
    ///////////// CAN Bus IDs  ////////////////
	// public final static int INTAKE_INDEXER = ;
	// public final static int MIDDLE_INDEXER = ;
	// public final static int SHOOTER_INDEXER = ;
	// public final static int SHOOTER_FALCON =;
	public final static int INTAKE_TALON = 5;
	public final static int INTAKE_ARM_TALON = 6;
    public final static int RIGHT_TALON_LEADER = 2;
	public final static int RIGHT_TALON_FOLLOWER = 4;
    public final static int LEFT_TALON_LEADER = 1;
	public final static int LEFT_TALON_FOLLOWER = 3;
	public final static int INTAKE_INDEXER = 7;
	public final static int MID_INDEXER = 8;
	public final static int SHOOTER_INDEXER = 9;
	public final static int SHOOTER_FX = 10;
	public final static int CLIMER_SRX = 11;
>>>>>>> everything

	//////////////////// DIO /////////////////////
	public final static int INDEXER_INTAKE_BEAM = 2;
	public final static int INDEXER_INTAKE_MID = 1;
	public final static int INDEXER_INTAKE_SHOOTER = 0;

<<<<<<< HEAD
    ///////////// Drive Train Values ////////////
    public final static double kControllerDeadband = .05;
    public final static int DRIVER_RIGHT_AXIS = 2;
    public final static int DRIVER_LEFT_AXIS = 1;

	/////////////// Vision Values ///////////////
	public final static double CAMERA_HEIGHT_METERS = 0.9144;
	public final static double CAMERA_PITCH_RADIANS = 0.785398;
	public final static double TARGET_HEIGHT_METERS = 2.64;

	////////// Talon Specific Values ////////////
=======
    ///////////// Drive Train Values /////////////
    public final static double kControllerDeadband = .1;
    public final static int DRIVER_RIGHT_AXIS = 4;
    public final static int DRIVER_LEFT_AXIS = 1;

	/////////// Indexer Specific Values //////////
	public final static double INDEXER_SPEED = 0.5;

	/////////// Vision Specific Values ///////////
	public final static double CAMERA_HEIGHT_METERS = 0;
	public final static double CAMERA_PITCH_RADIANS = 0;
	public final static double TARGET_HEIGHT_METERS = 0;

    //////////// Talon Specific Values ///////////
>>>>>>> everything
    /**
	 * Using the configSelectedFeedbackCoefficient() function, scale units to 3600 per rotation.
	 * This is nice as it keeps 0.1 degrees of resolution, and is fairly intuitive.
	 */
    public final static double kTurnTravelUnitsPerRotation = 3600;
    
    /**
	 * Empirically measure what the difference between encoders per 360'
	 * Drive the robot in clockwise rotations and measure the units per rotation.
	 * Drive the robot in counter clockwise rotations and measure the units per rotation.
	 * Take the average of the two.
	 */
	public final static int kEncoderUnitsPerRotation = 88554;
	public final static int kEncoderTicksPerInch = 1074;
    /**
	 * Set to zero to skip waiting for confirmation.
	 * Set to nonzero to wait and report to DS if action fails.
	 */
    public final static int kTimeoutMs = 0;

    /**
	 * Motor neutral dead-band, set to the minimum 0.1%.
	 */
    public final static double kNeutralDeadband = 0.005;

	// Open Loop Ramp-up times
	public final static double kOpenLoopRamp = 0.2;
    
    /**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
     * Not all set of Gains are used in this project and may be removed as desired.
     * 	
	 * 	                                    			  		 kP      kI   kD   kF        Iz    PeakOut */
	public final static Gains kGains_Turning =   	  new Gains(0.1,   0.0,  0.0, 0.003699, 200,  1.0 );
	public final static Gains kGains_Driving =   	  new Gains(0.1,   0.0,  0.0, 0.003699, 100,  1.0 );
	public final static Gains kGains_IntakeDown =	  new Gains(2.69,  0.0,  0.0, 0.341,   100,  1.0 );
	public final static Gains kGains_IntakeUp =  	  new Gains(0.299, 0.0,  0.0, 1.023,	200,  1.0 );
	public final static Gains kGains_IntakeHold= 	  new Gains(8.000, 0.0,  40,  0.0,		300,  1.0 );
	public final static Gains kGains_CloseShot =      new Gains(4.800, 0.0,  0.0, 0.341,    100,  1.0);
	public final static Gains kGains_MidTarmac =      new Gains(4.800, 0.0,  0.0, 0.341,    200,  1.0);
	public final static Gains kGains_TarmacLine =     new Gains(4.800, 0.0,  0.0, 0.341,    300,  1.0);
	public final static Gains kGains_ShooterGains =   new Gains(4.800, 0.0,  0.0, 0.341,    400,  1.0);

	/** ---- Flat constants, you should not need to change these ---- */
	/* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
	public final static int REMOTE_0 = 0;
	public final static int REMOTE_1 = 1;
	/* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;
	/* Firmware currently supports slots [0, 3] and can be used for either PID Set */
	public final static int SLOT_0 = 0;
	public final static int SLOT_1 = 1;
	public final static int SLOT_2 = 2;
	public final static int SLOT_3 = 3;
	/* ---- Named slots, used to clarify code ---- */
	public final static int kSlot_Distanc = SLOT_0;
	public final static int kSlot_Turning = SLOT_1;
	public final static int kSlot_Velocit = SLOT_2;
	public final static int kSlot_MotProf = SLOT_3;

	public final static int kSlot_AllShots   = SLOT_0;
	public final static int kSlot_CloseShot  = SLOT_1;
	public final static int kSlot_MidTarmac  = SLOT_2;
	public final static int kSlot_TarmacLine = SLOT_3;

	public static final int kCountsPerRev = 2048;    // Encoder counts per revolution of the motor shaft.
	public static final double kSensorGearRatio = 10.71; // Gear ratio is the ratio between the *encoder* and the wheels. On the AndyMark
														// drivetrain, encoders mount 1:1 with the gearbox shaft.
	public static final double kGearRatio = 10.71;   // Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead
														// of on the gearbox.
	public static final double kWheelRadiusInches = 3.25;
	public static final int k100msPerSecond = 10;

	public static boolean kGyroReversed = true;

	/* ---- Characterization Calculations ---- */
	public static final double ksVolts = 0.65154;
	public static final double kvVoltSecondsPerMeter = .000012616;
	public static final double kaVoltSecondsSquaredPerMeter = 0.0000003799;
	public static final double kPDriveVel = 8.5;
	public static final double kTrackwidthMeters = 0.6731;
	public static final DifferentialDriveKinematics kDriveKinematics = 
			new DifferentialDriveKinematics(kTrackwidthMeters);
	public static final double kMaxSpeedMetersPerSecond = 3;
	public static final double kMaxAccelerationMetersPerSecondSquared = 3;
	// Reasonable baseline values for a RAMSETE follower in units of meters and seconds
	public static final double kRamseteB = 2;
	public static final double kRamseteZeta = 0.7;
	
}