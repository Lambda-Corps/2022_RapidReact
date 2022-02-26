// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {

    private final TalonSRX m_shooter;
    private double k_MaxShooterOutput = .6;

    // Velocity PID calculations //
    /*
    * Shooting at ~ 40% is a good trajectory in simple testing
    * 40% = 7670 ticks per 100 ms <-- measured in phoenix tuner
    * kF = (.40 * 1023) / 7670 = .0534
    * 
    * We want KP to respond with no more than 20% of an increase to recover
    * if we're more than one motor rotation behind
    * kP = (.20 * 1023) / (ticks per rotation * gear ratio)
    * kP = (.20 * 1023) / (2048 * 1:1) = .0999
    */
    private double k_kF = .0534;
    private double k_kP = .0999;
    private double k_kD = 0;
    private double k_kI = 0;

    /** Creates a new Shooter. */
    public Shooter() {
        m_shooter = new TalonSRX(SHOOTER_TALON);
        m_shooter.configFactoryDefault();

        TalonSRXConfiguration config = new TalonSRXConfiguration();

        config.peakOutputReverse = 0; // Don't drive backward.
        config.nominalOutputReverse = 0;
        config.nominalOutputForward = 0;
        config.peakOutputForward = k_MaxShooterOutput;
        config.slot0.kF = k_kF;
        config.slot0.kP = k_kP;
        config.slot0.kI = k_kI;
        config.slot0.kD = k_kD;
        config.continuousCurrentLimit = 25;
        
        m_shooter.configAllSettings(config);

        // Invert the motor so forward shoots
        m_shooter.setInverted(true);
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }
}
