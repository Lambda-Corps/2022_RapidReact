// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {

    private final TalonFX m_shooter;
    private double k_MaxShooterOutput = 1;

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
        m_shooter = new TalonFX(SHOOTER_FX);
        m_shooter.configFactoryDefault();

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.peakOutputReverse = 0; // Don't drive backward.
        config.nominalOutputReverse = 0;
        config.nominalOutputForward = 0;
        config.peakOutputForward = k_MaxShooterOutput;
        config.slot0.kF = k_kF;
        config.slot0.kP = k_kP;
        config.slot0.kI = k_kI;
        config.slot0.kD = k_kD;
        config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 35, 40, 1.0);

        
        m_shooter.configAllSettings(config);

        // Invert the motor so forward shoots
        m_shooter.setInverted(true);
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

    public void update_shooter_pid_config(double kf, double kp, double ki, double kd){
        return;
    }
    
    public void update_shooter_max_output(double max){
        k_MaxShooterOutput = max;
    }

    public void test_shooter_percent(double speed){
        m_shooter.set(ControlMode.PercentOutput, speed);
    }

    public void velocityPID(double m_setpoint, double m_tolerance) {
        m_shooter.set(ControlMode.Velocity, m_setpoint);

    }

    public void configureVelocityPID(double kp, double ki, double kd, double kf) {
        m_shooter.config_kI(0, kp);
        m_shooter.config_kI(0, ki);
        m_shooter.config_kD(0, kd);
        m_shooter.config_kF(0, kf);
    }

    public void stopmotor() {
        m_shooter.set(ControlMode.PercentOutput, 0);
    }
}
