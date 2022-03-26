package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

/**
 * The command will waiting for the shooter to come up to speed,
 *  and will attempt to unstick any balls that prevent this from happening
 */
public class WaitForShooterToBeUpToSpeed extends CommandBase {
  Shooter m_shooter;

  //when true, the command will be attempting to unstick the ball
  //when false the command will be attempting to bring the shooter up to speed
  private boolean isBallInUnstickState = false;

  // number of iterations the motors have been unsticking the ball
  private int unstickBallTimeCount = 0;

  //the number of iterations to wait till finishing reversing the motors, 30 ms pass between every iterations
  private static final int MAX_TIME_COUNT_UNSTICKING_BALL = 5;

  public WaitForShooterToBeUpToSpeed(Shooter shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void execute() {
        if (isBallInUnstickState) {
            if (unstickBallTimeCount > MAX_TIME_COUNT_UNSTICKING_BALL) {
                isBallInUnstickState = false;
                m_shooter.startVelocityPID();
            } else {
                unstickBallTimeCount++;
            }
        } else {
            if (m_shooter.isBallStuck()) {
                isBallInUnstickState = true;
                m_shooter.unstickBalls();
            } else {
                unstickBallTimeCount = 0;
            }
        }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shooter.isUpToSpeed();
  }
}
