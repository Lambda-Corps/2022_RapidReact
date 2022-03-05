package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import java.net.PortUnreachableException;

public class Vision extends SubsystemBase {
  int LIMELIGHTPIPELINE = 0;
  int HD3000PIPELINE = 0;

  PhotonCamera m_HD3000 = new PhotonCamera("lifecam");
  PhotonCamera m_limelight = new PhotonCamera("eagletron");

  ShuffleboardTab m_visionTab;
  NetworkTableEntry m_tx, m_ty, m_ta, m_bestTarget, targetPitch, targetYaw, targetRange, targetCount, targetType, cargoTarget, hubTarget;
  PhotonTrackedTarget m_target;

  double pitch, yaw, area;

  private boolean m_hasTargets = false;

  public Vision() {
      m_limelight.setPipelineIndex(LIMELIGHTPIPELINE);
      m_HD3000.setPipelineIndex(HD3000PIPELINE);

      // double ballcamDiagFOV = 75.0; // degrees
      // double shootercamDiagFOV = 75.0; // degrees
      // Transform2d ballcameraToRobot = new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d()); // meters
      // Transform2d shootercameraToRobot = new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d()); // meters
      // double maxLEDRange = 20;          // meters
      // int ballcamResolutionWidth = 640;     // pixels
      // int ballcamResolutionHeight = 480;    // pixels
      // double ballminTargetArea = 10;        // square pixels
      // int shootercamResolutionWidth = 640;     // pixels
      // int shootercamResolutionHeight = 480;    // pixels
      // double shooterminTargetArea = 10;        // square pixels
    
    NetworkTableInstance.getDefault().getTable("photonvision").getEntry("version").setValue("v2022.1.4");
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive");
    cargoTarget = table.getEntry("BallTarget");
    hubTarget = table.getEntry("HighTarget");
  }

  @Override
  public void periodic() {
    // getCargoTargetStatus();
    // getHighTargetStatus();
  }

  private boolean getHighTargetStatus() {
    // var result = m_limelight.getLatestResult();
    //  hasTargets = result.hasTargets();
    //  hubTarget.setBoolean(hasTargets);
    // return hasTargets;
    return false;
  }

  private boolean getCargoTargetStatus() {
    // var result = m_HD3000.getLatestResult();
    //  hasTargets = result.hasTargets();
    //  cargoTarget.setBoolean(hasTargets);
    // return hasTargets;
    return false;
  }

  public double[] getHubTargetRange() {
    double[] range = {0,0};
    var result = m_limelight.getLatestResult();
    if (m_hasTargets == true) {
      range[0] = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
      range[1] = result.getBestTarget().getYaw();
    }

    return range;
  }

  public double getCargoTargetYaw() {
    double Yaw = 0;
    var result = m_HD3000.getLatestResult();
    if (m_hasTargets == true) {
      Yaw = result.getBestTarget().getYaw();
    }

   return Yaw;
  }

  public void setTeamPipeline() {
    int pipelineIndex;
    String allianceColor;
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      pipelineIndex = 1;
      allianceColor = "Blue Ball";
      targetType.setString(allianceColor);
      m_HD3000.setPipelineIndex(pipelineIndex);
    }else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      pipelineIndex = 0;
      allianceColor = "Red Ball";
      targetType.setString(allianceColor);
      m_HD3000.setPipelineIndex(pipelineIndex);
    }
  }

  public void setLEDon() {
    m_limelight.setLED(VisionLEDMode.kOn);
  }

  public void setLEDoff() {
    m_limelight.setLED(VisionLEDMode.kOff);
  }
}