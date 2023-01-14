package frc.robot.subsystems;
import static frc.robot.Constants.*;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  int LIMELIGHTPIPELINE = 0;
  int HD3000PIPELINE = 0;

  // PhotonCamera m_HD3000 = new PhotonCamera("lifecam");
  PhotonCamera m_limelight = new PhotonCamera("eagletron");
 
  ShuffleboardTab m_visionTab;
  NetworkTableEntry m_tx, m_ty, m_ta, m_cargoYaw, m_cargoHasTargets, m_bestTarget, targetPitch, targetYaw, targetRange, targetCount, cargoTarget, hubTarget;
  PhotonTrackedTarget m_target;

  double pitch, yaw, area;
  int pipelineIndex;

  public Vision() {
      m_limelight.setPipelineIndex(LIMELIGHTPIPELINE);
      // m_HD3000.setPipelineIndex(HD3000PIPELINE);

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
    
    NetworkTableInstance.getDefault().getTable("photonvision").getEntry("version").setValue("v2022.1.5");
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Drive");
    cargoTarget = table.getEntry("BallTarget");
    hubTarget = table.getEntry("HighTarget");

    NetworkTable cargoTable = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("lifecam");
    m_cargoYaw = cargoTable.getEntry("targetYaw");
    m_cargoHasTargets = cargoTable.getEntry("hasTarget");
  }

  @Override
  public void periodic() {
    getCargoTargetStatus();
    getHighTargetStatus();
  }

  private boolean getHighTargetStatus() {
    var result = m_limelight.getLatestResult();
     boolean hasTargets = result.hasTargets();
     hubTarget.setBoolean(hasTargets);
    return hasTargets;
  }

  private boolean getCargoTargetStatus() {
    // var result = m_HD3000.getLatestResult();
    //  boolean hasTargets = result.hasTargets();
    //  cargoTarget.setBoolean(hasTargets);
    // return hasTargets;
    return false;
  }

  public double[] getHubTargetRange() {
    double[] range = {0,0};
    var result = m_limelight.getLatestResult();
    if (result.hasTargets()) {
      range[0] = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, 
                                                             TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, 
                                                             Units.degreesToRadians(result.getBestTarget().getPitch()));
      range[1] = result.getBestTarget().getYaw();
    }

    return range;
  }
  
  public double getHubTargetRangeIndex0() {
    return getHubTargetRange()[0];
  }

  public double getHubTargetRangeIndex1() {
    return getHubTargetRange()[1];
  }

  public double getCargoTargetYaw() {
    double Yaw = m_cargoYaw.getDouble(0);
    return Yaw;
  }

  public void setTeamPipeline() {
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      pipelineIndex = 1;
      // m_HD3000.setPipelineIndex(pipelineIndex);
    }else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      pipelineIndex = 0;
      // m_HD3000.setPipelineIndex(pipelineIndex);
    }
  }

  public void setLEDon() {
    m_limelight.setLED(VisionLEDMode.kOn);
  }

  public void setLEDoff() {
    m_limelight.setLED(VisionLEDMode.kOff);
  }
}