package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Vision extends SubsystemBase {
  PhotonCamera m_camera;

  ShuffleboardTab m_visionTab;
  NetworkTableEntry m_tx, m_ty, m_ta, m_bestTarget, targetPitch, targetYaw, targetRange, targetCount, targetType;
  PhotonTrackedTarget m_target;

  double pitch, yaw, area;

  private boolean hasTargets;

  public Vision() {
    m_camera = new PhotonCamera("eagletron");
    
    m_visionTab = Shuffleboard.getTab("Vision");
    m_visionTab.addBoolean("Target Found", this::getTargetStatus);

    targetType = m_visionTab.add("Target Type", " ").getEntry();
  }

  @Override
  public void periodic() {
  }

  private boolean getTargetStatus() {
    var result = m_camera.getLatestResult();
     hasTargets = result.hasTargets();
    return hasTargets;
  }

  public double[] getTargetRange() {
    double[] range = {0,0};
    var result = m_camera.getLatestResult();
    if (hasTargets == true) {
      range[0] = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
      range[1] = result.getBestTarget().getYaw();
    }

    return range;
  }

  public void setTeamPipeline() {
    int pipelineIndex;
    String allianceColor;
    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      pipelineIndex = 0;
      allianceColor = "Blue Ball";
      targetType.setString(allianceColor);
      m_camera.setPipelineIndex(pipelineIndex);
    }else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      pipelineIndex = 1;
      allianceColor = "Red Ball";
      targetType.setString(allianceColor);
      m_camera.setPipelineIndex(pipelineIndex);
    }
  }

  public void targetMode(int targetIndex) {
    m_camera.setPipelineIndex(targetIndex);
    targetType.setString("Hub");
  }
}