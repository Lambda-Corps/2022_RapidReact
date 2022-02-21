package frc.robot.subsystems;

import java.util.List;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
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
    getTargetStatus();
    getPhotonTargets();
  }

  public boolean getTargetStatus() {
    var result = m_camera.getLatestResult();
     hasTargets = result.hasTargets();
    return hasTargets;
  }

  private void getPhotonTargets() {
    var result = m_camera.getLatestResult();
    if (hasTargets == true) {
      List<PhotonTrackedTarget> targets = result.getTargets();
      m_target = result.getBestTarget();
      
      double yaw = m_target.getYaw();
      double pitch = m_target.getPitch();

      targetPitch = m_visionTab.add("Target Y", pitch).getEntry();
      targetYaw = m_visionTab.add("Target X", yaw).getEntry();

      if (result.hasTargets()) {
        double range = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, Units.degreesToRadians(pitch));
        targetRange = m_visionTab.add("Target Range", range).getEntry();
      }

      int listSize = targets.size();
      targetCount = m_visionTab.add("Target Count", listSize).getEntry();
    }
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