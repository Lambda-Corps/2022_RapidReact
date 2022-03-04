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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Vision extends SubsystemBase {
  int LIMELIGHTPIPELINE = 0;
  int HD3000PIPELINE = 0;

  PhotonCamera m_HD3000 = new PhotonCamera("lifecam");
  PhotonCamera m_limelight = new PhotonCamera("eagletron");

  ShuffleboardTab m_visionTab;
  NetworkTableEntry m_tx, m_ty, m_ta, m_bestTarget, targetPitch, targetYaw, targetRange, targetCount, targetType;
  PhotonTrackedTarget m_target;

  double pitch, yaw, area;

  private boolean hasTargets;

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
  }

  @Override
  public void periodic() {
  }

  private boolean getHighTargetStatus() {
    var result = m_limelight.getLatestResult();
     hasTargets = result.hasTargets();
    return hasTargets;
  }

  private boolean getCargoTargetStatus(){
    var result = m_HD3000.getLatestResult();
     hasTargets = result.hasTargets();
    return hasTargets;
  }

  public double[] getTargetRange() {
    double[] range = {0,0};
    var result = m_limelight.getLatestResult();
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
      m_HD3000.setPipelineIndex(pipelineIndex);
    }else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      pipelineIndex = 1;
      allianceColor = "Red Ball";
      targetType.setString(allianceColor);
      m_HD3000.setPipelineIndex(pipelineIndex);
    }
  }

  public void targetMode(int targetIndex) {
    m_limelight.setPipelineIndex(targetIndex);
    targetType.setString("Hub");
  }
}