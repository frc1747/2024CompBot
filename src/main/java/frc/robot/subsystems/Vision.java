// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private PhotonCamera camera;
  private PhotonPipelineResult returned;
  private List<PhotonTrackedTarget> targets;

 
  // must be modified for each bot
  private final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(23.25);
  private final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(5);
  
  // must be modified for each goal which is not ideal 
  // try to find alternative solution
  private final double TARGET_HEIGHT_METERS = Units.inchesToMeters(36);


  public Vision() {
    camera = new PhotonCamera("Right");
    //returned = camera.getLatestResult();
    updateTargetsList();
  }

  public void updateTargetsList() {
    returned = camera.getLatestResult();
    if (returned.hasTargets() ) {
      targets = returned.getTargets();
      return;
    }
    targets = new ArrayList<PhotonTrackedTarget>();
  }

  public int getTagIndex(int TagID) {
    for (int i = 0; i < targets.size(); i++) {
      if (targets.get(i).getFiducialId() == TagID) {
        return i;
      }
    }
    return -1;
  }

  // not entirely accurate
  public double getTargetsMeters(int index) {
    if (returned.hasTargets()) {
      PhotonTrackedTarget target = targets.get(index);

      double distance = PhotonUtils.calculateDistanceToTargetMeters(
        CAMERA_HEIGHT_METERS,
        TARGET_HEIGHT_METERS,
        CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(target.getPitch()));
      System.out.println("Distance: " + distance + "\n");
      return distance;
    }
    return -1.0;
  }

  public double getYaw(int index) {
    if (returned.hasTargets() ) {
      //fix later
      PhotonTrackedTarget target = targets.get(index);

      return target.getYaw();
    }
    return 0.0;
  }

  public double getPitch(int index) {
    if (returned.hasTargets() ) {
      //fix later
      PhotonTrackedTarget target = targets.get(index);

      return target.getPitch();
    }
    return 0.0;
  }

  public double getSkew(int index) {
    if (returned.hasTargets() ) {
      //fix later
      PhotonTrackedTarget target = targets.get(index);

      return target.getSkew();
    }
    return 0.0;
  }

  public double getArea(int index) {
    if (returned.hasTargets() ) {
      //fix later
      PhotonTrackedTarget target = targets.get(index);

      return target.getArea();
    }
    return 0.0;
  }

  public double getAmbiguity(int index) {
    if (returned.hasTargets() ) {
      //fix later
      List<PhotonTrackedTarget> targets = returned.getTargets();
      PhotonTrackedTarget target = targets.get(index);

      return target.getPoseAmbiguity();
    }
    return 0.0;
  }

  public double getTrueCenter(int index) {
    // you have to change for each bot
    double trueCenter = (Math.asin(Units.inchesToMeters(7) / getTargetsMeters(index)));

    return trueCenter;
  }
}
    
        





     /*    
        int id = tar.getFiducialId();
        decoder.add(id);




      }
       this code is trying to make to list of the targets and what target is the index.
      */







/*

    This code is to print out the values of photon for target one 
     PhotonTrackedTarget target = targets.get(0);

    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double area = target.getArea();
    double skew = target.getSkew();

    System.out.printf("f% yaw /n , %f pitch /n ,%f  area /n,%f skew /n ",yaw,pitch,area,skew);

*/