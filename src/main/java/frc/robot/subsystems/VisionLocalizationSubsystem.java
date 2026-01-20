// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionLocalizationSubsystem extends SubsystemBase {
  /** Creates a new VisionLocalizationSubsystem. */
  Matrix<N3, N1> odometryStd =
      VecBuilder.fill(
          0.002, 0.002, 0.0035); // These are placeholder numbers calibrate later: x, y, z

  Matrix<N3, N1> visionStd = VecBuilder.fill(0.002, 0.002, 0.007); // N3 and N1 matrix dimensions
  SwerveDriveKinematics m_kinematics;
  private Detection[] detections;
  private ObjectTrackerSubsystem m_ots;
  private DrivetrainSubsystem m_dts;
  private int[] tagIds;
  private boolean hasIntializedPose = false;
  private Pose2d initialPose;

  public VisionLocalizationSubsystem(ObjectTrackerSubsystem ots, DrivetrainSubsystem dts) {
    m_ots = ots;
    m_dts = dts;
    this.m_kinematics = m_dts.m_kinematics;

    tagIds = new int[Constants.APRIL_TAG_POSITIONS.length - 1];
    for (int i = 1; i <= tagIds.length; i++) {
      tagIds[i - 1] = i;
    }
  }

  public Transform2d visionAutoData(double xPrime, double zPrime, double finalYa, int tagId) {
    Detection detection = null;
    for (int i = 0; i < 100; i++) {
      try {
        detection = m_ots.getSpecificAprilTag(tagId);
        break;
      } catch (Exception e) {
        System.out.println("Failed vision attempt " + i);
      }
    }

    if (detection == null) {
      SmartDashboard.putBoolean("ableToSeeAT", false);
      return null;
    }

    SmartDashboard.putBoolean("ableToSeeAT", true);

    double visionYa = -detection.ya;

    double x_vt =
        xPrime * Math.cos(Math.toRadians(visionYa)) - zPrime * Math.sin(Math.toRadians(visionYa));

    double z_vt =
        xPrime * Math.sin(Math.toRadians(visionYa)) + zPrime * Math.cos(Math.toRadians(visionYa));

    double deltaRobotX = -(detection.x + x_vt - Constants.CAM_X_OFFSET);
    double deltaRobotY = -(detection.z + z_vt - Constants.CAM_Y_OFFSET);

    double finalAngle = visionYa + finalYa;

    return new Transform2d(
        new Translation2d(Units.inchesToMeters(deltaRobotX), Units.inchesToMeters(deltaRobotY)),
        Rotation2d.fromDegrees(finalAngle));
  }

  public Pose2d visionToFieldPose(Transform2d visionPose, int tagID) {
    Pose2d tagPose = Constants.APRIL_TAG_POSITIONS[tagID];
    if (tagPose == null || visionPose == null) {
      return null;
    }
    Pose2d feildPose = tagPose.transformBy(visionPose);
    return feildPose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    detections = m_ots.getAllAprilTagDetections(tagIds);
    for (int i = 0; i < detections.length; i++) {
      if (detections[i] != null) {
        int tagId = Integer.parseInt(detections[i].objectLabel.substring(10));
        // if (!hasIntializedPose) {
        //   initialPose = visionToFieldPose(visionAutoData(0, 0, 0, i), i);

        // Why did we had a constant 1 for aprilTag id here?
        // So I commented it out, it might be the reason why nothing worked
        // initialPose = visionToFieldPose(visionAutoData(0, 0, 0, 1), i);
        // }
        try {
          m_dts.m_odometry.addVisionMeasurement(
              visionToFieldPose(visionAutoData(0, 0, 0, tagId), tagId), Timer.getFPGATimestamp());
              SmartDashboard.putNumber("visionAutoDataX", visionAutoData(0, 0, 0, tagId).getX());
              SmartDashboard.putNumber("visionAutoDataY", visionAutoData(0, 0, 0, tagId).getY());
            // System.out.println("Successfully add vision meas tag for " + tagId + "SUCESSS");
        } catch (Exception e) {
        //  System.out.println("Failed to add vision measurement for tag " + tagId);
        }

      } else {
        // System.out.println("No detection for tag " + i);
      }
    }
    // This is done in DrivetrainSubsystem now, so commented out
    // m_dts.m_odometry.updateWithTime(
    //     Timer.getFPGATimestamp(), m_dts.getGyroAngle(), m_dts.getSwerveModulePositions());
      m_dts.m_odometry.updateWithTime(Timer.getFPGATimestamp(), m_dts.getGyroAngle(),
      new SwerveModulePosition[]{
        new SwerveModulePosition(0, new Rotation2d(0)),
        new SwerveModulePosition(0, new Rotation2d(0)),
        new SwerveModulePosition(0, new Rotation2d(0)),
        new SwerveModulePosition(0, new Rotation2d(0)),
      });
    // Need x, z, ya from each
  }
}
