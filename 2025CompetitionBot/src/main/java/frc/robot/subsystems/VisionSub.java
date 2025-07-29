// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.LimelightHelpers;

public class VisionSub extends SubsystemBase {
  private static String LEFT = "limelight-left";
  private static String RIGHT = "limelight-right";
  private static Logger m_logger = Logger.getLogger(VisionSub.class.getName());

  LimelightHelpers.PoseEstimate mt2;
  double m_previousTimestamp = 0.0;// Map<String, Double> m_previousTimestamps = Map.of(LEFT, 0.0);//, RIGHT, 0.0);
  DrivetrainSub m_drivetrainSub;

  NetworkTable m_networkTableL = NetworkTableInstance.getDefault().getTable(LEFT);
  NetworkTable m_networkTableR = NetworkTableInstance.getDefault().getTable(RIGHT);
  NetworkTable m_mainNetworkTable;

  NetworkTableEntry m_tidL;
  NetworkTableEntry m_t2dL;
  NetworkTableEntry m_tvL;
  NetworkTableEntry m_txL;
  NetworkTableEntry m_tyL;
  NetworkTableEntry m_taL;
  NetworkTableEntry m_pipelineL;
  NetworkTableEntry m_pipetypeL;
  NetworkTableEntry m_botposeTargetL;
  NetworkTableEntry m_botposeL;

  NetworkTableEntry m_tidR;
  NetworkTableEntry m_t2dR;
  NetworkTableEntry m_tvR;
  NetworkTableEntry m_txR;
  NetworkTableEntry m_tyR;
  NetworkTableEntry m_taR;
  NetworkTableEntry m_pipelineR;
  NetworkTableEntry m_pipetypeR;
  NetworkTableEntry m_botposeTargetR;
  NetworkTableEntry m_botposeR;

  long id;
  double[] t2d;
  long tv;
  double x;
  double y;
  double a;
  long pipeline;
  String pipetype;
  double[] botposeTarget;
  double[] botpose;

  int m_printPosCounter = 0;

  /** Creates a new VisionSub. */
  public VisionSub(DrivetrainSub drivetrainSub) {
    m_t2dL = m_networkTableL.getEntry("t2d");
    m_tidL = m_networkTableL.getEntry("tid");
    m_tvL = m_networkTableL.getEntry("tv");
    m_txL = m_networkTableL.getEntry("tx");
    m_tyL = m_networkTableL.getEntry("ty");
    m_taL = m_networkTableL.getEntry("ta");
    m_pipelineL = m_networkTableL.getEntry("getpipe");
    m_pipetypeL = m_networkTableL.getEntry("getpipetype");
    m_botposeTargetL = m_networkTableL.getEntry("botpose_targetspace");
    m_botposeL = m_networkTableL.getEntry("botpose");

    m_t2dR = m_networkTableR.getEntry("t2d");
    m_tidR = m_networkTableR.getEntry("tid");
    m_tvR = m_networkTableR.getEntry("tv");
    m_txR = m_networkTableR.getEntry("tx");
    m_tyR = m_networkTableR.getEntry("ty");
    m_taR = m_networkTableR.getEntry("ta");
    m_pipelineR = m_networkTableR.getEntry("getpipe");
    m_pipetypeR = m_networkTableR.getEntry("getpipetype");
    m_botposeTargetR = m_networkTableR.getEntry("botpose_targetspace");
    m_botposeR = m_networkTableR.getEntry("botpose");

    m_drivetrainSub = drivetrainSub;
    init();
  }

  public void init() {
    m_logger.info("Initializing VisionSub Subsystem");
  }

  @Override
  public void periodic() {
    if(m_taL.getDouble(0) > m_taR.getDouble(0)) {
      id = m_tidL.getInteger(0);
      t2d = m_t2dL.getDoubleArray(new double[2]);
      tv = m_tvL.getInteger(0);
      x = m_txL.getDouble(0.0);
      y = m_tyL.getDouble(0.0);
      a = m_taL.getDouble(0.0);
      pipeline = m_pipelineL.getInteger(0);
      pipetype = m_pipetypeL.getString("");
      botposeTarget = m_botposeTargetL.getDoubleArray(new double[8]);
      botpose = m_botposeL.getDoubleArray(new double[8]);
      SmartDashboard.putBoolean("Vi Use Left LL", true);
    } else {
      id = m_tidR.getInteger(0);
      t2d = m_t2dR.getDoubleArray(new double[2]);
      tv = m_tvR.getInteger(-1);
      x = m_txR.getDouble(0.0);
      y = m_tyR.getDouble(0.0);
      a = m_taR.getDouble(0.0);
      pipeline = m_pipelineR.getInteger(0);
      pipetype = m_pipetypeR.getString("");
      botposeTarget = m_botposeTargetR.getDoubleArray(new double[8]);
      botpose = m_botposeR.getDoubleArray(new double[8]);
      SmartDashboard.putBoolean("Vi Use Left LL", false);
    }
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Vi Primary ID", id);
    SmartDashboard.putNumber("Vi Sees Tag", t2d[1]);
    SmartDashboard.putNumber("Vi # of Tags", tv);
    SmartDashboard.putNumber("Vi Tag X", x);
    SmartDashboard.putNumber("Vi Tag Y", y);
    SmartDashboard.putNumber("Vi Tag Area", a);
    SmartDashboard.putNumber("Vi Pipeline", pipeline);
    SmartDashboard.putString("Vi Pipetype", pipetype);
    // SmartDashboard.putString("Main Limelight:", "none");

    updateOdometry(m_drivetrainSub.getState());
  }

  public Pose2d getTagPose2d() {
    return new Pose2d(botposeTarget[0], botposeTarget[2], new Rotation2d((botposeTarget[4] / 360.0) * 2 * Math.PI));
  }

  public long getTv() {
    return tv;
  }

  public double getTx() {
    return x;
  }

  public boolean isFarFromAprilTag() {
    //y distance is negative
    if(botposeTarget[2] < Constants.Vision.kDistanceToCloseToDrive) {
      return true;
    } else {
      return false;
    }
  }

  private void updateOdometry(SwerveDriveState swerveDriveState) {
    updateOdemetry(swerveDriveState, LEFT);
    updateOdemetry(swerveDriveState, RIGHT);
  }

  private void updateOdemetry(SwerveDriveState swerveDriveState, String camera) {
    LimelightHelpers.SetRobotOrientation(camera, swerveDriveState.Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);
    if(mt2 == null) {
      return;
    }
    double timestamp = mt2.timestampSeconds;

    if(timestamp > m_previousTimestamp) {
      m_previousTimestamp = timestamp;
      double standardDeviation = 0.7; // 0.7 is a good starting value according to limelight docs.

      if(Math.abs(swerveDriveState.Speeds.omegaRadiansPerSecond) > Math.PI) // if our angular velocity is greater than
                                                                            // 360 degrees per second, ignore vision
                                                                            // updates
      {
        return;
      }
      if(mt2.tagCount == 0 || mt2.avgTagArea == 0) {
        return;
      }
      standardDeviation = (standardDeviation / mt2.tagCount) / (mt2.avgTagArea * 15.0);

      m_drivetrainSub.addVisionMeasurement(
          mt2.pose,
          // Always pass 999999 as the last argument, as megatag 2 requires heading as
          // input, so it does not actually calculate heading.
          // Passing in a very large number to that parameter basically tells the Kalman
          // filter to ignore our calculated heading.
          com.ctre.phoenix6.Utils.fpgaToCurrentTime(timestamp),
          VecBuilder.fill(standardDeviation, standardDeviation, 9999999));

    }

  }

}
