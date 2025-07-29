// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.utils.RobotStatus;
import frc.robot.utils.RobotStatus.ReefPosition;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class AutoDriveCmd extends Command {
  // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  private final VisionSub m_visionSub;
  private final SwerveRequest.RobotCentric autoDrive = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  Pose2d m_apriltagPos;
  double lrDist;
  double fbDist;
  int counter;
  double lrOffset;
  double fbOffset;
  boolean useOffset;
  private final DrivetrainSub m_drivetrainSub;
  double previousPosx;
  double previousPosy;
  double previousPosRot;
  double posXDifference;
  double posYDifference;
  double posRotDifference;
  //private final LedSub m_ledSub;

  /** Creates a new AutoDriveCmd. */
  public AutoDriveCmd(VisionSub visionSub, DrivetrainSub drivetrainSub, boolean useOffset) { //, LedSub LedSub) {
    m_visionSub = visionSub;
    m_drivetrainSub = drivetrainSub;
    this.useOffset = useOffset;
    //m_ledSub = LedSub;
    addRequirements(drivetrainSub);// Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(RobotStatus.LastReefPosition().equals(ReefPosition.kL2L3Algae)
        || RobotStatus.LastReefPosition().equals(ReefPosition.kL3L4Algae)) {
      fbOffset = 0.457;
    } else {
      fbOffset = 0.48;
      if(RobotStatus.LastReefPosition().equals(RobotStatus.ReefPosition.kL4)) {
        fbOffset += 0.0277; //This is half an inch in meters
      }
    }

    if(useOffset) {
      if(RobotStatus.isLeft()) {
        lrOffset = 0.165;
      } else {
        lrOffset = -0.165;
      }
    } else {
      lrOffset = 0;
    }
    // Use open-loop control for drive motors
    counter = 0;
  }

  public boolean isInFbZone(boolean feedForwardCutout) {
    double deadband = 0.02;
    if(feedForwardCutout) {
      deadband += 0.01;
    }
    return Math.abs(m_apriltagPos.getY()) < fbOffset + deadband && Math.abs(m_apriltagPos.getY()) > fbOffset - deadband;
  }

  public boolean isInLrZone(boolean feedForwardCutout) {
    double deadband = 0.02;
    if(feedForwardCutout) {
      deadband += 0.01;
    }
    return (Math.abs(lrDist) < deadband);
  }

  public boolean isInRotZone(boolean feedForwardCutout) {
    double deadband = 5;
    if(feedForwardCutout) {
      deadband += 0;
    }
    return (Math.abs(m_apriltagPos.getRotation().getDegrees()) < deadband);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_apriltagPos = m_visionSub.getTagPose2d();
    // check if angle is positive or negative
    lrDist = m_apriltagPos.getX() + lrOffset;
    fbDist = m_apriltagPos.getY() + fbOffset;
    double p = 1.75;
    double xPower = lrDist * p;
    double yPower = fbDist * p;
    double fbFeedforward = 0;
    double lrFeedforward = 0;
    double rotFeedforward = 0;

    posXDifference = Math.abs(previousPosx - m_apriltagPos.getX());
    posYDifference = Math.abs(previousPosy - m_apriltagPos.getY());
    posRotDifference = Math.abs(previousPosRot - m_apriltagPos.getRotation().getDegrees());
    SmartDashboard.putNumber("posXDifference", posXDifference);
    SmartDashboard.putNumber("posYDifference", posYDifference);
    SmartDashboard.putNumber("posRotDifference", posRotDifference);


    if(!isInFbZone(true)) {
      fbFeedforward += 0.15 * Math.signum(yPower);
    }

    if(!isInLrZone(true)) {
      lrFeedforward += 0.15 * Math.signum(xPower);
    }

    ChassisSpeeds robotSpeeds = m_drivetrainSub.getRobotRelativeSpeeds();

    if(!isInRotZone(true) && (Math.abs(robotSpeeds.omegaRadiansPerSecond) < 0.035)) {
      rotFeedforward += 0.03 * Math.signum(m_apriltagPos.getRotation().getDegrees());
    }

    m_drivetrainSub.setControl(
        autoDrive.withVelocityX(-yPower - fbFeedforward)
            .withVelocityY(xPower + lrFeedforward)
            .withRotationalRate(
                m_apriltagPos.getRotation().getDegrees() / 5 + rotFeedforward));

    SmartDashboard.putNumber("posXDifference", posXDifference);
    SmartDashboard.putNumber("posYDifference", posYDifference);
    SmartDashboard.putNumber("posRotDifference", posRotDifference);
    SmartDashboard.putNumber("m_visionSub.getTv", m_visionSub.getTv());


    if((m_visionSub.getTv() == 0)
        || ((posXDifference < 0.0075) && (posYDifference < 0.0075) && (posRotDifference < 3))) { //If x difference is less than .01 AND y difference is less than .01 AND rotational difference is less than .025 THEN we are not moving and counter goes up
      counter++;
      System.out.println(counter);
    } else {
      counter = 0;
    }
    //System.out.println(counter);

    // rotate in desired direction until angle is 0
    previousPosx = m_apriltagPos.getX();
    previousPosy = m_apriltagPos.getY();
    previousPosRot = m_apriltagPos.getRotation().getDegrees();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_drivetrainSub.applyRequest(() -> brake);
    m_drivetrainSub.setControl(autoDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if(isInFbZone(false) && isInLrZone(false) && isInRotZone(false)) {
      // System.out.println("5555555555555555555555555 zone condition");
      System.out.println("We reached target zone");
      return true;
    }
    /*
     * } else if(!isInFbZone(false) && isInLrZone(false) && Math.abs(m_apriltagPos.getRotation().getDegrees()) < 5) {
     * System.out.println("Not in fb zone!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1");
     * } else if(isInFbZone(false) && !isInLrZone(false) && Math.abs(m_apriltagPos.getRotation().getDegrees()) < 5) {
     * System.out.println("NOt in lr zone22222222222222222222222222222222222");
     * } else if(isInFbZone(false) && isInLrZone(false) && !(Math.abs(m_apriltagPos.getRotation().getDegrees()) < 5)) {
     * System.out.println("Not finishsed rotation3333333333333333333333333333333");
     * }
     */
    if(counter >= 25) {
      // System.out.println("------------------------------- counter condition");
      System.out.println("We've timed out"); //hey we timed out here//
      return true;
    }
    //m_ledSub.setElevatorColor((byte) 127, (byte) 127, (byte) 127);
    //m_ledSub.setClimbColor((byte) 127, (byte) 127, (byte) 127);
    return false;
  }
}
