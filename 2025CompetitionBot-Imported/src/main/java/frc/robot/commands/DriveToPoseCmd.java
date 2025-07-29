// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class DriveToPoseCmd extends Command {
  private DrivetrainSub m_drivetrainSub;
  private Pose2d m_targetPose;
  private Pose2d m_currentPose;
  private Transform2d m_error;

  private final SwerveRequest.FieldCentric backDrive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.Velocity).withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  private final double m_driveP = 5.0;
  private final double m_rotP = 0.1;

  private double m_disTolerance = 0.25; // m
  private final double PRECISE_DIS_TOLERANCE = 0.02;
  private double m_speedTolerance = 0.5; // m/s
  private final double PRECISE_SPEED_TOLERANCE = 0.2;
  private double m_angTolerance = 15; // degrees
  private final double PRECISE_ANG_TOLERANCE = 1.0;
  private double m_angSpeedTolerance = Math.PI / 10.0; // rad/s
  private final double PRECISE_ANG_SPEED_TOLERANCE = Math.PI / 30.0;

  /** Creates a new DriveToPoseCmd. */
  public DriveToPoseCmd(Pose2d targetPose, DrivetrainSub drivetrainSub) {
    this(targetPose, drivetrainSub, true);
  }

  public DriveToPoseCmd(Pose2d targetPose, DrivetrainSub drivetrainSub, boolean precise) {
    m_targetPose = targetPose;
    m_drivetrainSub = drivetrainSub;
    addRequirements(m_drivetrainSub);
    if(precise) {
      m_disTolerance = PRECISE_DIS_TOLERANCE;
      m_speedTolerance = PRECISE_SPEED_TOLERANCE;
      m_angTolerance = PRECISE_ANG_TOLERANCE;
      m_angSpeedTolerance = PRECISE_ANG_SPEED_TOLERANCE;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_error = new Transform2d();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentPose = m_drivetrainSub.getPose();

    // Minus and relative seem to do something completely different than expected
    m_error = new Transform2d(m_targetPose.getX() - m_currentPose.getX(), m_targetPose.getY() - m_currentPose.getY(),
        m_targetPose.getRotation().minus(m_currentPose.getRotation()));
    // P
    Translation2d outputDrivePower = m_error.getTranslation().times(m_driveP);
    double outputRotPower = m_error.getRotation().getDegrees() * m_rotP;

    double normPower = outputDrivePower.getNorm();
    if(normPower < 0.1) {
      outputDrivePower = outputDrivePower.times((0.1 / normPower));
    } else if(normPower > 3.0) {
      outputDrivePower = outputDrivePower.times((3.0 / normPower));
    }

    if(Math.abs(outputRotPower) < 0.1) {
      outputRotPower = outputRotPower * ((0.1 / Math.abs(outputRotPower)));
    } else if(Math.abs(outputRotPower) > 50) {
      outputRotPower = outputRotPower * ((50 / Math.abs(outputRotPower)));
    }

    // I believe setControl is just a less confusing version of applyRequest.
    m_drivetrainSub.setControl(backDrive.withVelocityX(outputDrivePower.getX())
        .withVelocityY(outputDrivePower.getY())
        .withRotationalRate(outputRotPower));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.setControl(backDrive.withVelocityX(0.0).withVelocityY(0.0).withVelocityY(0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double speed = Math.sqrt(((Math.pow(m_drivetrainSub.getRobotRelativeSpeeds().vxMetersPerSecond, 2))
        + (Math.pow(m_drivetrainSub.getRobotRelativeSpeeds().vyMetersPerSecond, 2))));
    if(m_error.getTranslation().getNorm() < m_disTolerance
        && Math.abs(m_error.getRotation().getDegrees()) < m_angTolerance
        && Math.abs(m_drivetrainSub.getRobotRelativeSpeeds().omegaRadiansPerSecond) < m_angSpeedTolerance
        && speed < m_speedTolerance) {
      return true;
    }
    return false;
  }
}
