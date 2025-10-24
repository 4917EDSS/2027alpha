// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.utils.FieldImage;
import frc.robot.utils.PathFollowTargetPos;
import java.util.ArrayList;
import com.ctre.phoenix6.swerve.SwerveRequest;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class PathFollowCmd extends Command {

  private final DrivetrainSub m_drivetrainSub;
  private final SwerveRequest.RobotCentric autoPath = new SwerveRequest.RobotCentric();
  FieldImage fieldImage = new FieldImage();
  ArrayList<int[]> path = new ArrayList<int[]>();
  int conversionFactor;
  int[] currentPos = new int[2];
  int fieldLength = 57; //I actually have no idea, were gonna have to figure this one out
  public int[] driveTargetPos;

  /** Creates a new PathFollowCmd. */
  public PathFollowCmd(DrivetrainSub drivetrainSub, int[] target) {
    driveTargetPos = target;
    conversionFactor = fieldLength / fieldImage.field.length;
    m_drivetrainSub = drivetrainSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathFollowTargetPos.finalPos = driveTargetPos;
    currentPos[0] = (int) Math.round(m_drivetrainSub.getPose().getX());
    currentPos[1] = (int) Math.round(m_drivetrainSub.getPose().getY());
    //double xPosDiff = m_drivetrainSub.getPose().getX() * conversionFactor - m_drivetrainSub.getPose().getX();
    //double yPosDiff = m_drivetrainSub.getPose().getY() * conversionFactor - m_drivetrainSub.getPose().getY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPos[0] = (int) Math.round(m_drivetrainSub.getPose().getX());
    currentPos[1] = (int) Math.round(m_drivetrainSub.getPose().getY());
    PathFollowTargetPos.startingPos = currentPos;
    double[] velocityVector = {(PathFollowTargetPos.currentTarget[0] - PathFollowTargetPos.startingPos[0]),
        PathFollowTargetPos.currentTarget[1] - PathFollowTargetPos.startingPos[1]};
    double magnitude = Math.sqrt(velocityVector[0] * velocityVector[0] + velocityVector[1] * velocityVector[1]);
    velocityVector[0] /= magnitude;
    velocityVector[1] /= magnitude;
    m_drivetrainSub.setControl(
        autoPath.withVelocityX(velocityVector[0])
            .withVelocityY(velocityVector[1])
            .withRotationalRate(0.0));
    //path = m_pathGenCmd.generatePath(null, null, null);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.setControl(autoPath.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(PathFollowTargetPos.startingPos[0] == PathFollowTargetPos.currentTarget[0]
        && PathFollowTargetPos.startingPos[1] == PathFollowTargetPos.currentTarget[1]) {
      return true;
    }
    return false;
  }
}
