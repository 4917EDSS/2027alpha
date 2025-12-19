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

  //Creates a memebr variable for the drivetrainsub
  private final DrivetrainSub m_drivetrainSub;
  //creates a new robot centric swerve request
  private final SwerveRequest.RobotCentric autoPath = new SwerveRequest.RobotCentric();
  //creates a new varible for the fieldimage utility
  FieldImage fieldImage = new FieldImage();
  //Creates a new arraylist to store the path
  ArrayList<int[]> path = new ArrayList<int[]>();
  //creates additional variables, refer to pathgencmd for their purpose
  int conversionFactor;
  int[] currentPos = new int[2];
  int fieldLength = 57; //I actually have no idea, were gonna have to figure this one out
  //creates a coord to store the target of the drivetrain
  public int[] driveTargetPos;

  /** Creates a new PathFollowCmd. */
  public PathFollowCmd(DrivetrainSub drivetrainSub, int[] target) {
    //gets values
    driveTargetPos = target;
    conversionFactor = fieldLength / fieldImage.field.length;
    m_drivetrainSub = drivetrainSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //sets the intended final pos of the robot in the pathfollowtargetpos util
    PathFollowTargetPos.finalPos = driveTargetPos;
    //sets the current pos to the pos of the robot
    currentPos[0] = (int) Math.round(m_drivetrainSub.getPose().getX());
    currentPos[1] = (int) Math.round(m_drivetrainSub.getPose().getY());
    //double xPosDiff = m_drivetrainSub.getPose().getX() * conversionFactor - m_drivetrainSub.getPose().getX();
    //double yPosDiff = m_drivetrainSub.getPose().getY() * conversionFactor - m_drivetrainSub.getPose().getY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //sets the current pos to the pos of the robot
    currentPos[0] = (int) Math.round(m_drivetrainSub.getPose().getX());
    currentPos[1] = (int) Math.round(m_drivetrainSub.getPose().getY());
    //sets the intended starting pos of the robot in the pathfollowtargetpos util
    PathFollowTargetPos.startingPos = currentPos;
    //2d vector representing the distance between robot and target
    double[] velocityVector = {(PathFollowTargetPos.currentTarget[0] - PathFollowTargetPos.startingPos[0]),
        PathFollowTargetPos.currentTarget[1] - PathFollowTargetPos.startingPos[1]};
    double magnitude = Math.sqrt(velocityVector[0] * velocityVector[0] + velocityVector[1] * velocityVector[1]);
    //normalizes the vector to 1
    velocityVector[0] /= magnitude;
    velocityVector[1] /= magnitude;
    //sets the direction of the wheels
    m_drivetrainSub.setControl(
        autoPath.withVelocityX(velocityVector[0])
            .withVelocityY(velocityVector[1])
            .withRotationalRate(0.0));
    //path = m_pathGenCmd.generatePath(null, null, null);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stops teh driving
    m_drivetrainSub.setControl(autoPath.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //checks if the the robot has reached the end of its path (starting pos moves with the robots pos, current target is where it is going)
    if(PathFollowTargetPos.startingPos[0] == PathFollowTargetPos.currentTarget[0]
        && PathFollowTargetPos.startingPos[1] == PathFollowTargetPos.currentTarget[1]) {
      return true;
    }
    return false;
  }
}
