// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.utils.RobotStatus;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class DriveToNearestScoreLocationCmd extends SelectCommand<Translation2d> {
  // All in meters.
  private static final Translation2d MIDDLE_OF_BLUE_REEF = new Translation2d(4.4886, 4.0315);
  private static final Translation2d MIDDLE_OF_RED_REEF = new Translation2d(13.0594, 4.0315);
  private static final double MIDDLE_OF_REEF_TO_SCORING_FACE = 0.831;
  private static final double MIDDLE_SCORING_FACE_TO_BRANCH = 0.164;
  private static final double REEF_TO_MIDDLE_OF_ROBOT_SCORING = 0.381 + 0.19; // Robot width plus distance from score
  private static List<Pose2d> s_targetPoses = new ArrayList<Pose2d>();
  private static List<Translation2d> s_targetTranslationsL = new ArrayList<Translation2d>();
  private static List<Translation2d> s_targetTranslationsR = new ArrayList<Translation2d>();

  /**
   * Some basic coordinates summarized from
   * https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
   * 
   * +y
   * ---------------------------
   * | blue red | 90
   * | | 180 0
   * | | 270
   * |0,0 |
   * --------------------------- +x
   * 
   * With a top down view where the blue side is the left side, 0,0 is the bottom
   * left corner.
   * x increases going to the right (away from blue towards red)
   * y increases going up (to the left from the perspective of a blue driver)
   * 0 degrees is pointed towards red's driver station
   * 90 degrees is pointed "up"
   * 180 is pointed towards blue
   * 270 is "down"
   */
  private static void generateTargetPoses() {
    for(Translation2d middleOfReef : Arrays.asList(MIDDLE_OF_BLUE_REEF, MIDDLE_OF_RED_REEF)) {
      for(double degrees = 0; degrees < 360; degrees += 60) {
        Rotation2d robotsRotation = Rotation2d.fromDegrees(degrees);
        // The scoring face "looks" at the robot. If the robot is scoring with heading 0
        // (facing
        // red), the actual scoring side is the 180 angle (towards blue).
        Rotation2d scoringFaceRotation = robotsRotation.minus(Rotation2d.k180deg);
        Translation2d reefToScoringFace = new Translation2d(
            MIDDLE_OF_REEF_TO_SCORING_FACE + REEF_TO_MIDDLE_OF_ROBOT_SCORING, scoringFaceRotation);
        Translation2d scoringFaceToBranch = new Translation2d(MIDDLE_SCORING_FACE_TO_BRANCH,
            scoringFaceRotation.plus(Rotation2d.kCW_90deg));

        Translation2d finalTranslationL = middleOfReef.plus(reefToScoringFace).plus(scoringFaceToBranch);
        Translation2d finalTranslationR = middleOfReef.plus(reefToScoringFace).minus(scoringFaceToBranch);
        s_targetPoses.add(new Pose2d(finalTranslationL, robotsRotation));
        s_targetPoses.add(new Pose2d(finalTranslationR, robotsRotation));
        s_targetTranslationsL.add(finalTranslationL);
        s_targetTranslationsR.add(finalTranslationR);
      }
    }
  }

  private static Map<Translation2d, Command> s_locationToCommandMap = new HashMap<Translation2d, Command>();

  public static void warmUpMap(DrivetrainSub drivetrainSub) {
    generateTargetPoses();
    for(Pose2d targetPose : s_targetPoses) {
      s_locationToCommandMap.put(targetPose.getTranslation(), new DriveToPoseCmd(targetPose, drivetrainSub));
    }
    assert s_locationToCommandMap.size() == 2 * 6 * 2; // 2 reefs, 6 faces, 2 scoring locations per
  }

  private static Translation2d getClosest(Pose2d location) {
    boolean useLeft = RobotStatus.isLeft();
    if(RobotStatus.getAlliance() == Alliance.Red) {
      // Swap what "left" means depending on our perspective.
      useLeft = !useLeft;
    }
    if(useLeft) {
      return location.getTranslation().nearest(s_targetTranslationsL);
    } else {
      return location.getTranslation().nearest(s_targetTranslationsR);
    }
  }

  /** Command for the closest one. */
  public DriveToNearestScoreLocationCmd(DrivetrainSub drivetrainSub) {
    super(s_locationToCommandMap, () -> getClosest(drivetrainSub.getPose()));
  }
}
