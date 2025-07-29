// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.DrivetrainSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class BackUpAfterScoringCmd extends DeferredCommand {

  private static Command returnAutoBuilder(DrivetrainSub drivetrainSub) {

    Pose2d backupPose = new Pose2d(
        drivetrainSub.getPose().getTranslation()
            .plus(new Translation2d(0.75, drivetrainSub.getPose().getRotation().minus(Rotation2d.k180deg))),
        drivetrainSub.getPose().getRotation());
    return new DriveToPoseCmd(backupPose, drivetrainSub, false);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  /** Creates a new BackUpAfterScoringCmd. */
  public BackUpAfterScoringCmd(DrivetrainSub drivetrainSub) {
    super(() -> returnAutoBuilder(drivetrainSub), Set.of(drivetrainSub));
  }
}
