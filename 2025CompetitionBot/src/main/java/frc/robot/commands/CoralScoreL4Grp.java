// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.CanSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.VisionSub;
import frc.robot.utils.RobotStatus;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralScoreL4Grp extends SequentialCommandGroup {
  public CoralScoreL4Grp(ArmSub armSub, CanSub canSub, DrivetrainSub drivetrainSub,
      ElevatorSub elevatorSub, VisionSub visionSub) {
    this(armSub, canSub, drivetrainSub, elevatorSub, visionSub, false);
  }

  /** Creates a new AutoCoralScoreL4Grp. */
  public CoralScoreL4Grp(ArmSub armSub, CanSub canSub, DrivetrainSub drivetrainSub,
      ElevatorSub elevatorSub, VisionSub visionSub, boolean forauto) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if(forauto) {
      addCommands(
          new ParallelCommandGroup(
              new MoveElArmGrp(Constants.Elevator.kL4PreScoreHeight, Constants.Arm.kL4PreScoreAngle, armSub,
                  elevatorSub), //Move to pre score position
              new AutoDriveCmd(visionSub, drivetrainSub, true) //Drive to score location
          ),
          new MoveElArmDeadlineGrp(Constants.Elevator.kL4PostScoreHeight, Constants.Arm.kL4PostScoreAngle, armSub,
              elevatorSub) //Move to post score location (score)
      );
    } else {
      addCommands(
          // If further away
          new ConditionalCommand(
              new ParallelCommandGroup(
                  new MoveElArmGrp(Constants.Elevator.kL4PreScoreHeight, Constants.Arm.kL4PreScoreAngle, armSub,
                      elevatorSub), //Move to pre score position
                  new AutoDriveCmd(visionSub, drivetrainSub, true) //Drive to score location
              ),
              // Else
              new SequentialCommandGroup(
                  new MoveElArmGrp(Constants.Elevator.kL4PreScoreHeight, Constants.Arm.kL4PreScoreAngle, armSub,
                      elevatorSub), //Move to pre score position
                  new AutoDriveCmd(visionSub, drivetrainSub, true) //Drive to score location
              ),
              () -> visionSub.isFarFromAprilTag()),

          new MoveElArmDeadlineGrp(Constants.Elevator.kL4PostScoreHeight, Constants.Arm.kL4PostScoreAngle, armSub,
              elevatorSub), //Move to post score location (score)

          new ConditionalCommand(
              new ParallelDeadlineGroup(
                  new BackUpAfterScoringCmd(drivetrainSub), //Back up
                  new MoveElArmGrp(Constants.Elevator.kCoralGrabbableHeight, Constants.Arm.kCoralGrabbableAngle, armSub,
                      elevatorSub)),
              new BackUpAfterScoringCmd(drivetrainSub), //Back up
              () -> RobotStatus.getClearNextAlgae()),
          new InstantCommand(() -> RobotStatus.clearClearNextAlgae()),

          new ScheduleCommand(new GrabCoralTeleopGrp(armSub, canSub, elevatorSub)) //Grab coral

      );
    }
  }
}
