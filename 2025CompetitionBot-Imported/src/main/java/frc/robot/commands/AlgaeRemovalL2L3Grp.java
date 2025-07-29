// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.CanSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.VisionSub;


// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeRemovalL2L3Grp extends SequentialCommandGroup {
  /** Creates a new AlgaeRemovalL2L3Grp. */
  public AlgaeRemovalL2L3Grp(ArmSub armSub, CanSub canSub, DrivetrainSub drivetrainSub,
      ElevatorSub elevatorSub,
      VisionSub visionSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new AlgaePrepL2L3Grp(armSub, elevatorSub), // Move elevator and arm to algae removal location
        new WaitCommand(1.0),
        new AutoDriveCmd(visionSub, drivetrainSub, false), // Drive to vision target
        new MoveElArmGrp(Constants.Elevator.kL2L3AlgaeRemovalPostHeight, Constants.Arm.kL2L3AlgaeRemovalPostAngle,
            armSub, elevatorSub), // Remove algae
        new BackUpAfterScoringCmd(drivetrainSub), // Backup
        new ScheduleCommand(new GrabCoralTeleopGrp(armSub, canSub, elevatorSub)));
  }
}
