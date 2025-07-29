// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.CanSub;
import frc.robot.subsystems.ElevatorSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabCoralAutoGrp extends SequentialCommandGroup {
  /** Creates a new GrabCoralGrp. */
  public GrabCoralAutoGrp(ArmSub armSub, CanSub canSub, ElevatorSub elevatorSub) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new MoveElArmGrp(Constants.Elevator.kCoralGrabbableHeight, Constants.Arm.kCoralGrabbableAngle, armSub,
            elevatorSub), //Get ready to grab coral
        new InstantCommand(() -> elevatorSub.setIntakeMotors(Constants.Intake.kAutoIntakePower)),
        new WaitForLowerCoralCmd(canSub),
        new MoveElArmGrp(Constants.Elevator.kCoralLoadedHeight, Constants.Arm.kMinArmAngle, armSub, elevatorSub),
        new InstantCommand(() -> elevatorSub.setIntakeMotors(0.0)));
  }
}
