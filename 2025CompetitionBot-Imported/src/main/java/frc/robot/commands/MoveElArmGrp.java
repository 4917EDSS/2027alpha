// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ElevatorSub;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveElArmGrp extends ParallelCommandGroup {
  public MoveElArmGrp(double height, double angle, ArmSub armSub, ElevatorSub elevatorSub) {
    this(height, angle, armSub, elevatorSub, false);
  }

  /** Creates a new MoveElArmGrp. */

  public MoveElArmGrp(double height, double angle, ArmSub armSub, ElevatorSub elevatorSub, boolean isSlow) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(

        new SetElevatorToHeightCmd(height, elevatorSub, isSlow),
        new InstantCommand(() -> SmartDashboard.putBoolean("Step 1", true)),
        new SetArmToPositionCmd(angle, armSub));

  }
}
