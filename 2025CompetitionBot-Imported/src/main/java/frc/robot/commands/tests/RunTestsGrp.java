// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSub;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.utils.TestManager;

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunTestsGrp extends SequentialCommandGroup {

  /** Creates a new RunTestsGrp. */
  public RunTestsGrp(ClimbSub climbSub, ArmSub armSub, ElevatorSub elevatorSub, TestManager testManager) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new InstantCommand(() -> testManager.resetTestStatuses()),
        // TestMotorCmd needs the test parameters, the subsystem we are testing and the testManager
        new TestMotorCmd(Constants.Tests.kElevatorMotor1, elevatorSub, testManager),
        new TestMotorCmd(Constants.Tests.kElevatorMotor2, elevatorSub, testManager),

        new TestMotorCmd(Constants.Tests.kArmMotor, armSub, testManager),

        // new TestMotorCmd(Constants.Tests.kClimbMotor, climbSub, testManager),
        new InstantCommand(() -> testManager.updateOverallStatus()));
  }
}
