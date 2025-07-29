// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.utils.RobotStatus;


/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class MoveElArmPostManualCmd extends Command {
  private ElevatorSub m_elevatorSub;
  private ArmSub m_armSub;

  /** Creates a new MoveElArm. */
  public MoveElArmPostManualCmd(ArmSub armSub, ElevatorSub elevatorSub) {
    m_armSub = armSub;
    m_elevatorSub = elevatorSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub, elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSub.enableAutomation();
    m_elevatorSub.setTargetHeight(RobotStatus.getLastPositionHeight());
    m_armSub.enableAutomation();
    m_armSub.setTargetAngle(RobotStatus.getLastPositionAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_elevatorSub.isAtTargetHeight() && m_armSub.isAtTargetAngle());
  }
}
