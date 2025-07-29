// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class SetElevatorToRelHeightCmd extends Command {
  private double m_relativeHeight;
  private ElevatorSub m_elevatorSub;

  /** Creates a new SetElevatorToRelHeightCmd. */
  public SetElevatorToRelHeightCmd(double relativeHeight, ElevatorSub elevatorSub) {
    m_elevatorSub = elevatorSub;
    m_relativeHeight = relativeHeight;

    addRequirements(m_elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSub.enableAutomation();
    m_elevatorSub.setTargetHeight(m_elevatorSub.getPositionMm() + m_relativeHeight);
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
    return m_elevatorSub.isAtTargetHeight();
  }
}
