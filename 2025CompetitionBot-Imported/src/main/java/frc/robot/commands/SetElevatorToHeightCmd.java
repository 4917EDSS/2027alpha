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
public class SetElevatorToHeightCmd extends Command {
  private double m_targetHeight;
  private ElevatorSub m_elevatorSub;
  private boolean m_isSlow;

  public SetElevatorToHeightCmd(double targetHeight, ElevatorSub elevatorSub) {
    this(targetHeight, elevatorSub, false);
  }

  /** Creates a new SetElevatorToHeightCmd. */
  public SetElevatorToHeightCmd(double targetHeight, ElevatorSub elevatorSub, boolean isSlow) {
    m_elevatorSub = elevatorSub;
    m_targetHeight = targetHeight;
    m_isSlow = isSlow;

    addRequirements(m_elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevatorSub.enableAutomation();
    m_elevatorSub.setTargetHeight(m_targetHeight);
    m_elevatorSub.setElevatorSlowMode((m_isSlow));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevatorSub.setElevatorSlowMode((false));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevatorSub.isAtTargetHeight();
  }
}
