// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class SetArmToPositionCmd extends Command {
  private final double m_targetAngle;
  private final ArmSub m_armSub;

  /** Creates a new MoveArmWithJoystickCmd. */
  public SetArmToPositionCmd(double angle, ArmSub armSub) {
    m_targetAngle = angle;
    m_armSub = armSub;

    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSub.enableAutomation();
    m_armSub.setTargetAngle(m_targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSub.setHolding();
  }

  // Returns true when the command should end (arm is at correct position)
  @Override
  public boolean isFinished() {
    return m_armSub.isAtTargetAngle();
  }
}
