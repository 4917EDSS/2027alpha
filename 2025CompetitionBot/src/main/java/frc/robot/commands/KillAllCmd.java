// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.logging.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.CanSub;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.subsystems.ElevatorSub;


public class KillAllCmd extends Command {
  private static Logger m_logger = Logger.getLogger(KillAllCmd.class.getName());
  ElevatorSub m_elevatorSub;
  ArmSub m_armSub;

  public KillAllCmd(ArmSub armSub, CanSub canSub, ClimbSub climbSub, DrivetrainSub drivetrainSub,
      ElevatorSub elevatorSub) {
    m_elevatorSub = elevatorSub;
    m_armSub = armSub;

    addRequirements(armSub, canSub, climbSub, drivetrainSub, elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_logger.fine("KillAllCmd - Init");
    //Hold arm and elevator at current positions
    m_elevatorSub.setTargetHeight(m_elevatorSub.getPositionMm());
    m_armSub.setTargetAngle(m_armSub.getAngle());
    m_elevatorSub.setIntakeMotors(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_logger.fine("KillAllCmd - End" + (interrupted ? " (interrupted)" : ""));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
