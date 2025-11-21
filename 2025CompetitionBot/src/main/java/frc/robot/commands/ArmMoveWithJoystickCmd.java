// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ArmMoveWithJoystickCmd extends Command {

  private final ArmSub m_armSub;
  private final CommandXboxController m_controller;
  private boolean m_wasInDeadZone = true;

  /** Creates a new MoveArmWithJoystickCmd. */
  public ArmMoveWithJoystickCmd(CommandXboxController controller, ArmSub armSub) {
    m_controller = controller;
    m_armSub = armSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pivotPower = -m_controller.getRightY();

    if((Math.abs(pivotPower) > 0.05) && m_wasInDeadZone) {
      m_armSub.disableAutomation();
      m_wasInDeadZone = false;
    }
    //else if we went into deadband, enable automation
    else if((Math.abs(pivotPower) < 0.05) && !m_wasInDeadZone) {
      m_armSub.setTargetAngle(m_armSub.getAngle());
      m_armSub.setHolding();
    }

    if(!m_wasInDeadZone) {
      double direction = (pivotPower >= 0.0) ? 1.0 : -1.0;
      m_armSub.setPower(pivotPower * pivotPower * direction);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
