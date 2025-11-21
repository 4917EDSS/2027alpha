// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSub;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class ElevatorMoveWithJoystickCmd extends Command {
  private final CommandXboxController m_controller;
  private final ElevatorSub m_elevatorSub;
  private boolean m_wasInDeadZone = true;

  /** Creates a new ElevatorWithJoystickCmd. */
  public ElevatorMoveWithJoystickCmd(CommandXboxController controller, ElevatorSub elevatorSub) {
    m_controller = controller;
    m_elevatorSub = elevatorSub;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get controller joystick value
    double elevatorPower = -m_controller.getLeftY();

    if((Math.abs(elevatorPower) > 0.05) && m_wasInDeadZone) {
      // When we leave the deadzone, disable automation
      m_elevatorSub.disableAutomation();
      m_wasInDeadZone = false;
    } else if((Math.abs(elevatorPower) < 0.05) && !m_wasInDeadZone) {
      // When we enter the deadzone, enable automation
      m_elevatorSub.setTargetHeight(m_elevatorSub.getPositionMm());
      m_elevatorSub.enableAutomation();
      m_wasInDeadZone = true;
    }

    // Only set the motor if the joystick is currently outside the deadzone
    if(!m_wasInDeadZone) {
      double direction = (elevatorPower >= 0.0) ? 1.0 : -1.0;
      m_elevatorSub.setPower(elevatorPower * elevatorPower * direction);
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
