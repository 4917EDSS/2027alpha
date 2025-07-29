// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CanSub;
import frc.robot.subsystems.DrivetrainSub;
import frc.robot.commands.WaitForUpperCoralCmd;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class BackUpToPickUpcmd extends Command {
  private final DrivetrainSub m_drivetrainSub;
  private final SwerveRequest.RobotCentric autoBackUpDrive = new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** Creates a new BackUpToPickUpcmd. */
  public BackUpToPickUpcmd(DrivetrainSub drivetrainSub, CanSub canSub) {
    m_drivetrainSub = drivetrainSub;
    addRequirements(drivetrainSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_drivetrainSub.setControl(
        autoBackUpDrive.withVelocityX(-0.25)
            .withVelocityY(0)
            .withRotationalRate(0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSub.setControl(
        autoBackUpDrive.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
