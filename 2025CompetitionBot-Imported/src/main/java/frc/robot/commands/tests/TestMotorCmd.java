// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import java.time.Duration;
import java.time.Instant;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.TestManager;
import frc.robot.utils.TestMotorParameters;
import frc.robot.utils.TestableSubsystem;
import frc.robot.utils.TestManager.Result;

/*
 * You should consider using the more terse Command factories API instead
 * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
 */
public class TestMotorCmd extends Command {
  private final TestMotorParameters m_parameters;
  private final TestableSubsystem m_subsystem;
  private final TestManager m_testManager;
  private final int m_testId;

  private Instant m_startTime;


  /** Creates a new TestMotorCmd. */
  public TestMotorCmd(TestMotorParameters parameters, TestableSubsystem subsystem, TestManager testManager) {
    m_parameters = parameters;
    m_subsystem = subsystem;
    m_testManager = testManager;
    m_testId = testManager.registerNewTest(m_parameters.kTestName);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.testEnableMotorTestMode(m_parameters.kMotorId);
    m_subsystem.testResetMotorPosition(m_parameters.kMotorId);

    m_startTime = Instant.now();
    m_subsystem.testSetMotorPower(m_parameters.kMotorId, m_parameters.kPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      // Test was interrupted.  Stop the motor and fail the test
      m_subsystem.testSetMotorPower(m_parameters.kMotorId, 0.0);
      m_testManager.updateTestStatus(m_testId, Result.kFail, "Test interrupted");
      return;
    }

    double currentAmps = m_subsystem.testGetMotorAmps(m_parameters.kMotorId);
    double currentPosition = m_subsystem.testGetMotorPosition(m_parameters.kMotorId);

    // Test done.  Stop the motor
    m_subsystem.testSetMotorPower(m_parameters.kMotorId, 0.0);

    // Check to see if the measured position is good, ok or bad
    TestManager.Result positionResult =
        m_testManager.determineResult(currentPosition, m_parameters.kPositionTarget,
            m_parameters.kPositionTolerance, m_parameters.kPositionMin);
    String positionText = "Position=" + currentPosition + " (Target=" + m_parameters.kPositionTarget + "+/-"
        + m_parameters.kPositionTolerance + ")";
    System.out.println(m_parameters.kTestName + " " + positionText);

    // Check to see if the measured current is good, ok or bad
    TestManager.Result ampsResult = m_testManager.determineResult(currentAmps, m_parameters.kAmpsTarget,
        m_parameters.kAmpsTolerance, m_parameters.kAmpsMin);
    String ampsText = "Amps=" + currentAmps + " (Target=" + m_parameters.kAmpsTarget + "+/-"
        + m_parameters.kAmpsTolerance + ")";
    System.out.println(m_parameters.kTestName + " " + ampsText);


    // Figure out the overall test result
    TestManager.Result testResult = TestManager.Result.kPass;
    if((ampsResult == TestManager.Result.kFail) || (positionResult == TestManager.Result.kFail)) {
      testResult = TestManager.Result.kFail;
    } else if((ampsResult == TestManager.Result.kWarn) || (positionResult == TestManager.Result.kWarn)) {
      testResult = TestManager.Result.kWarn;
    }

    // Update the test results
    m_testManager.updateTestStatus(m_testId, testResult, positionText + " " + ampsText);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Check to see if the test time has elapsed
    if(Duration.between(m_startTime, Instant.now()).toMillis() >= m_parameters.kTimeMs) {
      return true;
    }
    return false;
  }
}
