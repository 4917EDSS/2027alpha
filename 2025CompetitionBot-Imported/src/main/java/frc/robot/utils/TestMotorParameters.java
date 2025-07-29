// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** This class holds all of the motor test's run and pass/fail parameters */
public class TestMotorParameters {
  public final String kTestName;
  public final int kMotorId;
  public final double kPower;
  public final double kTimeMs;

  public final double kPositionMin;
  public final double kPositionTarget;
  public final double kPositionTolerance;

  public final double kAmpsMin;
  public final double kAmpsTarget;
  public final double kAmpsTolerance;

  /**
   * Initializes the read-only test parameters
   * 
   * @param testName Short test title (<12 characters, ideally)
   * @param motorId ID of motor to access in the subsystem (starting at 1)
   * @param power Power to run motor at, -1.0 to 1.0
   * @param timeMs Time test should run for before sampling the results
   * @param positionMin Minimum position that motor should pass in order to not be a complete fail
   * @param positionTarget Position that the motor is expected to get to +/- the tolerance
   * @param positionTolerance Tolerance around the position target that's considered a pass
   * @param ampsMin Minimum current (in amps) that motor should draw in order to not be a complete fail
   * @param ampsTarget Current that the motor is expected to draw +/- the tolerance
   * @param ampsTolerance Tolerance around the current target that's considered a pass
   */
  public TestMotorParameters(String testName, int motorId, double power, double timeMs, double positionMin,
      double positionTarget, double positionTolerance, double ampsMin, double ampsTarget, double ampsTolerance) {
    kTestName = testName;
    kMotorId = motorId;
    kPower = power;
    kTimeMs = timeMs;

    kPositionMin = positionMin;
    kPositionTarget = positionTarget;
    kPositionTolerance = positionTolerance;

    kAmpsMin = ampsMin;
    kAmpsTarget = ampsTarget;
    kAmpsTolerance = ampsTolerance;
  }
}
