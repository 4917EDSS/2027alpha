// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class creates a number of generic methods which can be used to test subsystem motors without
 * knowing any motor specifics. Motors are assumed to have built-in encoders to verify that they
 * actually moved during the test.
 * 
 * All of these methods need to be redefined in the actual subsystems and prefixed with @Override
 * to completely override the default behaviour written below (e.g. don't return -99999999.0). Just
 * copy the code below (excluding the constructor) and add @Override above each "public". Then put
 * real code in to do what the method says it does.
 * 
 */
public class TestableSubsystem extends SubsystemBase {
  /** Constructor that creates a new MotorSub. */
  public TestableSubsystem() {}

  //////////////////// Methods used for automated testing ////////////////////
  /**
   * Makes the motor ready for testing. This includes disabling any automation that uses this
   * motor
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  public void testEnableMotorTestMode(int motorId) {}

  /**
   * Puts the motor back into normal opeation mode.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  public void testDisableMotorTestMode(int motorId) {}

  /**
   * Resets the motor's encoder such that it reads zero
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  public void testResetMotorPosition(int motorId) {}

  /**
   * Sets the motor's power to the specified value. This needs to also disable anything else from
   * changing the motor power.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   * @param power Desired power -1.0 to 1.0
   */
  public void testSetMotorPower(int motorId, double power) {}

  /**
   * Returns the motor's current encoder value. Ideally this is the raw value, not the converted
   * value. This should be the INTERNAL encoder to minimize dependencies on other hardware.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   * @return Encoder value in raw or converted units
   */
  public double testGetMotorPosition(int motorId) {
    return -99999999.0;
  }

  /**
   * Returns the motor's current current-draw.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   * @return Electrical current draw in amps, or -1 if feature not supported
   */
  public double testGetMotorAmps(int motorId) {
    return -1.0;
  }
}
