// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import java.util.ArrayList;
import java.util.Map;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.commands.tests.RunTestsGrp;

/**
 * This class holds all of the test results and handles adding them to the
 * Shuffleboard.
 */
public class TestManager {
  /**
   * An enumeration type that represents fail/warn/pass internally and associated
   * integers for
   * use with the dashboard.
   */
  public enum Result {
    kFail(-1), kWarn(0), kPass(1);

    private int value;

    Result(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }

  /**
   * A simple class to represent the location of dashboard widgets
   */
  private class Coordinates {
    public int m_x = 0;
    public int m_y = 0;

    public Coordinates(int x, int y) {
      m_x = x;
      m_y = y;
    }
  }

  /**
   * A class to store the current status of individual tests as well as
   * information about their
   * dashboard widgets.
   */
  private class TestStatus {
    public Result m_result;
    public String m_text;
    public GenericEntry m_resultDisplay;
    public GenericEntry m_textDisplay;
  }

  // Member variables
  private final ShuffleboardTab m_boardTab;
  private GenericEntry m_overallStatusDisplay;
  private ArrayList<TestStatus> m_testStatuses;
  private int m_nextTestStatusIdx = 0;
  private Coordinates m_nextTestCoordinates;

  /**
   * Constructor. Need to also call setTestCommand before this class will be
   * usable.
   * 
   * @param testsCommand Command that runs all of the tests
   */
  public TestManager() {
    // Grab the Tests tab on the Shuffleboard. It should already have been created.
    m_boardTab = Shuffleboard.getTab(Constants.Tests.kTabName);

    // Setup the location where test widgets can be added (below the overall
    // indicators that are
    // set up later)
    m_testStatuses = new ArrayList<TestStatus>();
    m_nextTestCoordinates = new Coordinates(1, 0);
  }

  /**
   * Sets the test command. This needs to be done after the constructor because
   * the test command
   * takes this class as a parameter so it can request new tests.
   * 
   * @param testsCommand Command to use to start all tests
   */
  public void setTestCommand(RunTestsGrp testsCommand) {
    // Add the overall-tests-result indicator and the button to start the tests
    m_overallStatusDisplay = m_boardTab.add("Overall Status", Result.kFail.getValue())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -1, "max", 1, "center", -1, "show text", false, "num tick marks", 2))
        .withSize(1, 1)
        .withPosition(0, 0)
        .getEntry();
    m_boardTab.add(testsCommand)
        .withSize(1, 1)
        .withPosition(1, 0);
  }

  /**
   * Creates a new test entry on the dashboard. Includes test result and status
   * text.
   * 
   * @param name Descriptive name for the test
   * @return Test ID. Used to update the test information in the future
   */
  public int registerNewTest(String name) {
    // Add a new test entry at the next open index. Use the index as the test ID.
    m_testStatuses.add(m_nextTestStatusIdx, new TestStatus());

    // Now add the test result and status test widgets to the dashboard
    Coordinates testLocation = getNextPosition();

    TestStatus newTest = m_testStatuses.get(m_nextTestStatusIdx);
    newTest.m_result = Result.kFail;
    newTest.m_resultDisplay = m_boardTab.add(name + " Result", newTest.m_result.getValue())
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -1, "max", 1, "center", -1, "show text", false, "num tick marks", 2))
        .withSize(1, 1)
        .withPosition(testLocation.m_y, testLocation.m_x)
        .getEntry();
    newTest.m_text = "Not run";
    newTest.m_textDisplay = m_boardTab.add(name + " Status", newTest.m_text)
        .withSize(1, 1)
        .withPosition(testLocation.m_y + 1, testLocation.m_x)
        .getEntry();

    // Return the array index (i.e. the ID) of the test and then increment it for
    // the next one
    return m_nextTestStatusIdx++;
  }

  /**
   * Reset all of the test statuses to "fail" to start a new test run
   */
  public void resetTestStatuses() {
    // Reset all individual tests
    for(int id = 0; id < m_testStatuses.size(); id++) {
      updateTestStatus(id, Result.kFail, "Not Run");
    }

    // Reset overall status
    updateOverallStatus();
  }

  /**
   * Method that triggers the update of the overall status on the dashboard
   */
  public void updateOverallStatus() {
    Result overallResult = Result.kPass; // Assume a pass unless we find test that didn't

    for(TestStatus testStatus : m_testStatuses) {
      if(testStatus.m_result == Result.kFail) {
        overallResult = Result.kFail;
        break;
      } else if(testStatus.m_result == Result.kWarn) {
        overallResult = Result.kWarn;
      }
    }

    // Update the overall result on the Shuffleboard
    m_overallStatusDisplay.setInteger(overallResult.getValue());
  }

  /**
   * Update the result and text for a single test
   * 
   * @param id ID of the test as returned from registerNewTest()
   * @param result Current result of the test
   * @param status Status text describing the result
   */
  public void updateTestStatus(int id, Result result, String status) {
    TestStatus currentTest = m_testStatuses.get(id);

    currentTest.m_result = result;
    currentTest.m_text = status;
    currentTest.m_resultDisplay.setInteger(result.getValue());
    currentTest.m_textDisplay.setString(status);
  }

  /**
   * Utility function that checks if the values provided should yield a pass, warn
   * or fail
   * 
   * @param actualValue The value produced by the test
   * @param targetValue The value that the test should have produced
   * @param tolerance The tolerance + or - around the targetValue that is still
   *        considered a pass
   * @param minimumValue The minimum value that the value can be to produce a warn
   * @return Pass, warn or fail determination
   */
  public Result determineResult(double actualValue, double targetValue, double tolerance, double minimumValue) {
    Result calculatedResult = Result.kFail; // Assume a fail unless proven otherwise

    if(Math.abs(actualValue - targetValue) < tolerance) {
      calculatedResult = Result.kPass;
    } else if(actualValue > minimumValue) {
      calculatedResult = Result.kWarn;
    }

    return calculatedResult;
  }

  /**
   * Get the next free location
   * 
   * @return Coordinates of the next free dashboard location
   */
  private Coordinates getNextPosition() {
    Coordinates nextPosition = new Coordinates(m_nextTestCoordinates.m_x, m_nextTestCoordinates.m_y);

    // Move to the next position after this one.
    m_nextTestCoordinates.m_x++;

    // Make sure we aren't falling off the bottom of the dashboard
    if(m_nextTestCoordinates.m_x > Constants.Tests.kDashboardRows) {
      m_nextTestCoordinates.m_x = 0;
      m_nextTestCoordinates.m_y += 2;
    }

    // If we anticipate having many entries, make sure we're not falling off the right side of the dashboard. If so, create a new tab

    return nextPosition;
  }
}
