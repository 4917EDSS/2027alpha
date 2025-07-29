// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.logging.Level;
import edu.wpi.first.units.measure.Angle;
import frc.robot.utils.TestMotorParameters;
import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Max log level to print (SEVERE, WARNING, INFO, CONFIG, FINE, FINER, or FINEST)
  // e.g. Level.WARNING will only print WARNING and SEVERE log messages
  public static final Level kLogLevel = Level.FINE;

  ////////// Hardware mapping /////////////////////////////////////////////////////////////////////
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class CanIds {
    // These are the roboRIO CAN bus IDs
    // CTRE Swerve drivetrain uses CAN IDs 1-13 on CANivore bus
    // This does not conflict with the roboRIO bus which can also use these IDs
    public static final int kElevatorMotor = 1;
    public static final int kElevatorMotor2 = 2;
    public static final int kArmMotor = 3;
    public static final int kClimbMotor = 4;
    public static final int kIntakeMotor = 5;

    // We can have multiple custom 4917 Aruino boards connected via CAN.  List their IDs here.
    public static final int kElevatorCustomCanBoard = 4;
  }

  public static class DioIds {
    public static final int kElevatorUpperLimit = 1;
    public static final int kElevatorEncoderResetSwitch = 2;
    public static final int kClimbInLimitSwitch = 3;
    public static final int kClimbOutLimitSwitch = 4;
    public static final int kClimbLatchTopSwitch = 5;
    public static final int kClimbLatchBottomSwitch = 6;
  }

  public static final class PwmIds {
    public static final int kLedStripPwmPort = 0;
  }


  ////////// Subsystem constants /////////////////////////////////////////////////////////////////////
  public static final class Arm {
    // All angles in degrees
    public static final double kMinArmAngle = -90.0;
    public static final double kMaxArmAngle = 33.0;
    public static final double kMaxPower = 0.5;

    public static final double kEncoderPositionConversionFactor = 0.01; // 1 / 100 * 360; // From rotations to degrees (Gear Ration / 360 deg)
    public static final double kEncoderVelocityConversionFactor = 1.00; // From rotations per minute? to degrees per second
    public static final double kAbsoluteEncoderOffset = 0.0; //practice bot 0.65 // From range to 0 - 1

    public static final double kAngleTolerance = 1.5; // In degrees
    public static final double kAtTargetMaxVelocity = 0.02; // In degrees per second?

    public static final double kDangerZoneBottomVertical = -88.0; // In degrees
    public static final double kDangerZoneMidVertical = -74.0;
    public static final double kDangerZoneUpperAngle = -60.0; // In degrees
    public static final double kDangerZoneBraceAngle = -84.0; // In degrees

    public static final double kSlowDownLowerAngle = kMinArmAngle + 5; // In degrees
    public static final double kSlowDownUpperAngle = kMaxArmAngle - 5; // In degrees
    public static final double kSlowDownSpeed = 0.15; // 1 = full power

    public static final double kL2PreScoreAngle = 33;
    public static final double kL2PostScoreAngle = 0.0;
    public static final double kL3PreScoreAngle = 33;
    public static final double kL3PostScoreAngle = 0.0;
    public static final double kL4PreScoreAngle = 33;
    public static final double kL4PostScoreAngle = -2.0;
    public static final double kCoralGrabbableAngle = -90.0;

    public static final double kL3L4AlgaeRemovalPrepAngle = 13;
    public static final double kL2L3AlgaeRemovalPrepAngle = 8;
    public static final double kL2L3AlgaeRemovalPostAngle = 10;
    public static final double kL3L4AlgaeRemovalPostAngle = kMaxArmAngle;
    public static final double kArmConversionFactor = 3.6;
  }

  public static final class Climb {
    public static final double kClimbMotorPower = 1;
  }

  public static final class Elevator {
    // All heights are in millimeters
    public static final double kMinHeight = 300.0;
    public static final double kMaxHeight = 1670.0;

    public static final double kRotationsToMm = 5 / (0.75 * 25.4 * Math.PI); // Gearing / Spool diameter in inches * mm/in * PI

    public static final double kHeightTolerance = 5; // Height tolerance for elevator position
    public static final double kAtTargetMaxVelocity = 150; // Max velocity at elevator position

    // Sets a max power if we are close to the height limits
    public static final double kSlowDownLowerStagePower = -0.15;
    public static final double kSlowDownLowerStageHeight = kMinHeight + 10;
    public static final double kSlowDownUpperStagePower = 0.15;
    public static final double kSlowDownUpperStageHeight = kMaxHeight - 50;

    public static final double kDangerZoneBottom = 817;
    public static final double kDangerZoneBraceBottom = 1203;
    public static final double kDangerZoneBraceTop = 1405;

    public static final double kResetHeight = 728; // Height where elevator encounters the encoder reset switch 
    public static final double kCoralLoadedHeight = 690; // This should be some height above the bottom danger zone so arm can swing up
    public static final double kStartingHeight = kCoralLoadedHeight; // Height where elevator starts with coral pre-loaded
    public static final double kCoralGrabbableHeight = 901.0; // Height that coral can still slide in under the arm for the coral to be grabbable
    public static final double kL2PreScoreHeight = 723.0;
    public static final double kL2PostScoreHeight = 563.0;
    public static final double kL3PreScoreHeight = 1107.0;
    public static final double kL3PostScoreHeight = 980.0;
    public static final double kL4PreScoreHeight = kMaxHeight - 5;
    public static final double kL4PostScoreHeight = 1500;
    public static final double kL3L4AlgaeRemovalPrepHeight = 842;
    public static final double kL2L3AlgaeRemovalPrepHeight = 560;
    public static final double kL2L3AlgaeRemovalPostHeight = kL2L3AlgaeRemovalPrepHeight + 20;
    public static final double kL3L4AlgaeRemovalPostHeight = kL3L4AlgaeRemovalPrepHeight + 20;
    public static final double kL2PerpareScoreHeight = 820;
    // public static final double kScoreDropAngle = 30;
    // public static final double kElevatorDropScore = -30;
  }

  // Values that are specific to a particular physical robot
  public static final class RobotSpecific {
    public static final String PracticeSerialNumber = "03147322";
    public static final String CompetitionSerialNumber = "03264244";
    public static final String serialNumber = System.getenv("serialnum");

    public static final class Practice {
      public static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.318115234375);
      public static final Angle kFrontRightEncoderOffset = Rotations.of(0.023193359375);
      public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.064453125);
      public static final Angle kBackRightEncoderOffset = Rotations.of(0.025146484375);
      public static final double kGearRatio = 6.12; //from last year
      public static final double kDriveGearRatio = 5.142857142857142;
      public static final double kSteerGearRatio = 12.8;
      public static final boolean kInvertLowerFeeder = false;
    }

    public static final class Competition {
      public static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.37744140625);
      public static final Angle kFrontRightEncoderOffset = Rotations.of(-0.03759765625);
      public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.148193359375);
      public static final Angle kBackRightEncoderOffset = Rotations.of(-0.4365234375);
      public static final boolean kInvertLowerFeeder = true;
      public static final double kGearRatio = 6.05; //from last year
      public static final double kDriveGearRatio = 5.142857142857142;
      public static final double kSteerGearRatio = 12.8;
    }

    public static final class Unknown {
      public static final Angle kAbsoluteEncoderOffsetFL = Rotations.of(0);
      public static final Angle kAbsoluteEncoderOffsetFR = Rotations.of(0);
      public static final Angle kAbsoluteEncoderOffsetBL = Rotations.of(0);
      public static final Angle kAbsoluteEncoderOffsetBR = Rotations.of(0);
      public static final boolean kInvertLowerFeeder = true;
      public static final double kGearRatio = 0.0;
    }
  }

  public static final class Vision {
    //TODO: change apriltag heights to actual heights, as well as the offsets. This is from last year.
    public static final double kApriltagOffset = 0.0825; // Apriltag height + bot height (Will need to be changed in the future)
    public static final double kApriltagHeights[] =
        {1.22, 1.22, 1.32, 1.32, 1.22, 1.22, 1.32, 1.32, 1.22, 1.22, 1.24, 1.24, 1.24, 1.24, 1.24, 1.24};

    public static final double kDistanceToCloseToDrive = -1.25;
  }

  public static final class Intake {
    public static final double kIntakePower = 0.7;
    public static final double kAutoIntakePower = 0.8;
  }

  ////////// Test pass/fail/warn parameters ///////////////////////////////////////////////////////
  public static final class Tests {
    public static final String kTabName = "Tests";
    public static final int kDashboardRows = 5; // Max rows that we can use to display tests (start new column after this row)
    public static final int kDashboardCols = 6; // Max columns that we can use to display tests (start a new tab after this column)

    /*
     * TestMotorParameters are, in order:
     * 
     * testName - Short test title (<12 characters, ideally)
     * motorId - ID of motor to access in the subsystem (starting at 1)
     * power - Power to run motor at, -1.0 to 1.0
     * timeMs - Time test should run for before sampling the results
     * positionMin - Minimum position that motor should pass in order to not be a complete fail
     * positionTarget - Position that the motor is expected to get to +/- the tolerance
     * positionTolerance - Tolerance around the position target that's considered a pass
     * ampsMin - Minimum current (in amps) that motor should draw in order to not be a complete fail
     * ampsTarget - Current that the motor is expected to draw +/- the tolerance
     * ampsTolerance - Tolerance around the current target that's considered a pass
     */

    /* Elevator Motors */
    public static final TestMotorParameters kElevatorMotor1 =
        new TestMotorParameters("ElevatorMotor1", 1, 0.1, 1000, 75, 74, 10, 1.5, 20, 5);
    public static final TestMotorParameters kElevatorMotor2 =
        new TestMotorParameters("ElevatorMotor2", 2, 0.1, 1000, 75, 74, 10, 1.5, 20, 5);
    /* Intake Motors */
    public static final TestMotorParameters kIntakeMotor1 =
        new TestMotorParameters("IntakeMotor1", 1, 0.1, 1000, 75, 3.0, 10, 1.5, 0.1, 0.5);
    public static final TestMotorParameters kIntakeMotor2 =
        new TestMotorParameters("IntakeMotor2", 1, 0.1, 1000, 75, 3.0, 10, 1.5, 0.1, 0.5);
    /* Arm Motor */
    public static final TestMotorParameters kArmMotor =
        new TestMotorParameters("ArmMotor", 1, 0.1, 1000, 75, 3.0, 10, 1.5, 0.1, 0.5);
    /* Climb Motor */
    public static final TestMotorParameters kClimbMotor =
        new TestMotorParameters("ClimbMotor", 1, 0.1, 1000, 75, 3.0, 10, 1.5, 0.1, 0.5);
  }
}
