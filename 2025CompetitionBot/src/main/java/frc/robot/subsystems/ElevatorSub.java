// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;
import java.util.logging.Logger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.SubControl;
import frc.robot.utils.SubControl.State;
import frc.robot.utils.TestableSubsystem;

public class ElevatorSub extends TestableSubsystem {
  private static Logger m_logger = Logger.getLogger(ElevatorSub.class.getName());

  private final TalonFX m_elevatorMotor = new TalonFX(Constants.CanIds.kElevatorMotor);
  private final TalonFX m_elevatorMotor2 = new TalonFX(Constants.CanIds.kElevatorMotor2);
  private final SparkMax m_intakeMotor = new SparkMax(Constants.CanIds.kIntakeMotor, MotorType.kBrushless);
  private final DigitalInput m_elevatorUpperLimit = new DigitalInput(Constants.DioIds.kElevatorUpperLimit);
  private final DigitalInput m_encoderResetSwitch = new DigitalInput(Constants.DioIds.kElevatorEncoderResetSwitch);

  private double m_kS = 0.005;//0.01
  private double m_kG = 0.02;//0.05
  private double m_kV = 0.0;
  private double m_kP = 0.01;
  private double m_kI = 0.0;
  private double m_kD = 0.0;
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(m_kS, m_kG, m_kV);
  private final PIDController m_elevatorPID = new PIDController(m_kP, m_kI, m_kD);
  private final PIDController m_negitiveelevatorPID = new PIDController(0.01, 0, 0);

  private Supplier<Double> m_armAngle;
  private double m_targetHeight = 0.0;
  private double m_blockedPosition;
  private boolean m_enableAutomation = false;
  private boolean m_isElevatorEncoderSet = false;
  private double m_preTestHeight = 0;
  private double m_smartDashboardCounter = 0;
  private SubControl m_currentControl = new SubControl(); // Current states of mechanism
  private boolean m_isSlow = false;

  /** Creates a new ElevatorSub. */
  public ElevatorSub() {
    TalonFXConfigurator talonFxConfiguarator = m_elevatorMotor.getConfigurator();
    TalonFXConfigurator talonFxConfiguarator2 = m_elevatorMotor2.getConfigurator();

    // This is how you set a current limit inside the motor (vs on the input power
    // supply)
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 80; // Limit in Amps
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFxConfiguarator.apply(limitConfigs);
    talonFxConfiguarator2.apply(limitConfigs);

    // This is how you can set a deadband, invert the motor rotation and set
    // brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFxConfiguarator.apply(outputConfigs);

    outputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    talonFxConfiguarator2.apply(outputConfigs);


    SparkMaxConfig motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(false) // Set to true to invert the forward motor direction
        .smartCurrentLimit(60) // Current limit in amps
        .idleMode(IdleMode.kBrake);

    AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
    encoderConfig.zeroOffset(0);
    motorConfig.apply(encoderConfig);

    m_intakeMotor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    m_elevatorMotor2.setControl(new Follower(m_elevatorMotor.getDeviceID(), false));

    // Set the encoder conversion factor
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = Constants.Elevator.kRotationsToMm;
    m_elevatorMotor.getConfigurator().apply(config);
    m_elevatorMotor2.getConfigurator().apply(config);

    init();
  }

  /**
   * Put the subsystem in a known state
   */
  public void init() {
    m_logger.info("Initializing ElevatorSub Subsystem");
    m_isElevatorEncoderSet = false;
    m_elevatorMotor.setPosition(Constants.Elevator.kStartingHeight);
    setPositionMm(Constants.Elevator.kStartingHeight);
  }

  /**
   * Give this subsystem access to the arm's current angle
   */
  public void setArmAngleSupplier(Supplier<Double> armAngle) {
    m_armAngle = armAngle;
  }

  @Override
  public void periodic() {
    updateStateMachine();
    runHeightControl(m_enableAutomation);


    // If we haven't set the relative encoder's position yet, check if we are at the
    // switch that tells us to do so
    if(!m_isElevatorEncoderSet) {
      // Adds a counter to the encoder reset switch so that we don't reset position by
      // accident
      if(encoderResetSwitchHit() && (m_elevatorMotor.get() > 0.0)) {
        m_isElevatorEncoderSet = true;
        setPositionMm(Constants.Elevator.kResetHeight);
      }

      // If we hit the reset switch twice in a row, reset encoder
      // if(m_hitEncoderSwitchCounter >= 2) {     //This variable was deleted, if this code is added back, re-add that as well

      //   m_isElevatorEncoderSet = true;
      // }
    }
    SmartDashboard.putNumber("El Height", getPositionMm()); // Elevator position
    SmartDashboard.putBoolean("El Upper Limit", isAtUpperLimit()); // True if we are at the upper limit
    SmartDashboard.putBoolean("El Calib Switch", encoderResetSwitchHit()); // True if we hit the encoder reset switch
    SmartDashboard.putBoolean("El Height Is Set", m_isElevatorEncoderSet); // True once the encoder is set
    SmartDashboard.putNumber("El Current 1", testGetMotorAmps(1));
    SmartDashboard.putNumber("El Current 2", testGetMotorAmps(2));
    SmartDashboard.putBoolean("Is at Elivator limit", isAtTargetHeight());
    SmartDashboard.putNumber("Elevator Target", m_targetHeight);
    // Current power value is sent in setPower()

    boolean tuning = false;
    if(tuning) {
      // Lighten the load by only updating these twice a second
      if(++m_smartDashboardCounter >= 30) {
        m_smartDashboardCounter = 0;
        double kP = SmartDashboard.getNumber("El kP", m_kP);
        double kI = SmartDashboard.getNumber("El kI", m_kI);
        double kD = SmartDashboard.getNumber("El kD", m_kD);

        double kS = SmartDashboard.getNumber("El kS", m_kS);
        double kG = SmartDashboard.getNumber("El kG", m_kG);
        double kV = SmartDashboard.getNumber("El kV", m_kV);

        SmartDashboard.putNumber("El kP", kP);
        SmartDashboard.putNumber("El kI", kI);
        SmartDashboard.putNumber("El kD", kD);

        SmartDashboard.putNumber("El kS", kS);
        SmartDashboard.putNumber("El kG", kG);
        SmartDashboard.putNumber("El kV", kV);
      }
    }
  }

  /**
   * Manually set the power of the elevator motor(s).
   * 
   * @param power power value -1.0 to 1.0
   */
  public void setPower(double power) {
    // If lower limit switch is hit and the motor is going down, stop.
    // If upper limit switch is hit and the motor is going up, hold that height
    // If we are too close to the lower limit, set max power to a low value
    // If we are too close to the upper limit, set max power to a low value
    // Otherwise, set power normally
    double powerValue = power;
    if(isAtLowerLimit() && power < 0.0) {
      powerValue = 0.0;
    } else if(isAtUpperLimit() && power > 0.02) {
      powerValue = 0.02;
    } else if((getPositionMm() < Constants.Elevator.kSlowDownLowerStageHeight)
        && (power < Constants.Elevator.kSlowDownLowerStagePower)) {
      powerValue = Constants.Elevator.kSlowDownLowerStagePower;
    } else if((getPositionMm() > Constants.Elevator.kSlowDownUpperStageHeight)
        && (power > Constants.Elevator.kSlowDownUpperStagePower)) {
      powerValue = Constants.Elevator.kSlowDownUpperStagePower;
    } else if((getPositionMm() >= Constants.Elevator.kMaxHeight) && (power > 0.0)) {
      powerValue = 0;
    }

    if(powerValue > 0.85) {
      powerValue = 0.85;
    } else if(powerValue < -0.5) {
      powerValue = -0.5;
    }
    m_elevatorMotor.set(powerValue);
    SmartDashboard.putNumber("El Power", powerValue);
  }

  /**
   * Sets the encoder to the specified height
   */
  public void setPositionMm(double height) {
    m_elevatorMotor.setPosition(height);
  }

  /**
   * Returns the current height of the elevator
   * 
   * @return position in mm
   */
  public double getPositionMm() {
    return m_elevatorMotor.getPosition().getValueAsDouble();
  }

  /**
   * Allows the elevator encoder to be reset next time it move up past the reset
   * switch
   */
  public void allowEncoderReset() {
    m_isElevatorEncoderSet = false;
  }

  /**
   * Returns the current velocity of the elevator
   * 
   * @return velocity in mm per second
   */
  public double getVelocity() {
    return m_elevatorMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Sets the target height that the elevator should move to
   * 
   * @param targetHeight target height in mm
   */
  public void setTargetHeight(double targetHeight) {
    if(targetHeight >= Constants.Elevator.kMaxHeight) {
      targetHeight = Constants.Elevator.kMaxHeight;
    } else if(targetHeight <= Constants.Elevator.kMinHeight) {
      targetHeight = Constants.Elevator.kMinHeight;
    }

    m_targetHeight = targetHeight;
    enableAutomation();
    runHeightControl(false);
  }

  /**
   * Set the power of the coral intake wheels motor in the chute
   * 
   * @param power power to set the motor to (-1.0 to 1.0)
   */
  public void setIntakeMotors(double power) {
    m_intakeMotor.set(power);
  }

  /**
   * Enables closed loop control of elevator height
   */
  public void enableAutomation() {
    m_enableAutomation = true;
  }

  /**
   * Disables closed loop control of elevator height
   */
  public void disableAutomation() {
    m_enableAutomation = false;
  }

  /**
   * Returns if the elevator is at its lower limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  public boolean isAtLowerLimit() {
    return false; // We don't have a lower limit switch
  }

  /**
   * Returns if the elevator is at its upper limit or not
   * 
   * @return true when it's at the limit, false otherwise
   */
  public boolean isAtUpperLimit() {
    return !m_elevatorUpperLimit.get();
  }

  /**
   * Returns if the elevator is at the encoder reset switch or not
   * 
   * @return true when it's at the switch, false otherwise
   */
  public boolean encoderResetSwitchHit() {
    return !m_encoderResetSwitch.get();
  }

  /**
   * Enable/disable the limit on elevator power
   * 
   * @param isSlow true to limit the power
   */
  public void setElevatorSlowMode(boolean isSlow) {
    m_isSlow = isSlow;

  }

  /**
   * Handle the case where the elevator needs to stop due to a potential collision
   */
  private void updateStateMachine() {

    // Determine what power the mechanism should use based on the current state
    switch(m_currentControl.state) {
      case MOVING:
        SmartDashboard.putBoolean("El Is Blocked", false);
        // If the mechanism is moving, check if it has arrived at it's target.
        if(isBlocked()) {
          m_blockedPosition = getPositionMm();
          m_currentControl.state = State.INTERRUPTED;
        }
        break;

      case INTERRUPTED:
        SmartDashboard.putBoolean("El Is Blocked", true);
        // If the mechanism is no longer blocked, transition to MOVING
        if(!isBlocked()) {
          m_currentControl.state = State.MOVING;
          // Otherwise, hold this position
        }
        break;

      default:
        m_currentControl.state = State.INTERRUPTED;
        break;
    }
  }

  /**
   * Run the logic to check if the elevator movement is interrupted by a potential
   * collision
   */
  private boolean isBlocked() {
    /*
     * should stop the elevator if:
     * - the elevator position is lower than the bottom danger zone and the
     * elevator is moving downwards, and also the arm is not close to pointing
     * downwards
     * 
     * - the elevator position is higher than the bottom brace, and if the arm is
     * holding a coral
     * AND facing close to downwards, and the elevator is moving downwards
     * 
     * - the elevator position is higher than the bottom brace but lower than the
     * top brace, and the arm is
     * holding a coral and is facing close to downwards, AND the elevator is moving
     * upwards
     */

    double currentHeight = getPositionMm();
    double armAngle = m_armAngle.get();

    // Going up or going down, check for brace dangerzones.
    if((currentHeight > Constants.Elevator.kDangerZoneBraceBottom)
        && (currentHeight < Constants.Elevator.kDangerZoneBraceTop)
        && (armAngle < Constants.Arm.kDangerZoneBraceAngle)) {
      return true;
    }

    if((m_targetHeight < currentHeight)) {
      // moving downwards
      if((currentHeight < Constants.Elevator.kDangerZoneBottom) && (armAngle > Constants.Arm.kDangerZoneBottomVertical)
          && (armAngle < Constants.Arm.kDangerZoneUpperAngle)) {
        return true;
      }
    }

    return false;
  }

  /**
   * Calculates and sets the current power to apply to the elevator to get to or
   * stay at its target
   * 
   * @param updatePower set to false to update the Feedforward and PID controllers
   *        without changing the motor power
   */
  private void runHeightControl(boolean updatePower) {
    double activeTarget = m_targetHeight;

    if(m_currentControl.state == State.INTERRUPTED) {
      activeTarget = m_blockedPosition;
    }

    double ffPower = m_feedforward.calculate(getVelocity());
    //CONSTAN FORCE SPRING COMING IN
    if(getPositionMm() >= 1100) {
      ffPower += 0.03;
    }
    double pidPower = (m_elevatorPID.calculate(getPositionMm(), activeTarget));
    double negitivepidPower = (m_negitiveelevatorPID.calculate(getPositionMm(), activeTarget));
    double totalPower = ffPower;

    if(activeTarget > getPositionMm()) {
      totalPower += pidPower;
    } else {
      totalPower += negitivepidPower;
    }

    if(updatePower) {
      if(m_isSlow) {
        if(totalPower > 0.2) {
          totalPower = 0.2;
        } else if(totalPower < -0.2) {
          totalPower = -0.2;
        }
      }
      setPower(totalPower);
    }
  }

  /**
   * Returns if the elevator has reached it's target height or not
   * 
   * @return true if at target height, false if not
   */
  public boolean isAtTargetHeight() {
    // If we are within tolerance and our velocity is low, we're at our target
    if((Math.abs(m_targetHeight - getPositionMm()) < Constants.Elevator.kHeightTolerance)
        && (getVelocity() < Constants.Elevator.kAtTargetMaxVelocity)) {
      return true;
    } else if(m_targetHeight >= 1600 && isAtUpperLimit()) {
      return true;
    } else {
      return false;
    }
  }

  //////////////////// Methods used for automated testing ////////////////////
  /**
   * Makes the motor ready for testing. This includes disabling any automation
   * that uses this
   * motor
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  @Override
  public void testEnableMotorTestMode(int motorId) {
    // Save the current encoder values because the tests will reset them
    switch(motorId) {
      case 1:
        m_preTestHeight = m_elevatorMotor.getPosition().getValueAsDouble();
        break;
      case 2:
        m_preTestHeight = m_elevatorMotor.getPosition().getValueAsDouble();
        break;
      default:
        // Do nothing
        break;
    }

    // Disable any mechanism automation (PID, etc.). Check periodic()
    disableAutomation();
  }

  /**
   * Puts the motor back into normal opeation mode.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  @Override
  public void testDisableMotorTestMode(int motorId) {
    // Set the encoders to their pre-test values plus the change in position from
    // the test
    switch(motorId) {
      case 1:
        m_elevatorMotor.setPosition(m_preTestHeight + m_elevatorMotor.getPosition().getValueAsDouble(), 0.5);
        setTargetHeight(getPositionMm());
        m_isElevatorEncoderSet = false;
        break;
      case 2:
        m_elevatorMotor2.setPosition(m_preTestHeight + m_elevatorMotor2.getPosition().getValueAsDouble(), 0.5);
        break;
      default:
        // Do nothing
        break;
    }

    // Re-enable any mechanism automation
    enableAutomation();
  }

  /**
   * Resets the motor's encoder such that it reads zero
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  @Override
  public void testResetMotorPosition(int motorId) {
    switch(motorId) {
      case 1:
        m_elevatorMotor.setPosition(0.0, 0.5); // Set it to 0 and wait up to half a second for it to take effect
        break;
      case 2:
        m_elevatorMotor2.setPosition(0.0, 0.5); // Set it to 0 and wait up to half a second for it to take effect
        break;
      default:
        // Do nothing
        break;
    }
  }

  /**
   * Sets the motor's power to the specified value. This needs to also disable
   * anything else from
   * changing the motor power.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   * @param power Desired power -1.0 to 1.0
   */
  @Override
  public void testSetMotorPower(int motorId, double power) {
    switch(motorId) {
      case 1:
        m_elevatorMotor.set(power);
        break;
      case 2:
        // Motor 2 is configured as a motor 1 follower so set power on motor 1 for this
        // test
        m_elevatorMotor.set(power);
        break;
      default:
        // Do nothing
        break;
    }
  }

  /**
   * Returns the motor's current encoder value. Ideally this is the raw value, not
   * the converted
   * value. This should be the INTERNAL encoder to minimize dependencies on other
   * hardware.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   * @return Encoder value in raw or converted units
   */
  @Override
  public double testGetMotorPosition(int motorId) {
    double position = 0.0;

    switch(motorId) {
      case 1:
        position = m_elevatorMotor.getPosition().getValueAsDouble(); // This position is affected by the conversion
                                                                     // factor
        break;
      case 2:
        position = m_elevatorMotor2.getPosition().getValueAsDouble(); // This position is affected by the conversion
                                                                      // factor
        break;
      default:
        // Return an invalid value
        position = -99999999.0;
        break;
    }

    return position;
  }

  /**
   * Returns the motor's current current-draw.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   * @return Electrical current draw in amps, or -1 if feature not supported
   */
  @Override
  public double testGetMotorAmps(int motorId) {
    double current = 0.0;

    switch(motorId) {
      case 1:
        current = m_elevatorMotor.getStatorCurrent().getValueAsDouble();
        break;
      case 2:
        current = m_elevatorMotor2.getStatorCurrent().getValueAsDouble();
        break;
      default:
        // Return an invalid value
        current = -1.0;
        break;
    }
    return current;
  }
}
