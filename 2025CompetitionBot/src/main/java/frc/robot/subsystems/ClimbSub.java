// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.utils.TestableSubsystem;

public class ClimbSub extends TestableSubsystem {
  private static Logger m_logger = Logger.getLogger(ClimbSub.class.getName());

  private final LedSub m_ledSub;

  private final TalonFX m_climbMotor = new TalonFX(Constants.CanIds.kClimbMotor);
  private final DigitalInput m_climbInLimit = new DigitalInput(Constants.DioIds.kClimbInLimitSwitch);
  private final DigitalInput m_climbOutLimit = new DigitalInput(Constants.DioIds.kClimbOutLimitSwitch);
  private final DigitalInput m_climbLatchTopLimit = new DigitalInput(Constants.DioIds.kClimbLatchTopSwitch);
  private final DigitalInput m_climbLatchBottomLimit = new DigitalInput(Constants.DioIds.kClimbLatchBottomSwitch);

  /** Creates a new ClimbSub. */
  public ClimbSub(LedSub ledSub) {
    m_ledSub = ledSub;

    TalonFXConfigurator talonFxConfiguarator = m_climbMotor.getConfigurator();

    // This is how you set a current limit inside the motor (vs on the input power supply)
    CurrentLimitsConfigs limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 60; // Limit in Amps  // TOOD: Determine reasonable limit
    limitConfigs.StatorCurrentLimitEnable = true;
    talonFxConfiguarator.apply(limitConfigs);

    // This is how you can set a deadband, invert the motor rotoation and set brake/coast
    MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
    outputConfigs.DutyCycleNeutralDeadband = 0.02; // Ignore values below 2%
    outputConfigs.Inverted = InvertedValue.Clockwise_Positive; // Invert = Clockwise
    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFxConfiguarator.apply(outputConfigs);


    init();
  }

  public void init() {
    m_logger.info("Initializing ClimbSub Subsystem");
    setPower(0.0);
    resetPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(isAtOutLimit() && (m_climbMotor.get() > 0)) {
      setPower(0);
    } else if((isAtInLimit() || getPosition() <= 0.0) && (m_climbMotor.get() < 0)) {
      setPower(0);
    }

    if(isTopLatched() && isBottomLatched()) {
      m_ledSub.setElevatorColor((byte) 127, (byte) 127, (byte) 127);
      m_ledSub.setClimbColor((byte) 127, (byte) 127, (byte) 127);
    } else if(isBottomLatched() && !isTopLatched()) {
      m_ledSub.setElevatorColor((byte) 127, (byte) 0, (byte) 0);
      m_ledSub.setClimbColor((byte) 127, (byte) 0, (byte) 0);
    } else if(isTopLatched() && !isBottomLatched()) {
      m_ledSub.setElevatorColor((byte) 0, (byte) 0, (byte) 127);
      m_ledSub.setClimbColor((byte) 0, (byte) 0, (byte) 127);
    }

    // Add motor and limit switche(s) to Smartdashboard
    SmartDashboard.putNumber("Cl Angle", getPosition());
    SmartDashboard.putBoolean("Cl In Limit", isAtInLimit());
    SmartDashboard.putBoolean("Cl Out Limit", isAtOutLimit());
    SmartDashboard.putBoolean("Cl Top Latch", isTopLatched());
    SmartDashboard.putBoolean("Cl Bot Latch", isBottomLatched());
    SmartDashboard.putNumber("Cl Current", getElectricalCurrent());

  }

  public boolean isAtInLimit() {
    return !m_climbInLimit.get();
  }

  public boolean isAtOutLimit() {
    return !m_climbOutLimit.get();
  }

  public boolean isTopLatched() {
    return !m_climbLatchTopLimit.get();
  }

  public boolean isBottomLatched() {
    return !m_climbLatchBottomLimit.get();
  }


  /**
   * Manually set the power of the climb motor(s).
   * 
   * @param power power value -1.0 to 1.0
   */
  public void setPower(double power) {
    m_climbMotor.set(power);
    SmartDashboard.putNumber("Cl Power", power);
  }

  /**
   * Sets the current angle as the zero angle
   */
  public void resetPosition() {
    m_climbMotor.setPosition(0);
  }

  /**
   * Returns the current angular position of the climb arm
   * 
   * @return position in degrees
   */
  public double getPosition() {
    return m_climbMotor.getPosition().getValueAsDouble();
  }

  /**
   * Returns the current angular velocity of the climb arm
   * 
   * @return velocity in degrees per second
   */
  public double getVelocity() {
    return m_climbMotor.getRotorVelocity().getValueAsDouble();
  }

  /**
   * Returns how much current the motor is currently drawing
   * 
   * @return current in amps or -1.0 if motor can't measure current
   */
  public double getElectricalCurrent() {
    return m_climbMotor.getStatorCurrent().getValueAsDouble();
  }


  //////////////////// Methods used for automated testing ////////////////////
  /**
   * Makes the motor ready for testing. This includes disabling any automation that uses this
   * motor
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  @Override
  public void testEnableMotorTestMode(int motorId) {
    // unnecessary
  }

  /**
   * Puts the motor back into normal opeation mode.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   */
  @Override
  public void testDisableMotorTestMode(int motorId) {
    // unnecessary
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
        m_climbMotor.setPosition(0.0, 0.5); // Set it to 0 and wait up to half a second for it to take effect
        break;
      default:
        // Do nothing
        break;
    }
  }

  /**
   * Sets the motor's power to the specified value. This needs to also disable anything else from
   * changing the motor power.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   * @param power Desired power -1.0 to 1.0
   */
  @Override
  public void testSetMotorPower(int motorId, double power) {
    switch(motorId) {
      case 1:
        m_climbMotor.set(power);
        break;
      default:
        // Do nothing
        break;
    }
  }

  /**
   * Returns the motor's current encoder value. Ideally this is the raw value, not the converted
   * value. This should be the INTERNAL encoder to minimize dependencies on other hardware.
   * 
   * @param motorId 1 for the first motor in the subsystem, 2 for the second, etc.
   * @return Encoder value in raw or converted units
   */
  @Override
  public double testGetMotorPosition(int motorId) {
    double position = 0.0;

    switch(motorId) {
      case 1:
        position = m_climbMotor.getPosition().getValueAsDouble(); // This position is affected by the conversion factor
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
        current = getElectricalCurrent();
        break;
      default:
        // Return an invalid value
        current = -1.0;
        break;
    }

    return current;
  }
}
