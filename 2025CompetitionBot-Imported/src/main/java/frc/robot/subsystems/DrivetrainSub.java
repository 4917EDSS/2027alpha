package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import java.util.function.Supplier;
import java.util.logging.Logger;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.RobotStatus;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 * If the robot is saying that the storage is low then check if a usb is plugged
 * in to the
 * robot and restart the robot.
 * 
 * NOTE: This was called CommandSwerveDrivetrain but we renamed it to fit our
 * convention
 */

public class DrivetrainSub extends TunerSwerveDrivetrain implements Subsystem {
  private static Logger m_logger = Logger.getLogger(DrivetrainSub.class.getName());
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  private final Field2d m_field = new Field2d();

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

  /* Swerve requests to apply during SysId characterization */
  private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  // private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
  // private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

  /*
   * SysId routine for characterizing translation. This is used to find PID gains
   * for the drive motors.
   */
  private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
      new SysIdRoutine.Config(
          null, // Use default ramp rate (1 V/s)
          Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
          null, // Use default timeout (10 s)
          // Log state with SignalLogger class
          state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
      new SysIdRoutine.Mechanism(
          output -> setControl(m_translationCharacterization.withVolts(output)),
          null,
          this));

  /*
   * SysId routine for characterizing steer. This is used to find PID gains for
   * the steer motors.
   */
  // private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
  //     new SysIdRoutine.Config(
  //         null, // Use default ramp rate (1 V/s)
  //         Volts.of(7), // Use dynamic voltage of 7 V
  //         null, // Use default timeout (10 s)
  //         // Log state with SignalLogger class
  //         state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
  //     new SysIdRoutine.Mechanism(
  //         volts -> setControl(m_steerCharacterization.withVolts(volts)),
  //         null,
  //         this));

  /*
   * SysId routine for characterizing rotation.
   * This is used to find PID gains for the FieldCentricFacingAngle
   * HeadingController.
   * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
   * importing the log to SysId.
   */
  // private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
  //     new SysIdRoutine.Config(
  //         /* This is in radians per second², but SysId only supports "volts per second" */
  //         Volts.of(Math.PI / 6).per(Second),
  //         /* This is in radians per second, but SysId only supports "volts" */
  //         Volts.of(Math.PI),
  //         null, // Use default timeout (10 s)
  //         // Log state with SignalLogger class
  //         state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
  //     new SysIdRoutine.Mechanism(
  //         output -> {
  //           /* output is actually radians per second, but SysId only supports "volts" */
  //           setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
  //           /* also log the requested output for SysId */
  //           SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
  //         },
  //         null,
  //         this));

  /* The SysId routine to test */
  private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   * <p>
   * This constructs the underlying hardware devices, so users should not
   * construct
   * the devices themselves. If they need the devices, they can access them
   * through
   * getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public DrivetrainSub(
      SwerveDrivetrainConstants drivetrainConstants,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    if(Utils.isSimulation()) {
      startSimThread();
    }

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      throw new RuntimeException(e);
    }

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds. Also optionally outputs individual
        // module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
            // drive trains
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // Currently doesn't work and I do not know which variable is supposed to go
        // there
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if(alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

    initializeFieldDashboard();
    init();
  }

  public void init() {
    m_logger.info("Initializing DrivetrainSub Subsystem");
  }

  /**
   * Returns a command that applies the specified control request to this swerve
   * drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine
   * specified by {@link #m_sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  public Pose2d getPose() {
    return getState().Pose;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return getState().Speeds;
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds));
  }

  @Override
  public void periodic() {

    ChassisSpeeds robotSpeeds = getRobotRelativeSpeeds();
    SmartDashboard.putNumber("Chassis vx", robotSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Chassis vy", robotSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Chassis rotation velocity", robotSpeeds.omegaRadiansPerSecond);
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled.
     * This ensures driving behavior doesn't change until an explicit disable event
     * occurs during testing.
     */
    if(!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent(allianceColor -> {
        RobotStatus.setAlliance(allianceColor);
        setOperatorPerspectiveForward(
            allianceColor == Alliance.Red
                ? kRedAlliancePerspectiveRotation
                : kBlueAlliancePerspectiveRotation);
        m_hasAppliedOperatorPerspective = true;
      });
    }
    m_field.setRobotPose(getPose());
    SmartDashboard.putNumber("Dr X Pos", getState().Pose.getX());
    SmartDashboard.putNumber("Dr Y Pos", getState().Pose.getY());
    SmartDashboard.putNumber("Dr Speed",
        Math.sqrt(Math.pow(getState().Speeds.vxMetersPerSecond, 2) + Math.pow(getState().Speeds.vyMetersPerSecond, 2)));
    SmartDashboard.putNumber("Dr Heading", getState().Pose.getRotation().getDegrees());
    SmartDashboard.putBoolean("Left?", RobotStatus.isLeft());
    SmartDashboard.putBoolean("Right?", !RobotStatus.isLeft());
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  private void initializeFieldDashboard() {
    // Display field position
    SmartDashboard.putData("Field", m_field);
    m_field.setRobotPose(getState().Pose);
  }

}
