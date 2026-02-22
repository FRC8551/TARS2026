// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive m_swerve;
  private final CommandXboxController m_driverController;

  private final PIDController m_hubAimController = new PIDController(0.05, 0, 0);

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(CommandXboxController driverController) {

    m_driverController = driverController;

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      m_swerve = new SwerveParser(SwerveConstants.kSwerveConfigurationDirectory)
          .createSwerveDrive(SwerveConstants.kMaxSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    // Swerve configuration
    m_swerve.setAngularVelocityCompensation(true, true, 0.1);
    m_swerve.setAutoCenteringModules(false);
    // m_swerve.setChassisDiscretization(true, 0.02); // Change this to true
    m_swerve.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    m_swerve.setHeadingCorrection(true);
    // m_swerve.setModuleEncoderAutoSynchronize(true, 1);
    // m_swerve.setModuleStateOptimization(true);
    m_swerve.setMotorIdleMode(false);

    if (SwerveDriveTelemetry.isSimulation) {
      m_swerve.resetOdometry(new Pose2d(2.5, 4.030, Rotation2d.fromDegrees(0)));
    }

    setupPathPlanner();

    new VisionSubsystem(m_swerve::getYaw, m_swerve.getGyro().getYawAngularVelocity()::magnitude,
        m_swerve::addVisionMeasurement, m_swerve::setVisionMeasurementStdDevs);

    m_hubAimController.enableContinuousInput(0, 359);
  }

  @Override
  public void periodic() {
    ChassisSpeeds robotVelocity = m_swerve.getRobotVelocity();
    double velocity = Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond) * 2.23694;
    SmartDashboard.putNumber(SwerveConstants.kSlash + "Speedometer (MPH)", Math.round(velocity * 1000.0) / 1000.0);

    double[] fuel = SmartDashboard.getEntry("Fuel").getDoubleArray(new double[0]);
    Pose2d robotPose = m_swerve.getPose();

    int count = fuel.length / 3;
    double[] arr = new double[count * 3];

    for (int i = 0; i < count; i++) {

      double xRobot = fuel[i * 3 + 1];
      double yRobot = fuel[i * 3 + 2];

      Transform2d robotToFuel = new Transform2d(
          new Translation2d(xRobot, yRobot),
          new Rotation2d());

      Pose2d fuelFieldPose = robotPose.transformBy(robotToFuel);

      int base = i * 3;
      arr[base] = fuelFieldPose.getX();
      arr[base + 1] = fuelFieldPose.getY();
      arr[base + 2] = 0.0; // On the floor
    }

    SmartDashboard.getEntry("Field/Fuel").setDoubleArray(arr);

    // Limelight 3G MT1
    // LimelightHelpers.PoseEstimate ll3Gmt1 =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-better");
    // SmartDashboard.putString("ll3Gmt1 Pose", ll3Gmt1.pose.toString());
    // if (ll3Gmt1 != null && ll3Gmt1.tagCount > 0) {
    // m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 0.5));
    // m_swerve.addVisionMeasurement(new Pose2d(6.5, 4.5,
    // Rotation2d.fromDegrees(90)), ll3Gmt1.timestampSeconds);
    // }

    // Limelight 3G MT2
    // LimelightHelpers.SetRobotOrientation("limelight-better",
    // m_swerve.swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(),
    // 0, 0, 0, 0, 0);
    // LimelightHelpers.PoseEstimate ll3Gmt2 =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-better");
    // SmartDashboard.putString("ll3Gmt2 Pose", ll3Gmt2.pose.toString());
    // if (ll3Gmt2 != null && ll3Gmt2.tagCount > 0) {
    // m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 0.5));
    // m_swerve.addVisionMeasurement(ll3Gmt2.pose, ll3Gmt2.timestampSeconds);
    // }

    // LimelightHelpers.SetRobotOrientation("limelight-better",
    // m_swerve.swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(),
    // 0, 0, 0, 0, 0);
    // LimelightHelpers.PoseEstimate ll3Gmt2 =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-better");
    // if (ll3Gmt2 != null &&
    // Math.abs(m_swerve.getGyro().getYawAngularVelocity().magnitude()) <= 720 &&
    // ll3Gmt2.tagCount != 0) {
    // System.out.println(ll3Gmt2.tagCount);
    // m_swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 0.5));
    // m_swerve.addVisionMeasurement(ll3Gmt2.pose, ll3Gmt2.timestampSeconds);
    // }
  }

  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          m_swerve::getPose,
          m_swerve::resetOdometry,
          m_swerve::getRobotVelocity,
          (speeds, feedforwards) -> m_swerve.drive(speeds),
          new PPHolonomicDriveController(
              new PIDConstants(5, 0, 0),
              new PIDConstants(5, 0, 0)),
          config,
          this::isRedAlliance,
          this);
    } catch (Exception e) {
      e.printStackTrace();
    }

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      m_swerve.field.getObject("path").setPoses(poses);
    });
  }

  public Command driveToPose(Pose2d pose) {
    PathConstraints constraints = new PathConstraints(
        m_swerve.getMaximumChassisVelocity(), 1.0,
        m_swerve.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        MetersPerSecond.of(0));
  }

  public Command driveRobotOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      if (m_driverController.rightBumper().getAsBoolean()) {
        var robotTranslation = m_swerve.getPose().getTranslation();
        var targetTranslation = isRedAlliance() ? new Translation2d(4.623, 4.030) : new Translation2d(11.917, 4.030);

        var angle = targetTranslation.minus(robotTranslation).getAngle();

        m_swerve.drive(new ChassisSpeeds(
            velocity.get().vxMetersPerSecond,
            velocity.get().vyMetersPerSecond,
            m_hubAimController.calculate(m_swerve.getOdometryHeading().getDegrees(), angle.getDegrees())));
      } else {
        m_swerve.drive(velocity.get());
      }
    });
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      if (m_driverController.rightBumper().getAsBoolean()) {
        var robotTranslation = m_swerve.getPose().getTranslation();
        var targetTranslation = isRedAlliance() ? new Translation2d(4.623, 4.030) : new Translation2d(11.917, 4.030);

        var angle = targetTranslation.minus(robotTranslation).getAngle();

        m_swerve.driveFieldOriented(new ChassisSpeeds(
            velocity.get().vxMetersPerSecond,
            velocity.get().vyMetersPerSecond,
            m_hubAimController.calculate(m_swerve.getOdometryHeading().getDegrees(), angle.getDegrees())));
      } else {
        m_swerve.driveFieldOriented(velocity.get());
      }

    });
  }

  public SwerveDrive getSwerveDrive() {
    return m_swerve;
  }

  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public void zeroGyro() {
    m_swerve.zeroGyro();
  }
}
