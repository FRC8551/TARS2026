// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax m_lowerIndexerMotor = new SparkMax(ShooterConstants.kLowerIndexerMotorId,
      MotorType.kBrushless);
  private final SparkMax m_upperIndexerMotor = new SparkMax(ShooterConstants.kUpperIndexerMotorId,
      MotorType.kBrushless);
  private final SparkFlex m_shooterLeftMotor = new SparkFlex(ShooterConstants.kShooterLeftMotorId,
      MotorType.kBrushless);
  private final SparkFlex m_shooterRightMotor = new SparkFlex(ShooterConstants.kShooterRightMotorId,
      MotorType.kBrushless);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    SparkMaxConfig lowerIndexerConfig = new SparkMaxConfig();
    SparkMaxConfig upperIndexerConfig = new SparkMaxConfig();

    upperIndexerConfig.apply(lowerIndexerConfig);

    m_lowerIndexerMotor.configure(lowerIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_upperIndexerMotor.configure(upperIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkFlexConfig shooterRightConfig = new SparkFlexConfig();
    SparkFlexConfig shooterLeftConfig = new SparkFlexConfig();

    shooterRightConfig.closedLoop.feedForward.kV(0.00015);
    shooterRightConfig.closedLoop.p(0.001);
    shooterRightConfig.idleMode(IdleMode.kCoast);
    shooterRightConfig.inverted(false);

    shooterLeftConfig.apply(shooterRightConfig);
    shooterLeftConfig.inverted(true);

    m_shooterLeftMotor.configure(shooterLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_shooterRightMotor.configure(shooterRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(ShooterConstants.kSlash + "Shooter RPM", m_shooterRightMotor.getEncoder().getVelocity());
  }

  public Command runShooter() {
    return run(() -> {
      m_shooterLeftMotor.getClosedLoopController().setSetpoint(ShooterConstants.kShooterRPM,
          ControlType.kVelocity);
      m_shooterRightMotor.getClosedLoopController().setSetpoint(ShooterConstants.kShooterRPM,
          ControlType.kVelocity);
      if (m_shooterRightMotor.getEncoder().getVelocity() > ShooterConstants.kShooterRPM
          - ShooterConstants.kShooterRPMTolerance) {
        m_lowerIndexerMotor.set(0.75);
        m_upperIndexerMotor.set(0.85);
      } else {
        m_lowerIndexerMotor.set(0.37);
        m_upperIndexerMotor.set(0.85);
      }
    });
  }

  public Command stopShooter() {
    return run(() -> {
      m_shooterLeftMotor.set(0);
      m_shooterRightMotor.set(0);
      m_lowerIndexerMotor.set(0);
      m_upperIndexerMotor.set(0);
    });
  }
}
