// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex m_shooterMotor = new SparkFlex(ShooterConstants.kShooterMotorId, MotorType.kBrushless);
  private final VictorSPX m_indexerMotor = new VictorSPX(ShooterConstants.kIndexerMotorId);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter RPM", m_shooterMotor.getEncoder().getVelocity());
  }

  public void runShooter() {
    m_shooterMotor.getClosedLoopController().setSetpoint(2900, ControlType.kVelocity);
    if (m_shooterMotor.getEncoder().getVelocity() >= 2900) {
      m_indexerMotor.set(VictorSPXControlMode.PercentOutput, -0.6);
    } else {
      m_indexerMotor.set(VictorSPXControlMode.PercentOutput, -0.3);
    }
  }

  public void stopShooter() {
    m_shooterMotor.getClosedLoopController().setSetpoint(0, ControlType.kVelocity);
    m_indexerMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }
}
