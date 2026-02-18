// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

    private final Supplier<Rotation2d> getYaw;
    private final BiConsumer<Pose2d, Double> addVisionMeasurement;
    private final Consumer<Matrix<N3, N1>> setVisionStdDevs;

    private final String[] limelights = { "limelight", "limelight" };

    public VisionSubsystem(
            Supplier<Rotation2d> getYaw,
            Supplier<Double> getYawRate,
            BiConsumer<Pose2d, Double> addVisionMeasurement,
            Consumer<Matrix<N3, N1>> setVisionStdDevs) {
        this.getYaw = getYaw;
        this.addVisionMeasurement = addVisionMeasurement;
        this.setVisionStdDevs = setVisionStdDevs;
    }

    @Override
    public void periodic() {
        Rotation2d yaw = getYaw.get();
        double yawDeg = yaw.getDegrees();

        for (String ll : limelights) {

            // Feed gyro heading to MegaTag2
            LimelightHelpers.SetRobotOrientation(ll, yawDeg, 0, 0, 0, 0, 0);

            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(ll);

            boolean valid = mt2.tagCount > 0;

            if (valid) {

                // Dynamic trust based on tag count
                double xyStdDev = mt2.tagCount >= 2 ? 0.3 : 0.6;

                setVisionStdDevs.accept(
                        VecBuilder.fill(xyStdDev, xyStdDev, 9999999));

                addVisionMeasurement.accept(
                        mt2.pose,
                        mt2.timestampSeconds);

                continue;
            }

            // Fallback to MegaTag1
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(ll);

            if (mt1.tagCount > 0) {

                setVisionStdDevs.accept(
                        VecBuilder.fill(0.7, 0.7, 9999999));

                addVisionMeasurement.accept(
                        mt1.pose,
                        mt1.timestampSeconds);
            }
        }
    }
}