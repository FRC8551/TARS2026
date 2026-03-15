package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UserConfig {

    public enum DriveMode {
        RobotOriented,
        FieldOrientedAngularVelocity,
        FieldOrientedDirectAngle
    }

    private static final SendableChooser<DriveMode> m_driveModeChooser = new SendableChooser<>();
    private static final SendableChooser<Boolean> m_hubAimChooser = new SendableChooser<>();
    private static final SendableChooser<Boolean> m_bumpAimChooser = new SendableChooser<>();
    private static final SendableChooser<Boolean> m_beansModeChooser = new SendableChooser<>();
    private static final SendableChooser<Boolean> m_apriltagLocalizationChooser = new SendableChooser<>();

    public static final void initialize() {
        m_driveModeChooser.setDefaultOption("Field-Oriented Direct Angle", DriveMode.FieldOrientedDirectAngle);
        m_driveModeChooser.addOption("Field-Oriented Angular Velocity", DriveMode.FieldOrientedAngularVelocity);
        m_driveModeChooser.addOption("Robot-Oriented", DriveMode.RobotOriented);

        m_hubAimChooser.setDefaultOption("Enabled", true);
        m_hubAimChooser.addOption("Disabled", false);

        m_bumpAimChooser.setDefaultOption("Enabled", true);
        m_bumpAimChooser.addOption("Disabled", false);

        m_beansModeChooser.setDefaultOption("Enabled", true);
        m_beansModeChooser.addOption("Disabled", false);

        m_apriltagLocalizationChooser.setDefaultOption("Enabled", true);
        m_apriltagLocalizationChooser.addOption("Disabled", false);

        SmartDashboard.putData("Drive Mode", m_driveModeChooser);
        SmartDashboard.putData("Hub Aim", m_hubAimChooser);
        SmartDashboard.putData("Bump Aim", m_bumpAimChooser);
        SmartDashboard.putData("Beans Mode", m_beansModeChooser);
        SmartDashboard.putData("AprilTag Localization", m_apriltagLocalizationChooser);
    }

    public static DriveMode getDriveMode() {
        return m_driveModeChooser.getSelected();
    }

    public static boolean getHubAimEnabled() {
        return m_hubAimChooser.getSelected();
    }

    public static boolean getBumpAimEnabled() {
        return m_bumpAimChooser.getSelected();
    }

    public static boolean getBeansModeEnabled() {
        return m_beansModeChooser.getSelected();
    }

    public static boolean getAprilTagLocalizationEnabled() {
        return m_apriltagLocalizationChooser.getSelected();
    }
}
