package frc.robot;

import java.io.Console;
import java.util.Map;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs.MAXSwerveModule;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class shuffleManager {
    private ShuffleboardTab shuffleTab = Shuffleboard.getTab("tooning");
    private GenericEntry slew = shuffleTab.add("slew", Constants.DriveConstants.SLEW_RATE_BASE).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1, "max", 50)).getEntry();
    private GenericEntry maxSpeed = shuffleTab.add("max speed", Constants.DriveConstants.K_MAX_SPEED_METERS_PER_SECOND).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1, "max", 75)).getEntry();
    private GenericEntry maxRot = shuffleTab.add("max wheel rot per s", Constants.DriveConstants.MAX_ROT_RPS).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1, "max", 30)).getEntry();

    public void initShuffleboard() {
        updateShuffleboard();
    }
    public void updateShuffleboard() {
        if (Constants.DriveConstants.SHUFFLE_MANAGER_ENABLE) {
            Constants.DriveConstants.K_MAX_SPEED_METERS_PER_SECOND = maxSpeed.getDouble(Constants.DriveConstants.K_MAX_SPEED_METERS_PER_SECOND);
            if (Constants.DriveConstants.MAX_ROT_RPS != maxRot.getDouble(Constants.DriveConstants.MAX_ROT_RPS)){
                Constants.DriveConstants.MAX_ROT_RPS = maxRot.getDouble(Constants.DriveConstants.MAX_ROT_RPS);
                Constants.DriveConstants.K_MAX_ANGULAR_SPEED = maxRot.getDouble(Constants.DriveConstants.MAX_ROT_RPS) * 2 * Math.PI;
            }
            if (Constants.DriveConstants.SLEW_RATE_BASE != slew.getDouble(Constants.DriveConstants.SLEW_RATE_BASE)) {
                Constants.DriveConstants.SLEW_RATE_BASE = slew.getDouble(Constants.DriveConstants.SLEW_RATE_BASE);
                Constants.DriveConstants.FILTER_X = new SlewRateLimiter(Constants.DriveConstants.SLEW_RATE_BASE);
                Constants.DriveConstants.FILTER_Y = new SlewRateLimiter(Constants.DriveConstants.SLEW_RATE_BASE);
            }
        }
    }
}