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
    private GenericEntry slew = shuffleTab.addPersistent("slew", Constants.DriveConstants.slewRateBase).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1, "max", 50)).getEntry();
    private GenericEntry maxSpeed = shuffleTab.addPersistent("max speed", Constants.DriveConstants.kMaxSpeedMetersPerSecond).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1, "max", 75)).getEntry();
    private GenericEntry maxRot = shuffleTab.addPersistent("max wheel rot per s", Constants.DriveConstants.maxRotRps).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1, "max", 30)).getEntry();

    public void initShuffleboard() {
        updateShuffleboard();
    }
    public void updateShuffleboard() {
        Constants.DriveConstants.kMaxSpeedMetersPerSecond = maxSpeed.getDouble(Constants.DriveConstants.kMaxSpeedMetersPerSecond);
        if (Constants.DriveConstants.maxRotRps != maxRot.getDouble(Constants.DriveConstants.maxRotRps)){
            Constants.DriveConstants.maxRotRps = maxRot.getDouble(Constants.DriveConstants.maxRotRps);
            Constants.DriveConstants.kMaxAngularSpeed = maxRot.getDouble(Constants.DriveConstants.maxRotRps) * 2 * Math.PI;
        }
        if (Constants.DriveConstants.slewRateBase != slew.getDouble(Constants.DriveConstants.slewRateBase)) {
            Constants.DriveConstants.slewRateBase = slew.getDouble(Constants.DriveConstants.slewRateBase);
            Constants.DriveConstants.filterx = new SlewRateLimiter(Constants.DriveConstants.slewRateBase);
            Constants.DriveConstants.filtery = new SlewRateLimiter(Constants.DriveConstants.slewRateBase);
        }
    }
}