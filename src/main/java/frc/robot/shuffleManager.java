package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class shuffleManager {
    private ShuffleboardTab shuffleTab = Shuffleboard.getTab("tooning");
    private GenericEntry slew = shuffleTab.add("slew", Constants.DriveConstants.slewRateBase).getEntry();
    private GenericEntry maxSpeed = shuffleTab.add("max speed", Constants.DriveConstants.kMaxSpeedMetersPerSecond).getEntry();
    private GenericEntry maxRot = shuffleTab.add("max rot", Constants.DriveConstants.kMaxAngularSpeed).getEntry();

    public void initShuffleboard() {
        updateShuffleboard();
    }
    public void updateShuffleboard() {
        Constants.DriveConstants.kMaxSpeedMetersPerSecond = maxSpeed.getDouble(Constants.DriveConstants.kMaxSpeedMetersPerSecond);
        Constants.DriveConstants.kMaxAngularSpeed = maxRot.getDouble(Constants.DriveConstants.kMaxAngularSpeed);
        if (Constants.DriveConstants.slewRateBase != slew.getDouble(Constants.DriveConstants.slewRateBase)) {
            Constants.DriveConstants.slewRateBase = slew.getDouble(Constants.DriveConstants.slewRateBase);
            Constants.DriveConstants.filterx = new SlewRateLimiter(Constants.DriveConstants.slewRateBase);
            Constants.DriveConstants.filtery = new SlewRateLimiter(Constants.DriveConstants.slewRateBase);
        }
    }
}