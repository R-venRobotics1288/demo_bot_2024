// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final boolean SHUFFLE_MANAGER_ENABLE = true;
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double K_MAX_SPEED_METERS_PER_SECOND = 15;

    public static final double MAX_ROT_RPS = 1;
    public static final double K_MAX_ANGULAR_SPEED = 2 * Math.PI * MAX_ROT_RPS; // radians per second

    public static final double SLEW_RATE_BASE = 15;
    public static final SlewRateLimiter FILTER_X = new SlewRateLimiter(SLEW_RATE_BASE);
    public static final SlewRateLimiter FILTER_Y = new SlewRateLimiter(SLEW_RATE_BASE);

    // Chassis configuration
    public static final double K_TRACK_WIDTH = Units.inchesToMeters(22.5);
    // Distance between centers of right and left wheels on robot
    public static final double K_WHEEL_BASE = Units.inchesToMeters(22.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics K_DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(K_WHEEL_BASE / 2, K_TRACK_WIDTH / 2),
        new Translation2d(K_WHEEL_BASE / 2, -K_TRACK_WIDTH / 2),
        new Translation2d(-K_WHEEL_BASE / 2, K_TRACK_WIDTH / 2),
        new Translation2d(-K_WHEEL_BASE / 2, -K_TRACK_WIDTH / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double K_FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double K_FRONT_RIGHT_CHASSIS_ANGULAR_OFSFET = 0;
    public static final double K_BACK_LEFT_CHASSIS_ANGLUAR_OFFSET = 0;
    public static final double K_BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI;

    // SPARK MAX CAN IDs
    public static final int K_FRONT_LEFT_DRIVING_CAN_ID = 1;
    public static final int K_REAR_LEFT_DRIVING_CAN_ID = 7;
    public static final int K_FRONT_RIGHT_DRIVING_CAN_ID = 8;
    public static final int K_REAR_RIGHT_DRIVING_CAN_ID = 3;

    public static final int K_FRONT_LEFT_TURNING_CAN_ID = 10;
    public static final int K_REAR_LEFT_TURNING_CAN_ID = 6;
    public static final int K_FRONT_RIGHT_TURNING_CAN_ID = 2;
    public static final int K_REAR_RIGHT_TURNING_CAN_ID = 9;

    public static final int K_FRONT_LEFT_TURNING_ENCODER_ID = 20;
    public static final int K_FRONT_RIGHT_TURNING_ENCODER_ID = 21;
    public static final int K_BACK_LEFT_TURNING_ENCODER_ID = 22;
    public static final int K_BACK_RIGHT_TURNING_ENCODER_ID = 23;

    public static final int K_GYRO_CAN_ID = 30;

    public static final boolean K_GYRO_REVERSED = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int K_DRIVING_MOTOR_PINION_TEETH = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double K_DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.K_FREE_SPEED_RPM / 60;
    public static final double K_WHEEL_DIAMETER_METERS = 0.0762;
    public static final double K_WHEEL_CIRCUMFERENCE_METERS = K_WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double K_DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (K_DRIVING_MOTOR_PINION_TEETH * 15);
    public static final double K_DRIVE_WHEEL_SPEED_RPS = (K_DRIVING_MOTOR_FREE_SPEED_RPS * K_WHEEL_CIRCUMFERENCE_METERS)
        / K_DRIVING_MOTOR_REDUCTION;
  }

  public static final class OIConstants {
    public static final int K_DRIVER_CONTROLLER_PORT = 0;
    public static final double K_DRIVE_DEADBAND = 0.05;
  }

  public static final class AutoConstants {
    public static final double K_MAX_SPEED_METERS_PER_SECOND = 3;
    public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
    public static final double K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
    public static final double K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

    public static final double K_PX_CONTROLLER = 1;
    public static final double K_PY_CONTROLLER = 1;
    public static final double K_P_THETA_CONTROLLER = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints K_THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(
        K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, K_MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
  }

  public static final class NeoMotorConstants {
    public static final double K_FREE_SPEED_RPM = 5676;
  }
}