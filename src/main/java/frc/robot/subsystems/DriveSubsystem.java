// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.K_FRONT_LEFT_DRIVING_CAN_ID,
      DriveConstants.K_FRONT_LEFT_TURNING_CAN_ID,
      DriveConstants.K_FRONT_LEFT_TURNING_ENCODER_ID,
      DriveConstants.K_FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.K_FRONT_RIGHT_DRIVING_CAN_ID,
      DriveConstants.K_FRONT_RIGHT_TURNING_CAN_ID,
      DriveConstants.K_FRONT_RIGHT_TURNING_ENCODER_ID,
      DriveConstants.K_FRONT_RIGHT_CHASSIS_ANGULAR_OFSFET);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.K_REAR_LEFT_DRIVING_CAN_ID,
      DriveConstants.K_REAR_LEFT_TURNING_CAN_ID,
      DriveConstants.K_BACK_LEFT_TURNING_ENCODER_ID,
      DriveConstants.K_BACK_LEFT_CHASSIS_ANGLUAR_OFFSET);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.K_REAR_RIGHT_DRIVING_CAN_ID,
      DriveConstants.K_REAR_RIGHT_TURNING_CAN_ID,
      DriveConstants.K_BACK_RIGHT_TURNING_ENCODER_ID,
      DriveConstants.K_BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.K_GYRO_CAN_ID);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.K_DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = Constants.DriveConstants.FILTER_X.calculate(xSpeed * DriveConstants.K_MAX_SPEED_METERS_PER_SECOND);
    double ySpeedDelivered = Constants.DriveConstants.FILTER_Y.calculate(ySpeed * DriveConstants.K_MAX_SPEED_METERS_PER_SECOND);
    double rotDelivered = rot * DriveConstants.K_MAX_ANGULAR_SPEED;

    SwerveModuleState[] swerveModuleStates = DriveConstants.K_DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.K_MAX_SPEED_METERS_PER_SECOND);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  /**public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
     m_frontLeft.setDesiredState(desiredStates[0]);
     m_frontRight.setDesiredState(desiredStates[1]);
     m_rearLeft.setDesiredState(desiredStates[2]);
     m_rearRight.setDesiredState(desiredStates[3]);

  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getYaw().getValueAsDouble() * (DriveConstants.K_GYRO_REVERSED ? -1.0 : 1.0);
  }
}
