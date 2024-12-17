// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.DriveConstants;

import frc.robot.Configs;

public class MAXSwerveModule {
  private final CANSparkMax m_drivingSpark;
  private final CANSparkMax m_turningSpark;

  public final CANCoder m_absoluteEncoder;  

  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkPIDController m_drivingClosedLoopController;
  private final SparkPIDController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(
    int driveMotorChannel,
    int turningMotorChannel,
    int turningEncoderChannel,
    double offset) {
    m_drivingSpark = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningSpark = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    m_drivingClosedLoopController = m_drivingSpark.getPIDController();
    m_turningClosedLoopController = m_turningSpark.getPIDController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
     m_drivingSpark = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
     m_turningSpark = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
     m_drivingSpark.setIdleMode(IdleMode.kBrake);
     m_turningSpark.setInverted(false);
     m_turningSpark.burnFlash();
     m_drivingSpark.burnFlash();
 
     m_drivingEncoder = m_drivingSpark.getEncoder();
     m_drivingEncoder.setPosition(0);
     m_turningEncoder = m_turningSpark.getEncoder();
     m_absoluteEncoder = new CANCoder(turningEncoderChannel);
 
     // Configuration settings for Absolute Encoder
     CANCoderConfiguration config = new CANCoderConfiguration();
     config.sensorCoefficient = 2 * Math.PI / kEncoderResolution;
     config.unitString = "rad";
     config.sensorTimeBase = SensorTimeBase.PerSecond;
     config.magnetOffsetDegrees = 0;
     config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;

     config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
     m_absoluteEncoder.configAllSettings(config);
 
     m_driveEncoder.setPositionConversionFactor(2 * Math.PI * kWheelRadius / kEncoderResolution);
     m_turningEncoder.setPositionConversionFactor(12.8 * Math.PI * 2);
 
     // Limit the PID Controller's input range between -pi and pi and set the input
     // to be continuous.
     m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
 
     absoluteEncoderOffset = offset;
 
     m_chassisAngularOffset = chassisAngularOffset;
     m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
     m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
