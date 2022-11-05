// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class MechDrive extends SubsystemBase {
  /** Creates a new mechanum drive train. */
  private final VictorSP m_leftFront = new VictorSP(DriveConstants.kFrontLeftDrive);
  private final VictorSP m_leftRear = new VictorSP(DriveConstants.kBackLeftDrive);
  private final VictorSP m_rightFront = new VictorSP(DriveConstants.kFrontRightDrive);
  private final VictorSP m_rightRear = new VictorSP(DriveConstants.kBackRightDrive);

  private final MecanumDrive m_mechDrive =
   new MecanumDrive(m_leftFront, m_leftRear, m_rightFront, m_rightRear);

  // Front Left Encoder
  private final Encoder m_FrontLeftEncoder
   = new Encoder(
     DriveConstants.kFrontLeftEncoderPorts[1],
     DriveConstants.kFrontLeftEncoderPorts[2],
     DriveConstants.kFrontLeftEncoderReversed);

  // Rear Left Encoder
  private final Encoder m_RearLeftEncoder
  = new Encoder(
    DriveConstants.kRearLeftEncoderPorts[1],
    DriveConstants.kRearLeftEncoderPorts[2],
    DriveConstants.kRearLeftEncoderReversed);

  // Front Right Encoder
  private final Encoder m_FrontRightEncoder
   = new Encoder(
     DriveConstants.kFrontRightEncoderPorts[1],
     DriveConstants.kFrontRightEncoderPorts[2],
     DriveConstants.kFrontRightEncoderReversed);

  // Rear Right Encoder
  private final Encoder m_RearRightEncoder
   = new Encoder(
     DriveConstants.kRearRightEncoderPorts[1],
     DriveConstants.kRearRightEncoderPorts[2],
     DriveConstants.kRearRightEncoderReversed);

  // gyro
  private final Gyro m_gyro = new ADXRS450_Gyro();

  MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

  public MechDrive() {
     // Sets the distance per pulse for the encoders
     m_FrontLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
     m_RearLeftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
     m_FrontRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
     m_RearRightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
     // We need to invert one side of the drivetrain so that positive voltages
     // result in both sides moving forward. Depending on how your robot's
     // gearbox is constructed, you might have to invert the left side instead.
     m_rightFront.setInverted(true);
     m_rightRear.setInverted(true);
  }
 


  
  
  @Override
  public void periodic() {
     // Update the odometry in the periodic block
     m_odometry.update(
        m_gyro.getRotation2d(),
        new MecanumDriveWheelSpeeds(
            m_FrontLeftEncoder.getRate(),
            m_RearLeftEncoder.getRate(),
            m_FrontRightEncoder.getRate(),
            m_RearRightEncoder.getRate()));
    // This method will be called once per scheduler run
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   *
   * @param xSpeed input for forward/backward movement
   * @param ySpeed input for side to side movement
   * @param rot Angle for rotation
   * @param fieldRelative
   */  

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      m_mechDrive.driveCartesian(ySpeed, xSpeed, rot, -m_gyro.getAngle());
    } else
      m_mechDrive.driveCartesian(ySpeed, xSpeed, rot);
    }
  

  /** Sets the front left drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_leftFront.setVoltage(volts.frontLeftVoltage);
    m_leftRear.setVoltage(volts.rearLeftVoltage);
    m_rightFront.setVoltage(volts.frontRightVoltage);
    m_rightRear.setVoltage(volts.rearRightVoltage);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_FrontLeftEncoder.reset();
    m_RearLeftEncoder.reset();
    m_FrontRightEncoder.reset();
    m_RearRightEncoder.reset();
  }

  /**
   * Gets the front left drive encoder.
   *
   * @return the front left drive encoder
   */
  public Encoder getFrontLeftEncoder() {
    return m_FrontLeftEncoder;
  }

  /**
   * Gets the rear left drive encoder.
   *
   * @return the rear left drive encoder
   */
  public Encoder getRearLeftEncoder() {
    return m_RearLeftEncoder;
  }

  /**
   * Gets the front right drive encoder.
   *
   * @return the front right drive encoder
   */
  public Encoder getFrontRightEncoder() {
    return m_FrontRightEncoder;
  }

  /**
   * Gets the rear right drive encoder.
   *
   * @return the rear right encoder
   */
  public Encoder getRearRightEncoder() {
    return m_RearRightEncoder;
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(
        m_FrontLeftEncoder.getRate(),
        m_RearLeftEncoder.getRate(),
        m_FrontRightEncoder.getRate(),
        m_RearRightEncoder.getRate());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_mechDrive.setMaxOutput(maxOutput);
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
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
