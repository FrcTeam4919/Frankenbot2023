// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXR450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class MechDrive extends SubsystemBase {
  /** Creates a new mechanum drive train. */
  private final PWMSparkMax m_leftFront = new PWMSparkMax(0);
  private final PWMSparkMax m_leftRear = new PWMSparkMax(1);
  private final PWMSparkMax m_rightFront = new PWMSparkMax(2);
  private final PWMSparkMax m_rightRear = new PWMSparkMax(3);

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

  public MechDrive() {
  }
 


  /**
   *
   * @param xSpeed input for forward/backward movement
   * @param ySpeed input for side to side movement
   * @param rot
   */
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
