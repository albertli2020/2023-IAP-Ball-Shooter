// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feedwheel extends SubsystemBase {
  /** Creates a new Shooter. */
  private WPI_TalonSRX feedwheel = new WPI_TalonSRX(Constants.leftFlyWheelID);
  private PIDController PID = new PIDController(Constants.leftFlyWheelPIDConstants.kp, Constants.leftFlyWheelPIDConstants.ki, Constants.leftFlyWheelPIDConstants.kd, 0.01);
  private SimpleMotorFeedforward FF = new SimpleMotorFeedforward(Constants.leftFlyWheelFF.kS, Constants.leftFlyWheelFF.kV, Constants.leftFlyWheelFF.kA);
  public Feedwheel() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
