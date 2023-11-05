// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  private WPI_TalonSRX flyWheel = new WPI_TalonSRX(Constants.leftFlyWheelID);
  private PIDController PID = new PIDController(Constants.leftFlyWheelPIDConstants.kp, Constants.leftFlyWheelPIDConstants.ki, Constants.leftFlyWheelPIDConstants.kd, 0.01);
  private SimpleMotorFeedforward FF = new SimpleMotorFeedforward(Constants.leftFlyWheelFF.kS, Constants.leftFlyWheelFF.kV, Constants.leftFlyWheelFF.kA);
  public Flywheel() {}

  public double getEncoder(){
    return(flyWheel.getSelectedSensorPosition());
  }
  public double getPID(double goal){
    return(PID.calculate(getEncoder(), goal));
  }
  
  public double getFF(double goal){
    return(FF.calculate(goal));
  }

  public void setSpeed(double goal){
    flyWheel.set(getFF(goal) + getPID(goal), goal);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
