// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class FeedWheel extends SubsystemBase {
  /** Creates a new Shooter. */
  private WPI_TalonSRX feedWheel = new WPI_TalonSRX(Constants.feedWheelID);
  private PIDController PID = new PIDController(Constants.feedWheelPIDConstants.kp, Constants.feedWheelPIDConstants.ki, Constants.feedWheelPIDConstants.kd, 0.01);
  private SimpleMotorFeedforward FF = new SimpleMotorFeedforward(Constants.feedWheelFF.kS, Constants.feedWheelFF.kV, Constants.feedWheelFF.kA);
  public FeedWheel() {
    feedWheel.configFactoryDefault();
    feedWheel.setInverted(false);
    feedWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

  }

  public double getRPM(){
    return(feedWheel.getSelectedSensorVelocity() / 4096 * 600);
  }
  public void setFeedOnOff(boolean onOrOff){
    if (onOrOff) {
      feedWheel.set(ControlMode.PercentOutput, 0.5);
    }
    else {
      feedWheel.set(ControlMode.PercentOutput, 0.0);
    }
  }

  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feed Wheel RPM: ", getRPM());
    setFeedOnOff(RobotContainer.getJoystick().getTrigger());


    // This method will be called once per scheduler run
  }
}
