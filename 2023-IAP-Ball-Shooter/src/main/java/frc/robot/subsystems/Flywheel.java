// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants;
public class FlyWheel extends SubsystemBase {
  /* Creates a new Flywheel. */
  private WPI_TalonSRX flyWheel = new WPI_TalonSRX(Constants.flyWheelID);
  private PIDController PID = new PIDController(Constants.flyWheelPIDConstants.kp, Constants.flyWheelPIDConstants.ki, Constants.flyWheelPIDConstants.kd, 0.01);
  private SimpleMotorFeedforward FF = new SimpleMotorFeedforward(Constants.flyWheelFF.kS, Constants.flyWheelFF.kV, Constants.flyWheelFF.kA);
  private Timer timer = new Timer();
  boolean flyWheelOn = false;
  double speed = 0;
  double initialTime;
  boolean achievedInitialSpeed = false;
  boolean recovery = false;
  ArrayList<Double> timesToRecover = new ArrayList<>();
  public FlyWheel() {
    flyWheel.configFactoryDefault();
    flyWheel.setInverted(false);
    flyWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    PID.setSetpoint(0.0);
    PID.setTolerance(0.5);
    timer.reset();
    resetEncoder();
  }

 
  public double getRPM(){
    return(flyWheel.getSelectedSensorVelocity() / 4096 * 600);
  }

  public void setSpeed(double goal){
    flyWheel.set(FF.calculate(goal) + PID.calculate(getRPM(), goal));
  }

  public void applyPower(){
    flyWheel.set(ControlMode.PercentOutput, 0.5);
  }
  
  public void resetEncoder(){
    flyWheel.setSelectedSensorPosition(0);
  }

  public double getTime(){
    return timer.get();
  }

  public void updateSpeed(double newSpeed){
    //new speed in rpm
    speed = newSpeed;
  }

  public void checkRecovery(double currRPM){
    if(Math.abs(currRPM - speed) > 20){
      recovery = true;
      timer.start();
    }
  }

  

  @Override
  public void periodic() {
    double currRPM = getRPM();
    SmartDashboard.putNumber("fly wheel rpm: ", currRPM);
    SmartDashboard.putNumber("current goal: ", PID.getSetpoint());
    SmartDashboard.putNumber("current time: ", getTime());
    SmartDashboard.putNumber("Time to get up to speed: ", initialTime);
    for(int i = 0; i < timesToRecover.size(); i ++){
      String shotNumber = "Shot ";
      shotNumber = shotNumber + i + ": ";
      SmartDashboard.putNumber(shotNumber, timesToRecover.get(i));
    }

    setSpeed(speed);

    if(RobotContainer.getJoystick().getRawButtonPressed(2)){
      updateSpeed(240);
      if(flyWheelOn = false){
        flyWheelOn = true;
        timer.start();
      }
    }

    if(achievedInitialSpeed == false){
      if(Math.abs(currRPM - speed) < 5){
        achievedInitialSpeed = true;
        initialTime = timer.get();
        timer.stop();
      }
    }

    if(recovery){
      if(Math.abs(currRPM - speed) < 5){
        recovery = false;
        timesToRecover.add(timer.get());
        timer.stop();
      }
    }
    
    checkRecovery(currRPM);

    


    


    /* 
    if(RobotContainer.getJoystick().getRawButtonPressed(2)){
      setSpeed(240);
    }
    */
    // This method will be called once per scheduler run
  }
}
