// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.FeedWheel;
import frc.robot.subsystems.FlyWheel;

public class RobotContainer {
  private static Joystick joystick;
  private final static FeedWheel feedWheel = new FeedWheel();
  private final static FlyWheel flyWheel = new FlyWheel();

  public RobotContainer() {
    joystick = new Joystick(Constants.joystickPort);
    
    configureBindings();
  }

  public FeedWheel getFeedWheel(){
    return feedWheel;
  }

  public FlyWheel getFlyWheel(){
    return flyWheel;
  }

  public static Joystick getJoystick(){
    return joystick;
  }
  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
