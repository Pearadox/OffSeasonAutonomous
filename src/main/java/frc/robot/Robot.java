/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.subsystems.*;
import frc.robot.pathfollowing.*;
import frc.robot.commands.*;



public class Robot extends TimedRobot {
  public static Drivetrain drivetrain;
  public static IMU gyro;
  public static OI oi;
  public static Follower follower;
  public static Preferences prefs;

  public static String folder = "/home/lvuser/paths/";
  public static boolean isFollowingPath = false;

  Command autonomousCommand;

  @Override
  public void robotInit() {
    follower = new Follower();
    drivetrain = new Drivetrain();
    gyro = new IMU();
    prefs = Preferences.getInstance();

    oi = new OI();

    gyro.zero();

    drivetrain.zeroEncoders();
  }

  @Override
  public void robotPeriodic() {
    drivetrain.updateTrajectory();
    SmartDashboard.putNumber("leftEncoder", drivetrain.getLeftEncoder());
    SmartDashboard.putNumber("rightEncoder", drivetrain.getRightEncoder());
    SmartDashboard.putNumber("Heading", gyro.getYaw());
    SmartDashboard.putNumber("LeftEncoderFeet", drivetrain.getLeftEncoderFeet());
    SmartDashboard.putNumber("RightEncoderFeet", drivetrain.getRightEncoderFeet());
  }
  
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }
  
  @Override
  public void autonomousInit() {

    autonomousCommand = new RtoCL();
  
   
    
    
    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  public void stopAutonomous() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    stopAutonomous();
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void testPeriodic() {
  }
}
