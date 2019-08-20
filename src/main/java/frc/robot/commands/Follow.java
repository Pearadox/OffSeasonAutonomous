
package frc.robot.commands;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

import java.util.*;
import frc.robot.pathfollowing.*;

public class Follow extends Command {

  public static boolean practicebot = false;

  double ka = 0.035; 
  double kp = practicebot ? 0.007 : 0.01;
  double kd = 0.0;
  double kh = .7;
  double ka_reverse = 0.035;
  double kp_reverse = 0.1; 
  double kh_reverse = practicebot ? 0.95 : 0.8;
  double ka_rotate = 0.03;
  double kp_rotate = 0.035;
  double kh_rotate = 0.05;
  double kp_forward = 0.125;

  boolean reverse, mirror;
  boolean followRotation;
  boolean followForward;
  String pathName;
  boolean nonexistentPath;
  ArrayList<TPoint> pathL, pathR;
  double startTime;
  double lastTime = 0;
  double lastError_r = 0;
  double lastError_l = 0;
  double startHeading = 0;
  double stopPercentage = 1;

  public Follow(boolean followRotation, boolean reverse, boolean mirror) {
    requires(Robot.drivetrain);

    this.followRotation = followRotation;
    this.reverse = reverse;
    this.mirror = mirror;
    this.stopPercentage = 1;

    // check if follow ka, follow kp, follow kd exist and put them in if they don't
    if (!Preferences.getInstance().containsKey("MP ka")) Preferences.getInstance().putDouble("MP ka", ka);
    if (!Preferences.getInstance().containsKey("MP kp")) Preferences.getInstance().putDouble("MP kp", kp);
    if (!Preferences.getInstance().containsKey("MP kh")) Preferences.getInstance().putDouble("MP kh", kh);
    if (!Preferences.getInstance().containsKey("MP ka_reverse")) Preferences.getInstance().putDouble("MP ka_reverse", ka_reverse);
    if (!Preferences.getInstance().containsKey("MP kh_reverse")) Preferences.getInstance().putDouble("MP kh_reverse", kh_reverse);
    if (!Preferences.getInstance().containsKey("MP kp_reverse")) Preferences.getInstance().putDouble("MP kp_reverse", kp_reverse);
    if (!Preferences.getInstance().containsKey("MP ka_rotate")) Preferences.getInstance().putDouble("MP ka_rotate", ka_rotate);
    if (!Preferences.getInstance().containsKey("MP kh_rotate")) Preferences.getInstance().putDouble("MP kh_rotate", kh_rotate);
    if (!Preferences.getInstance().containsKey("MP kp_rotate")) Preferences.getInstance().putDouble("MP kp_rotate", kp_rotate);
  }

  public Follow(ArrayList<ArrayList<TPoint>> list, boolean followRotation, boolean followForward) {
    this(followRotation, false, false);
    this.followForward = followForward;
    pathL = list.get(0);
    pathR = list.get(1);
  }

  public Follow(String pathName, boolean reverse, boolean mirror) {
    this(pathName, reverse, mirror, 1);
  }

  public Follow(String pathName, boolean reverse, boolean mirror, double stopPercentage) {
    this(false, reverse, mirror);
    this.pathName = pathName;
    this.stopPercentage = stopPercentage;

    nonexistentPath = !Robot.follower.paths.keySet().contains(pathName);

    ArrayList<ArrayList<TPoint>> pathPair = Robot.follower.paths.get(this.pathName);
    pathL = pathPair.get(0);
    pathR = pathPair.get(1);
  }

  @Override
  protected void initialize() {
    if(nonexistentPath) return;
    Robot.isFollowingPath = true;
    Robot.drivetrain.zeroEncoders();
    // Get paths and put them in the TPoint lists
    startTime = Timer.getFPGATimestamp();
    
    startHeading = Math.toRadians(Robot.gyro.getYaw());
    if(stopPercentage <= 0) stopPercentage = 1;

    Robot.prefs = Preferences.getInstance();

    kp = Robot.prefs.getDouble("MP kp", kp);
    ka = Robot.prefs.getDouble("MP ka", ka);
    kh = Robot.prefs.getDouble("MP kh", kh);

    ka_reverse = Robot.prefs.getDouble("MP ka_reverse", ka_reverse);
    kp_reverse = Robot.prefs.getDouble("MP kp_reverse", kp_reverse);
    kh_reverse = Robot.prefs.getDouble("MP kh_reverse", kh_reverse);
    
    ka_rotate = Robot.prefs.getDouble("MP ka_rotate", ka_rotate);
    kp_rotate = Robot.prefs.getDouble("MP kp_rotate", kp_rotate);
    kh_rotate = Robot.prefs.getDouble("MP kh_rotate", kh_rotate);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    try {
      if(nonexistentPath) return;
      // Get all trajectory points
      double currentTime = Timer.getFPGATimestamp();
      double runTime = currentTime - startTime;
      int index = (int)Math.round(runTime / Robot.follower.dt);
      if(index >= pathL.size()) return;

      TPoint targetL = pathL.get(index);
      TPoint targetR = pathR.get(index);
      TPoint currentL = Robot.drivetrain.currentLeftTrajectoryPoint;
      TPoint currentR = Robot.drivetrain.currentRightTrajectoryPoint;
      
      double targetHeading_rad = mirror ? -targetL.heading_rad : targetL.heading_rad;
      double kp = reverse ? this.kp_reverse : this.kp;
      kp = followForward ? this.kp_forward : kp;
      double kh = reverse ? this.kh_reverse  : this.kh;
      double ka = reverse ? this.ka_reverse : this.ka;

      if(followRotation) {
        double differenceTicks = (targetL.position_ft-targetR.position_ft)/RobotMap.feetPerTick;
        targetHeading_rad = differenceTicks / RobotMap.halfTurn * Math.PI;
        ka = this.ka_rotate;
        kp = this.kp_rotate;
        kh = this.kh_rotate;
      }

      double pos_targetL = (reverse^mirror) ? targetR.position_ft : targetL.position_ft;
      double vel_targetL = (reverse^mirror) ? targetR.velocity_ft : targetL.velocity_ft;
      double accel_targetL = (reverse^mirror) ? targetR.acceleration_ft : targetL.acceleration_ft;
      double pos_targetR = (reverse^mirror) ? targetL.position_ft : targetR.position_ft;
      double vel_targetR = (reverse^mirror) ? targetL.velocity_ft : targetR.velocity_ft;
      double accel_targetR = (reverse^mirror) ? targetL.acceleration_ft : targetR.acceleration_ft;

      if(reverse) {
        pos_targetL *= -1;
        vel_targetL *= -1;
        accel_targetL *= -1;
        pos_targetR *= -1;
        vel_targetR *= -1;
        accel_targetR *= -1;
      }
      
      // Calculate the differences
      double start_head_target = pathL.get(0).heading_rad;

      double pos_error_l = pos_targetL - currentL.position_ft;
      double pos_error_r = pos_targetR - currentR.position_ft;
      double head_error = (targetHeading_rad - start_head_target) - (currentL.heading_rad - startHeading);

      head_error %= (2*Math.PI);
      if(head_error > Math.PI) head_error-=2*Math.PI;
      if(head_error < -Math.PI) head_error+=2*Math.PI; 

      double leftOutput = Robot.follower.kv * vel_targetL +
                          ka * accel_targetL +
                          kp * pos_error_l +
                          kd * ((pos_error_l - lastError_l) / 
                            (currentTime - lastTime) - vel_targetL) +
                          kh * head_error;
      double rightOutput = Robot.follower.kv * vel_targetR +
                          ka * accel_targetR +
                          kp * pos_error_r +
                          kd * ((pos_error_r - lastError_r) / 
                            (currentTime - lastTime) - vel_targetL) -
                          kh * head_error;
      Robot.drivetrain.drive(leftOutput, rightOutput);

      // SmartDashboard.putNumber("V", vel_targetL);
      // SmartDashboard.putNumber("A"  , ka*accel_targetL);
      SmartDashboard.putNumber("P",  kp*pos_error_l);
      // SmartDashboard.putNumber("D", kd * ((pos_error_l - lastError_l) / 
      // (currentTime - lastTime) - vel_targetL));
      SmartDashboard.putNumber("H", kh*head_error);

      lastTime = currentTime;
      lastError_r = pos_error_r;
      lastError_l = pos_error_l;

    } catch(Exception e) {e.printStackTrace();}
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(nonexistentPath) return true;
    double runTime = Timer.getFPGATimestamp() - startTime;
    double totalRunTime = pathL.size() * Robot.follower.dt * stopPercentage;
    return runTime >= totalRunTime;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.stop();
    Robot.isFollowingPath = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
