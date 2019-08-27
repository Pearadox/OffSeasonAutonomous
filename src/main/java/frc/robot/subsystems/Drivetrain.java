package frc.robot.subsystems;

import frc.robot.pathfollowing.*;
import frc.robot.*;
import frc.robot.commands.*;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Drivetrain extends Subsystem {
//  /* 
	//VictorSPX is motor controller, but Talon is better and costs more and is more loved
	//To save money, two talons are used as masters and Victors are chained to follow
	//Master motors controls slave motors
	//Numbers represent CAN IDs on CAN bus

// */

	//encoders track wheel rotations (ticks)
	//two ports are used because they have two channels because idk, encoders are digital


/*
	Victor left1 = new Victor(0);
	Victor left2 = new Victor(1);
	Victor left3 = new Victor(2);
	Victor right1 = new Victor(3);
	Victor right2 = new Victor(4);
	Victor right3 = new Victor(5);
 */
	
	CANSparkMax left1 = new CANSparkMax(1, MotorType.kBrushless);
	CANSparkMax left2 = new CANSparkMax(3, MotorType.kBrushless);
	CANSparkMax right1 = new CANSparkMax(2, MotorType.kBrushless);
	CANSparkMax right2 = new CANSparkMax(4, MotorType.kBrushless);

	CANEncoder encoderL1 = new CANEncoder(left1);
	CANEncoder encoderL2 = new CANEncoder(left2);
	CANEncoder encoderR1 = new CANEncoder(right1);
	CANEncoder encoderR2 = new CANEncoder(right2);
	
	
	
	
	
	

	double lastFeet_r = 0;
	double lastTime = 0;
	double lastVelocity_r = 0;
	double lastAcceleration_r = 0;
	double lastFeet_l = 0;
	double lastVelocity_l = 0;
	double lastAcceleration_l = 0;
	double limitRotation = .1;
	public TPoint currentLeftTrajectoryPoint;
	public TPoint currentRightTrajectoryPoint;
	
	public Drivetrain() {

		left1.setInverted(true);
		left2.setInverted(true);

		

		// /*
		// rightSlave1.setInverted(true);
		// rightSlave2.setInverted(true);
		// rightMaster.setInverted(true);
		// leftEncoder.setReverseDirection(true);

		// rightMaster.setNeutralMode(NeutralMode.Brake);
		// rightSlave1.setNeutralMode(NeutralMode.Brake);
		// rightSlave2.setNeutralMode(NeutralMode.Brake);
		// leftMaster.setNeutralMode(NeutralMode.Brake);
		// leftSlave1.setNeutralMode(NeutralMode.Brake);
		// leftSlave2.setNeutralMode(NeutralMode.Brake);
		// */
	}

	
	public void arcadeDrive(double forward, double rotate ) {
		drive(-forward+rotate*.1, -forward-rotate*.1);

	}
	
	public void drive(double leftSpeed, double rightSpeed) {
		setLeft(leftSpeed);
		setRight(rightSpeed);
	}
	
	public void setLeft(double leftSpeed) {
		setLeftMotor(1, leftSpeed);
		setLeftMotor(2, leftSpeed );
	}
	
	public void setRight(double rightSpeed) {
		setRightMotor(1, rightSpeed);
		setRightMotor(2, rightSpeed);
	}

	public void setRightMotor(int motor, double speed) {
		if (motor == 1) {
			right1.set(speed);
			right1.clearFaults();
		} else if (motor == 2) {
			right2.set(speed);
			right2.clearFaults();
			SmartDashboard.putNumber("right2",speed);
		}
		// else if(motor == 3) rightSlave2.set(ControlMode.PercentOutput, speed);

	}

	public void setLeftMotor(int motor, double speed) {
		if (motor == 1) {
			left1.set(speed);
			left1.clearFaults();
		} else if (motor == 2) {
			left2.set(speed);
			left2.clearFaults();
			SmartDashboard.putNumber("left2",speed);
		}
		
		// else if(motor == 3) leftSlave2.set(ControlMode.PercentOutput, speed);
	}

	public void stop() {
		drive(0, 0);
	}
	
	public double getLeftEncoder() {
		// return leftEncoder.get();
		return (encoderL1.getPosition() + encoderL2.getPosition())/2;
	}

	public double getRightEncoder() {
		// return rightEncoder.get();
		return (encoderR1.getPosition() + encoderR2.getPosition())/2;
	}
	
	public double getLeftEncoderInches() {
		double gearboxRatio = 4.67;
		double shaftWheelRatio = 26.0/12.0;
		double circumference = 6*Math.PI;
		
		return getLeftEncoder() * circumference / (gearboxRatio * shaftWheelRatio);

		
		// return getLeftEncoder()/ RobotMap.ticksPerRev * 2 * Math.PI * 3;
	}
	
	public double getRightEncoderInches() {
		// return getRightEncoder()/ RobotMap.ticksPerRev * 2 * Math.PI * 3;
		double gearboxRatio = 4.67;
		double shaftWheelRatio = 26.0/12.0;
		double circumference = 6*Math.PI;
		
		return getRightEncoder() * circumference / (gearboxRatio * shaftWheelRatio);
	}

	public double getRightEncoderFeet() {
		return getRightEncoderInches()/12;
	}

	public double getLeftEncoderFeet() {
		return getLeftEncoderInches()/12;
	}

	public double getHeading() {
		return (getLeftEncoder() - getRightEncoder()) / RobotMap.halfTurn*180;
	}
	
	public void zeroEncoders() {
		encoderL1.setPosition(0);
		encoderL2.setPosition(0);
		encoderR1.setPosition(0);
		encoderR2.setPosition(0);
	}

	public double getYawEncoder() {
		// double initialDifference = Robot.drivetrain.getLeftEncoder() - Robot.drivetrain.getRightEncoder();
		// return initialDifference * 360 / 1024;
		return 0;
	}

	public void updateTrajectory(){
		double changeFeet_r = getRightEncoderFeet()-lastFeet_r;
		double changeSeconds = Timer.getFPGATimestamp()-lastTime;
		lastFeet_r = getRightEncoderFeet();
		lastTime = Timer.getFPGATimestamp();
		double velocity_r = changeFeet_r/changeSeconds;
		double changeVelocity_r = lastVelocity_r - lastVelocity_r; 
		double acceleration_r = changeVelocity_r / changeSeconds;
		lastVelocity_r = velocity_r;

		double changeAcceleration_r = acceleration_r - lastAcceleration_r;
		double jerk_r = changeAcceleration_r / changeSeconds;
		lastAcceleration_r = acceleration_r;

		double heading_rad = Math.toRadians(Robot.gyro.getYaw());

		currentRightTrajectoryPoint = new TPoint(getRightEncoderFeet(), velocity_r, acceleration_r, heading_rad);

		double changeFeet = getLeftEncoderFeet()-lastFeet_l;
		lastFeet_l = getLeftEncoderFeet();
		lastTime = Timer.getFPGATimestamp();
		double velocity_l = changeFeet/changeSeconds;
		
		double changeVelocity_l = velocity_l - lastVelocity_l; 
		double acceleration_l = changeVelocity_l / changeSeconds;
		lastVelocity_l = velocity_l;

		double changeAcceleration_l = acceleration_l - lastAcceleration_l;
		double jerk_l = changeAcceleration_l / changeSeconds;
		lastAcceleration_l = acceleration_l;


		currentLeftTrajectoryPoint = new TPoint(getLeftEncoderFeet(), velocity_l, acceleration_l, heading_rad);

	}
	
    public void initDefaultCommand() {
        setDefaultCommand(new DriveWithJoystick());
	}

}