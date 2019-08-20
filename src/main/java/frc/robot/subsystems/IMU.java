package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class IMU extends Subsystem {

	public static AHRS navx;
	double yawOffset = 0, pitchOffset = 0;
	
	public IMU () {
		navx = new AHRS(SPI.Port.kMXP);
		navx.zeroYaw();
	}
	
	public double getYaw() {
		return navx.getAngle() - yawOffset;
		// return 0;
	}

	public double getPitch() {
		return navx.getRoll() - pitchOffset;
		// return 0;
	}
	
	public void zero() {
		zero(0);
	}

	public void zero(double extraOffset) {
		yawOffset += getYaw() - extraOffset;
		pitchOffset += getPitch();
	}

    public void initDefaultCommand() {
    
    }
}

