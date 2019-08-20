package frc.robot.pathfollowing;

public class TPoint {

    public double position_ft, velocity_ft, acceleration_ft, heading_rad;
    
        public TPoint(double position_ft, double velocity_ft, double acceleration_ft, double heading_rad){
    
           this.position_ft = position_ft;
           this.velocity_ft = velocity_ft;
           this.acceleration_ft = acceleration_ft;
           this.heading_rad = heading_rad;
        }
    }