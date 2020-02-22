package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.vision.LimeLight;

public class Align {

    private LimeLight limeLight = null;

    // Variables for limelight distance tracking
    private double P = 0.15;
    private double I = 0.05;
    private double D = 0.12;
    private double tolerance = 0.2;

    //TODO: Tune PID for alignment
    private PIDController visionAlignPID = new PIDController(P, I, D);

    //Variables to get distance to wall. THESE MUST BE ACCURATE.
    private double targetHeight = 89;
    private double limelightHeight = 44;
    private double limelightAngle = 40.8;
    private double distanceToWall = 0;
    private double angleToTarget = 0;

    public Align(RobotMap robotMap){
        this.limeLight = robotMap.limeLight;
        visionAlignPID.setTolerance(tolerance);
    }

    public double visionAlign(){
        return visionAlignPID.calculate(limeLight.getTx(), 0.0);
    }
    public boolean atSetpoint(){
        visionAlignPID.calculate(limeLight.getTx(),0.0);
        return visionAlignPID.atSetpoint();
    }

    public double getDistance(){
        // Calculate distance to wall using limelight.
        angleToTarget = limeLight.getTy();
        distanceToWall = (targetHeight - limelightHeight) / Math.tan(Math.toRadians(limelightAngle + angleToTarget));

        return distanceToWall;
    }
}
