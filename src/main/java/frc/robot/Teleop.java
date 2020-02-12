package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.CameraMode;
import frc.robot.vision.LimeLight;

public class Teleop {
    // Variables for robot classes
    private SWATDrive driveTrain = null;
    private Encoders encoders = null;
    private OI joysticks = null;
    private LimeLight limeLight = null;
    private Shooter shooter = null;
    //private Climber climber = null;
    private ColorSensor colorSensor = null;
    private RangeFinder rangeFinder = null;

    public boolean PIDEnabled = false;
    public boolean aligning = false;
    private boolean discoOn = false;
    private int onTargetCounter = 0;

    //vision vars
    private PIDController visionAlignPID = null;
    private boolean visionActive = false;
    private boolean climbing = false;
    private double P = 0.05;
    private double I = 0.0026;
    private double D = 0.05;
    private double tolerance = 0.2;

    private double targetHeight = 81;
    private double limelightHeight = 19;
    private double angleToTarget;
    private double limelightAngle = 40;
    private double distanceToWall;


    public Teleop(RobotMap robotMap) {
        //Initialize Classes
        joysticks = new OI();
        this.driveTrain = robotMap.swatDrive;
        // this.encoders = robotMap.getDriveEncoders(); // TODO: Re-enable this
        this.limeLight = robotMap.limeLight;
        this.shooter = robotMap.shooter;
        this.colorSensor = robotMap.colorSensor;
        this.rangeFinder = robotMap.rangeFinder;
        //this.climber = robotMap.climber;
    }

    public void teleopInit() {
        //encoders.reset();

        //Sets up PID
        visionAlignPID = new PIDController(P, I, D);
        visionAlignPID.setTolerance(tolerance);

        // Disable Vision Processing on Limeligh
        limeLight.setCameraMode(CameraMode.Drive);
        SmartDashboard.putNumber("Tolerance", tolerance);
        SmartDashboard.putNumber("Setpoint", visionAlignPID.getSetpoint());

    }

    public void TeleopPeriodic() {
        angleToTarget = limeLight.getTy();
        distanceToWall = (targetHeight-limelightHeight) / Math.tan(Math.toRadians(limelightAngle+angleToTarget));
        SmartDashboard.putNumber("Distance To Wall", distanceToWall);
        SmartDashboard.putNumber("Angle To Target", angleToTarget);
        joysticks.checkInputs();

        if (!climbing) {
            if (joysticks.getOverride()) {
                shooter.toggleOverride();
            }
            shooter.intake(false);
            //boolean shootButton = joysticks.getShoot();
            //shooter.shoot(shootButton);
            shooter.shooterPeriodic();

            if(joysticks.getVisionButton()) {
                visionActive = !visionActive;
                if (visionActive) {
                    onTargetCounter = 0;
                    limeLight.setCameraMode(CameraMode.Vision);
                }else{
                    limeLight.setCameraMode(CameraMode.Drive);
                }
            }

            //SmartDashboard.putNumber("EncoderLeft", encoders.getLeft());
            //SmartDashboard.putNumber("EncoderRight", encoders.getRight());
            //SmartDashboard.putNumber("EncoderLeftDistance", encoders.getLeftDistance());
            //SmartDashboard.putNumber("EncoderRightDistance", encoders.getRightDistance());

            String colorGuess = colorSensor.findColor();
            SmartDashboard.putString("Color", colorGuess);

            // Vision Alignment
            if(visionActive) {
                // Disable Vision if Aligned
                if(visionAlignPID.atSetpoint()){
                    DriverStation.reportWarning("On target", false);
                    onTargetCounter++;
                    // Once has been on target for 10 counts: Disable PID, Reset Camera Settings
                    if (onTargetCounter > 10) {
                        visionActive = false;
                    }
                } else {
                    DriverStation.reportWarning("Not on target", false);
                    //Rotate using values from the limelight
                    drive(0.0, visionAlignPID.calculate(limeLight.getTx(), 0.0), true);

                }
            // If Vision is disabled normal driving and control operations. (AKA Mainly not vision code)
            }else{

                //This is where the robot is driven (disabled during vision)
                drive(joysticks.getXSpeed(), joysticks.getZRotation(), true);

                if(joysticks.getDiscoButton()){
                    discoOn = !discoOn;
                    if(discoOn){
                        limeLight.setCameraMode(CameraMode.Disco);
                    }else{
                        limeLight.setCameraMode(CameraMode.Drive);
                    }
                }
                /*if(joysticks.getGearShift()) {
                    driveTrain.gearShift();
                }*/

                if (joysticks.getSlow()) {
                    driveTrain.slow();
                }
            }
        }

        SmartDashboard.putNumber("Lasershark Distance", rangeFinder.getDistance());

        /* if (joysticks.getClimbButton()) {
            climber.toggleClimb();
        } */
    }

	public void drive(double speedA, double speedB, boolean arcade) {

        if(arcade) {
            driveTrain.arcadeDrive(speedA, speedB);
        }
        else {
            driveTrain.tankDrive(speedA, speedB);
        }
	}

    // -- Vision: --
    // Update tolerance for Vision PID from shuffleboard
    public void updateFromShuffleData(){
        tolerance = SmartDashboard.getNumber("Tolerance", tolerance);
    }

    // -- End Vision --

	public void TestPeriodic() {
        //safe mode
        //driveTrain.gearShiftSafe();
        joysticks.checkInputs();

        driveTrain.arcadeDrive(joysticks.getXSpeed() * driveTrain.getMaxStraightSpeed(), joysticks.getZRotation() * driveTrain.getMaxTurnSpeed());
        if (joysticks.getSlow()) {
            driveTrain.slow();
        }
        else;
    }
    /*public void setDriveScheme(String driveScheme){
        joysticks.setDriveScheme(driveScheme);
    }
    public void setGunnerScheme(String gunnerScheme){
        joysticks.setGunnerScheme(gunnerScheme);
    }*/
}
