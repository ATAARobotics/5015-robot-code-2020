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
    private Climber climber = null;
    private ColorSensor colorSensor = null;
    private RangeFinder rangeFinder = null;

    private boolean climbing = false;

    //Vision Control Variables
    private boolean discoOn = false;
    private int onTargetCounter = 0;

    //Vision PID and PID values
    private PIDController visionAlignPID = null;
    private boolean visionActive = false;
    //TODO: Tune vision PID values
    private double P = 0.0;
    private double I = 0.0;
    private double D = 0.0;
    private double tolerance = 0.2;

    //Variables for limelight distance tracking
    private double targetHeight = 89;
    private double limelightHeight = 19;
    private double limelightAngle = 50;
    private double distanceToWall;
    private double angleToTarget;


    public Teleop(RobotMap robotMap) {
        //Initialize Classes
        joysticks = new OI();
        this.driveTrain = robotMap.swatDrive;
        this.encoders = robotMap.getDriveEncoders();
        this.limeLight = robotMap.limeLight;
        this.shooter = robotMap.shooter;
        this.colorSensor = robotMap.colorSensor;
        this.rangeFinder = robotMap.rangeFinder;
        this.climber = robotMap.climber;
    }

    public void teleopInit() {
        encoders.reset();

        //Sets up PID
        visionAlignPID = new PIDController(P, I, D);
        visionAlignPID.setTolerance(tolerance);

        // Disable Vision Processing on Limelight
        limeLight.setCameraMode(CameraMode.Drive);
        SmartDashboard.putNumber("Tolerance", tolerance);
        SmartDashboard.putNumber("Setpoint", visionAlignPID.getSetpoint());

    }

    public void TeleopPeriodic() {

        //Calculate distance to wall using limelight.
        angleToTarget = limeLight.getTy();
        distanceToWall = (targetHeight-limelightHeight) / Math.tan(Math.toRadians(limelightAngle+angleToTarget));
        SmartDashboard.putNumber("Distance To Wall", distanceToWall);
        SmartDashboard.putNumber("Angle To Target", angleToTarget);

        joysticks.checkInputs();

        if (!climbing) {
            if (joysticks.getOverride()) {
                shooter.toggleOverride();
            }
            shooter.intake();
            boolean shootButton = joysticks.getShoot();
            shooter.shoot(shootButton);
            shooter.shooterPeriodic();

            //When vision button is pressed, toggle vision and CameraMode
            if(joysticks.getVisionButton()) {
                visionActive = !visionActive;
                if (visionActive) {
                    onTargetCounter = 0;
                    limeLight.setCameraMode(CameraMode.Vision);
                }else{
                    limeLight.setCameraMode(CameraMode.Drive);
                }
            }

            SmartDashboard.putNumber("EncoderLeft", encoders.getLeft());
            SmartDashboard.putNumber("EncoderRight", encoders.getRight());
            SmartDashboard.putNumber("EncoderLeftDistance", encoders.getLeftDistance());
            SmartDashboard.putNumber("EncoderRightDistance", encoders.getRightDistance());

            SmartDashboard.putNumber("Drivetrain Temperature", driveTrain.getTemperature());
            SmartDashboard.putNumber("Shooter Temperature", shooter.getTemperature());

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
                    driveTrain.arcadeDrive(0.0, visionAlignPID.calculate(limeLight.getTx(), 0.0));

                }
            // If Vision is disabled normal driving and control operations. (AKA Mainly not vision code)
            }else{

                //This is where the robot is driven (disabled during vision)
				driveTrain.arcadeDrive(joysticks.getXSpeed(), joysticks.getZRotation());

                if(joysticks.getDiscoButton()){
                    discoOn = !discoOn;
                    if(discoOn){
                        limeLight.setCameraMode(CameraMode.Disco);
                    }else{
                        limeLight.setCameraMode(CameraMode.Drive);
                    }
                }
                if(joysticks.getGearShift()) {
                    driveTrain.gearShift();
                }

                if (joysticks.getSlow()) {
                    driveTrain.slow();
                }
            }
        }

        SmartDashboard.putNumber("Lasershark Distance", rangeFinder.getDistance());

        if (joysticks.getClimbButton()) {
            climber.toggleClimb();
        }

        climber.manualClimb(joysticks.getManualClimb());
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
    public void setDriveScheme(String driveScheme){
        joysticks.setDriveScheme(driveScheme);
    }
    public void setGunnerScheme(String gunnerScheme){
        joysticks.setGunnerScheme(gunnerScheme);
    }
}
