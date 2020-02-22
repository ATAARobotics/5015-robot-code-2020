package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
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
    private Align alignment = null;

    private boolean climbing = false;

    // Vision Control Variables
    private boolean discoOn = false;
    private int onTargetCounter = 0;

    // Vision PID and PID values
    private boolean visionActive = false;

    NetworkTableEntry driveTemp;
    NetworkTableEntry shootTemp;

    public Teleop(RobotMap robotMap) {
        // Initialize Classes
        joysticks = new OI();
        this.driveTrain = robotMap.swatDrive;
        this.encoders = robotMap.getDriveEncoders();
        this.limeLight = robotMap.limeLight;
        this.shooter = robotMap.shooter;
        this.colorSensor = robotMap.colorSensor;
        this.rangeFinder = robotMap.rangeFinder;
        this.climber = robotMap.climber;
        this.alignment = robotMap.align;
    }

    public void teleopInit() {

        String colorGuess = colorSensor.findColor();
        SmartDashboard.putString("Color", colorGuess);
        encoders.reset();

        // Disable Vision Processing on Limelight
        limeLight.setCameraMode(CameraMode.Vision);

    }

    public void TeleopPeriodic() {
        climber.moveClimber();
        SmartDashboard.putNumber("Distance To Wall", alignment.getDistance());
        SmartDashboard.putNumber("Angle To Target", limeLight.getTy());

        joysticks.checkInputs();

        if (!climbing) {
            shooter.setOverride(joysticks.getOverride());
            shooter.intake();
            boolean shootButton = joysticks.getShoot();
            shooter.shoot(shootButton);
            shooter.shooterPeriodic();

            if(joysticks.getReverse()){
                joysticks.setDriveScheme("Reverse");
            }else{
                joysticks.setDriveScheme("Default");
            }

            // When vision button is pressed, toggle vision and CameraMode
            if (joysticks.getVisionButton()) {
                visionActive = !visionActive;
                if (visionActive) {
                    onTargetCounter = 0;
                    limeLight.setCameraMode(CameraMode.Vision);
                } else {
                    limeLight.setCameraMode(CameraMode.Drive);
                }
            }

            SmartDashboard.putNumber("EncoderLeft", encoders.getLeft());
            SmartDashboard.putNumber("EncoderRight", encoders.getRight());
            SmartDashboard.putNumber("EncoderLeftDistance", encoders.getLeftDistance());
            SmartDashboard.putNumber("EncoderRightDistance", encoders.getRightDistance());

            // Vision Alignment
            if(visionActive) {
                // Disable Vision if Aligned
                if(alignment.atSetpoint()){
                    DriverStation.reportWarning("On target", false);
                    onTargetCounter++;
                    // Once has been on target for 10 counts: Disable PID, Reset Camera Settings
                    if (onTargetCounter > 10) {
                        //Shoot at different speeds based on distance from wall
                        //TODO: Determine necessary values.
                        if(alignment.getDistance() > 0 && alignment.getDistance() < 10){
                            //shooter.shoot(true, 0.0);
                        }else if(alignment.getDistance() > 10 && alignment.getDistance() < 20){
                            //shooter.shoot(true, 0.0);
                        }else if(alignment.getDistance() > 20 && alignment.getDistance() < 30){
                            //shooter.shoot(true, 0.0);
                        }else if(alignment.getDistance() > 30 && alignment.getDistance() < 40){
                            //shooter.shoot(true, 0.0);
                        }else if(alignment.getDistance() > 40 && alignment.getDistance() < 50){
                            //shooter.shoot(true, 0.0);
                        }
                        visionActive = false;
                        limeLight.setCameraMode(CameraMode.Drive);
                    }
                } else {
                    DriverStation.reportWarning("Not on target", false);
                    //Rotate using values from the limelight
                    driveTrain.arcadeDrive(0.0, alignment.visionAlign());

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
                if(joysticks.getIntakeToggle()){
                    shooter.toggleIntake();
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

	public void TestPeriodic() {
        joysticks.checkInputs();

        climber.release(joysticks.getClimbRelease());
    }
    public void setDriveScheme(String driveScheme){
        joysticks.setDriveScheme(driveScheme);
    }
    public void setGunnerScheme(String gunnerScheme){
        joysticks.setGunnerScheme(gunnerScheme);
    }
}
