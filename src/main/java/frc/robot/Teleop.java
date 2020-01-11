package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.CameraMode;
import frc.robot.vision.LimeLight;

public class Teleop {
    // Vairables for robot classes
    private SWATDrive driveTrain = null;
    private Encoders encoders = null;
    private OI joysticks = null;
    private LimeLight limeLight = null;
    private Shooter shooter = null;

    public boolean PIDEnabled = false;
    public boolean aligning = false;
    private boolean discoOn = false;
    private int onTargetCounter = 0;
    private PIDSubsystem visionAlignPID;
    private boolean visionActive = false;
    private double P = 0.05;
    private double I = 0.0026;
    private double D = 0.05;
    private double tolerance = 0.2;

    /*UltrasonicCode
    private Ultrasonics ultrasonics;
    */

    public Teleop(SWATDrive swatDrive, Encoders encoders, LimeLight limeLight, Shooter shooter) {
        //Initialize Classes
        joysticks = new OI();
        driveTrain = swatDrive;
        this.encoders = encoders;
        this.limeLight = limeLight;
        this.shooter = shooter;
    }
    public void teleopInit() {
        encoders.reset();

        //Sets up PID
        visionAlignPID = new PIDSubsystem("AlignPID", P, I, D) {
            @Override
            protected double returnPIDInput() {return limeLight.getTx(); }
            @Override
            protected void usePIDOutput(double output) { drive(0, output, true);
                //DriverStation.reportWarning("Vision Running " + output, true);
            }
            @Override
            protected void initDefaultCommand() { }
            };
        
        visionAlignPID.setAbsoluteTolerance(tolerance);
        visionAlignPID.getPIDController().setContinuous(false);
        visionAlignPID.setOutputRange(-1,1);
        visionAlignPID.setInputRange(-27, 27);
        // Disable Vision Processing on Limelight
        limeLight.setCameraMode(CameraMode.Drive);
        SmartDashboard.putNumber("Tolerance", tolerance);
        SmartDashboard.putNumber("Setpoint", visionAlignPID.getSetpoint());
    }

    public void TeleopPeriodic() {
        joysticks.checkInputs();
        visionAlignPID.setAbsoluteTolerance(tolerance);
        //drive

        if(joysticks.getVisionButton()) {
            visionActive = !visionActive;
            if (visionActive) {
                onTargetCounter = 0;
            }
        }
        
        SmartDashboard.putNumber("EncoderLeft", encoders.getLeft());
        SmartDashboard.putNumber("EncoderRight", encoders.getRight());
        SmartDashboard.putNumber("EncoderLeftDistance", encoders.getLeftDistance());
        SmartDashboard.putNumber("EncoderRightDistance", encoders.getRightDistance());

        // Vision Alignment
        if(visionActive) {

            // Disable Vision if Aligned
            if(PIDEnabled && visionAlignPID.onTarget()){
                DriverStation.reportWarning("On target", false);
                onTargetCounter++;
                // Once has been on target for 10 counts: Disable PID, Reset Camera Settings
                if (onTargetCounter > 10) {
                    stopAlignPID();
                    limeLight.setCameraMode(CameraMode.Drive);
                    visionActive = false;
                }
            } else {
                //onTargetCounter = 0;
                DriverStation.reportWarning("Not on target", false);
                // Start PID if not started already, vision is enabled, and not aligned
                if(!PIDEnabled){
                    limeLight.setCameraMode(CameraMode.Vision);
                    startAlignPID();
                }
            }
        // If Vision is disabled normal driving and control operations. (AKA Mainly not vision code)
        }else{
            // If PID is enabled but vision is disabled stop vision alignment PID and reset camera settings
            if(PIDEnabled){
                stopAlignPID();
                limeLight.setCameraMode(CameraMode.Drive);
            }

            //This is where the robot is driven (disabled during vision)
            driveTrain.arcadeDrive(joysticks.getXSpeed() * driveTrain.getMaxStraightSpeed(), joysticks.getZRotation() * driveTrain.getMaxTurnSpeed());

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
            else; 
        }    
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
    // Start alignment PID
    public void startAlignPID() {
        visionAlignPID.setSetpoint(0.0);
        visionAlignPID.enable();
        PIDEnabled = true;
    }

    // Stop Alignment PID
    public void stopAlignPID() {
        visionAlignPID.disable();
        PIDEnabled = false;
        
    }
    // Update tolerance for Vision PID from shuffleboard
    public void updateFromShuffleData(){
        tolerance = SmartDashboard.getNumber("Tolerance", tolerance);
    }

    // -- End Vision --

	public void TestPeriodic() {
        //safe mode
        driveTrain.gearShiftSafe();
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