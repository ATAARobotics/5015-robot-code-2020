/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.LimeLight;

public class Robot extends TimedRobot {
    //Create objects to run auto and teleop code
    public Teleop teleop = null;
    Auto auto = null;
    Encoders encoders = null;
    LimeLight limeLight = null;
    RobotMap robotMap = null;
    Shooter shooter = null;
    SWATDrive driveTrain = null;
    Gyro gyro = null;

    //Add variables for the auto selector
    private final String defaultAuto = "Default";
    private final String auto2 = "Auto 2";
    private String autoSelected;
    private final SendableChooser<String> autoPicker = new SendableChooser<>();

    private final String defaultDriverScheme = "Default";
    private final String reverseTurning = "Reverse Turning";
    private String driveSchemeSelected;
    private final SendableChooser<String> driveSchemePicker = new SendableChooser<>();

    private final String defaultGunnerScheme = "Default";
    private final String funMode = "Fun Mode";
    private String gunnerSchemeSelected;
    private final SendableChooser<String> gunnerSchemePicker = new SendableChooser<>();

    private Double ballsStored = 3.0;

    public Robot() {
        robotMap = new RobotMap();
        encoders = robotMap.getDriveEncoder();
        driveTrain = new SWATDrive(robotMap);
        gyro = robotMap.getGyro();
        limeLight = new LimeLight();
        shooter = new Shooter(robotMap.getBallMagazineEncoder(), robotMap.getBallMagazineMotor(), robotMap.getShooter(), robotMap.getBallDetector());
        teleop = new Teleop(driveTrain, encoders, limeLight, shooter);
        auto = new Auto(driveTrain, encoders, gyro);
    }
    
    @Override
    public void robotInit() {
        autoPicker.setDefaultOption("Default Auto", defaultAuto);
        autoPicker.addOption("Auto 2", auto2);
        SmartDashboard.putData("Auto choices", autoPicker);

        autoPicker.setDefaultOption("Default", defaultDriverScheme);
        autoPicker.addOption("Reverse Turning", reverseTurning);
        SmartDashboard.putData("Drive Scheme choices", driveSchemePicker);

        autoPicker.setDefaultOption("Default", defaultGunnerScheme);
        autoPicker.addOption("Fun Mode", funMode);
        SmartDashboard.putData("Gunner Scheme choices", gunnerSchemePicker);

        SmartDashboard.putNumber("Balls Stored", 3.0);

        teleop.teleopInit();
        robotMap.getGyro().initializeNavX();
        auto.AutoInit();
        limeLight.ledOff();
    }

    /**
    * This function is called every robot packet, no matter the mode. Use
    * this for items like diagnostics that you want ran during disabled,
    * autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before
    * LiveWindow and SmartDashboard integrated updating.
    * 
    */
    @Override
    public void robotPeriodic() {

    }

    @Override
    public void disabledInit() {
        super.disabledInit();
        auto.drivePID.disable();
        auto.turnPID.disable();
    }

    @Override
    public void disabledPeriodic() {
        ballsStored = SmartDashboard.getNumber("Balls Stored", 3.0);
        shooter.setBallsStored(ballsStored);
    }
    
    @Override
    public void autonomousInit() {
        autoSelected = autoPicker.getSelected();
        auto.setAutoMode(autoSelected);
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        auto.AutoInit();
    }

    /**
    * This function is called periodically during autonomous.
    */
    @Override
    public void autonomousPeriodic() {
        auto.AutoPeriodic();
    }

    /**
    * This function is called periodically during operator control.
    */
    @Override
    public void teleopInit() {
        auto.AutoDisabled();

        driveSchemeSelected = driveSchemePicker.getSelected();
        teleop.setDriveScheme(driveSchemeSelected);

        gunnerSchemeSelected = gunnerSchemePicker.getSelected();
        teleop.setGunnerScheme(gunnerSchemeSelected);
        teleop.teleopInit();   
    }
    @Override
    public void teleopPeriodic() {
        teleop.TeleopPeriodic();
    }
    
    public void testPeriodic() {
        teleop.TestPeriodic();
    }
}
