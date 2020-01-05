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
    public static Teleop teleop = new Teleop();
    Auto auto = new Auto();
    LimeLight limeLight = new LimeLight();

    //Add variables for the auto selector
    private static final String defaultAuto = "Default";
    private static final String auto2 = "Auto 2";
    private String autoSelected;
    private final SendableChooser<String> autoPicker = new SendableChooser<>();

    
    @Override
    public void robotInit() {
        autoPicker.setDefaultOption("Default Auto", defaultAuto);
        autoPicker.addOption("Auto 2", auto2);
        SmartDashboard.putData("Auto choices", autoPicker);
        teleop.teleopInit();
        teleop.robotMap.getGyro().initializeNavX();
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
    }
    
    @Override
    public void autonomousInit() {
        autoSelected = autoPicker.getSelected();
        auto.setAutoMode(autoSelected);
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + autoSelected);
        auto.AutoInit();
        System.out.println("Enabling auto from robot");
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
