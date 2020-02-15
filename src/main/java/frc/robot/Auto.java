package frc.robot;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
//import frc.robot.pathweaver.PathFinder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A file dedicated to all auto related code
 *
 * @author Alexander Greco and Jacob Guglielmin
 */
public class Auto {

    String autoSelected;

    // TODO Tune PIDs

    // Drive values
    double Dp = 0;
    double Di = 0;
    double Dd = 0;

    double speed = 0;

    // Turn values
    double Tp = 0;
    double Ti = 0;
    double Td = 0;

    double turn = 0;

    PIDController drivePID;
    PIDController turnPID;

    boolean driveIsEnabled;
    boolean turnIsEnabled;

    // Adjusts motor speeds so that they match
    // private final double LEFT_SPEED_CONSTANT = -0.851;
    // private final double RIGHT_SPEED_CONSTANT = -1;

    Encoders encoders = null;
    SWATDrive swatDrive = null;
    Gyro gyro = null;
    List<String> autoCommands;
    final String root = "./autos";
    final String rev = "5015-2020-rev1";
    String fileName = "./auto/swatbots.auto";

    Path path = Paths.get(fileName);

    public Auto(RobotMap robotMap) {
        this.gyro = robotMap.getGyro();
        this.encoders = robotMap.getDriveEncoders();
        this.swatDrive = robotMap.swatDrive;
    }

    /**
     * Function that contains tasks designed to be ran at initalization
     */
    public void AutoInit() {

        encoders.reset();
        gyro.reset();

        // Speed PID
        drivePID = new PIDController(Dp, Di, Dd);

        drivePID.setTolerance(1.0);
        drivePID.setIntegratorRange(-1.0, 1.0);

        // Turn PID
        turnPID = new PIDController(Tp, Ti, Td);

        turnPID.setTolerance(2.0);
        turnPID.setIntegratorRange(-1.0, 1.0);
        turnPID.setSetpoint(0.0);

    }

    /**
     * Periodic function that contains tasks that are designed to be ran
     * periodically.
     */
    public void AutoPeriodic() {
        switch (autoSelected) {
            case "Default":

                //Default Auto Code
                //TODO add auto code

                break;

            case "Auto 2":

                //Other auto code

            default:
                break;
        }
    }

    /**
     * Function that contains tasks designed to be ran when the robot is disabled.
     */
    public void AutoDisabled() {
        
    }

    public void setAutoMode(String autoMode) {
        autoSelected = autoMode;
    }
}
