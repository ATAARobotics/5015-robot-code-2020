package frc.robot;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.vision.CameraMode;
import frc.robot.vision.LimeLight;

//import frc.robot.pathweaver.PathFinder;

/**
 * A file dedicated to all auto related code
 *
 * @author Alexander Greco and Jacob Guglielmin
 */
public class Auto {

    String autoSelected;

    // TODO Tune PID
    // Turn values
    double Tp = 0.04;
    double Ti = 0.03;
    double Td = 0.007;

    double turn = 0;
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
    List<List<String>> splitAutoCommands = new ArrayList<List<String>>();
    final String root = "./autos";
    final String rev = "5015-2020-rev1";
    String fileName = "./auto/AAAAAA.auto";

    Path path = Paths.get(fileName);

    private int commandNumber = 0;
    private boolean nextCommand = false;
    private boolean targetLock = false;

    private Shooter shooter;
    private LimeLight limeLight;

    private Align alignment;

    private int onTargetCounter = 0;

    public Auto(RobotMap robotMap) {
        this.gyro = robotMap.getGyro();
        this.encoders = robotMap.getDriveEncoders();
        this.swatDrive = robotMap.swatDrive;
        this.shooter = robotMap.shooter;
        this.alignment = robotMap.align;
    }

    /**
     * Function that contains tasks designed to be ran at initalization
     */
    public void AutoInit() {
        encoders.reset();
        gyro.reset();
        limeLight.setCameraMode(CameraMode.Vision);
        // Speed PID
        /* Config the peak and nominal outputs, 12V means full */

        // Turn PID
        turnPID = new PIDController(Tp, Ti, Td);

        turnPID.setTolerance(2.0);
        turnPID.setIntegratorRange(-1.0, 1.0);
        turnPID.setSetpoint(0.0);
        //Remove rev#
        autoCommands.remove(0);
        //auto start
        int start = 1;
        int end = autoCommands.size()-1;
        int i=0;
        //Get the index of the line which defines the selected auto and get the first command based on its location
        outer: for (;i<autoCommands.size();i++) {
            String command = autoCommands.get(i);
            if(command.equals(autoSelected)) {
               start=i+1;
               break outer;
            }
        }
        //Get the index of the line which defines the next auto  and get the last command based on its location
        outer: for (;i<autoCommands.size();i++) {
            String command = autoCommands.get(i);
            if(command.equals(autoSelected)) {
               end=i-1;
               break outer;
            }
        }
        //Split commands into command type and value ex.["m","10"]
        autoCommands = autoCommands.subList(start, end);
        for (String command : autoCommands) {
            List<String> item = Arrays.asList(command.split(" "));
            splitAutoCommands.add(item);
        }
    }

    /**
     * Periodic function that contains tasks that are designed to be ran
     * periodically.
     */
    public void AutoPeriodic() {
        List<String> command = splitAutoCommands.get(commandNumber);
        String commandType = command.get(0);
        double commandValue = Double.parseDouble(command.get(1));
        //Move with encoder value PID
        if(commandType.equals("m")) {
            commandValue = commandValue/12;
            nextCommand = encoders.PID(commandValue);
        }
        //Rotate with gyro value PID
        else if(commandType.equals("r")) {
            //TODO: PID for rotation
            turnPID.calculate(gyro.getAngle(), commandValue);
            nextCommand = turnPID.atSetpoint();
        }
        //Shoot until empty
        else if(commandType.equals("s")) {
            if(!targetLock) {
                if(alignment.atSetpoint()){
                    DriverStation.reportWarning("On target", false);
                    onTargetCounter++;
                    // Once has been on target for 10 counts: Disable PID, Reset Camera Settings
                    if (onTargetCounter > 10) {
                        //Pass target distance to shooter
                        targetLock = true;
                    }
                } else {
                    DriverStation.reportWarning("Not on target", false);
                    //Rotate using values from the limelight
                    swatDrive.arcadeDrive(0.0, alignment.visionAlign());
                }
            } else {
                targetLock = false;
                //Check ballsStored to see if magazine has been emptied
                if(shooter.getBallsStored() != 0) {
                    shooter.setShooterSpeed(alignment.getDistance());
                    shooter.shoot(true);
                }
                else {
                    shooter.shoot(false);
                    nextCommand = true;
                }
            }
        }
        //increment commandNumber after a completed command and run resets
        if(nextCommand) {
            commandNumber++;
            encoders.reset();
            encoders.PID(0);
            nextCommand = false;
        }
        encoders.PID(100);
    }

    /**
     * Function that contains tasks designed to be ran when the robot is disabled.
     */
    public void AutoDisabled() {

    }

    //Get auto selected from dashboard
    public void setAutoMode(String autoMode) {
        autoSelected = autoMode;
    }

    //Get commands from auto file
	public void setAutoCommands(List<String> autoCommands2) {
        autoCommands = autoCommands2;
	}
}
