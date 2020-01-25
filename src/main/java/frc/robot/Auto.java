package frc.robot;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
//import frc.robot.pathweaver.PathFinder;

/**
 * A file dedicated to all auto related code
 *
 * @author Alexander Greco and Jacob Guglielmin
 */
public class Auto {

    String autoSelected;

    //TODO Tune PIDs

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

    PIDSubsystem drivePID;
    PIDSubsystem turnPID;

    boolean driveIsEnabled;
    boolean turnIsEnabled;

    // Adjusts motor speeds so that they match
    // private final double LEFT_SPEED_CONSTANT = -0.851;
    // private final double RIGHT_SPEED_CONSTANT = -1;

    Encoders encoders = null;
    SWATDrive swatDrive = null;
    Gyro gyro = null;

    public Auto(RobotMap robotMap) {
        this.gyro = robotMap.getGyro();
        // this.encoders = robotMap.getDriveEncoders(); // TODO: Re-enable this
        this.swatDrive = robotMap.swatDrive;
    }

    /**
     * Function that contains tasks designed to be ran at initalization
     */
    public void AutoInit() {

        //encoders.reset();
        gyro.reset();

        // Speed PID
        drivePID = new PIDSubsystem(Dp, Di, Dd) {

            @Override
            protected void initDefaultCommand() {

            }

            @Override
            protected void usePIDOutput(double output) {
                speed = output;
            }

            @Override
            protected double returnPIDInput() {
                return -encoders.getLeftDistance();
            }

            @Override
            public void enable() {
                // Enable PID
                super.enable();
                // Set enabled variable to true
                driveIsEnabled = true;
            }

            @Override
            public void disable() {
                // Disable PID
                super.disable();
                // Set enabled variable to false
                driveIsEnabled = false;
            }
        };
        drivePID.setAbsoluteTolerance(1.0);
        drivePID.setOutputRange(-1.0, 1.0);

        // Turn PID
        turnPID = new PIDSubsystem(Tp, Ti, Td) {

            @Override
            protected void initDefaultCommand() {

            }

            @Override
            protected void usePIDOutput(double output) {
                turn = output;
            }

            @Override
            protected double returnPIDInput() {
                // GYRO
                return -gyro.getAngle();
            }

            @Override
            public void enable() {
                // Enabled PID
                super.enable();
                // Set enabled variable to true
                turnIsEnabled = true;
            }

            @Override
            public void disable() {
                // Disable PID
                super.disable();
                // Set enabled variable to false
                turnIsEnabled = false;
            }
        };
        turnPID.setAbsoluteTolerance(2.0);
        turnPID.setOutputRange(-1.0, 1.0);
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
        drivePID.disable();
        turnPID.disable();
    }

    public void setAutoMode(String autoMode) {
        autoSelected = autoMode;
    }
}
