package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Ball shooter code
 *
 * @author Jacob Guglielmin, Ben Heard, Alexander Greco
 */

enum IntakeCase {
    WAITING,
    STARTING,
    RUNNING,
    REVERSE,
    OFF
}

enum ShootCase {
    INITIAL,
    WARMUP,
    RUNNING,
    BALL_SHOOTING
}


/**
 * The shooter class controls the ball intake, storage, and shooter.
 * It will automatically intake, but not too many balls.
 */
public class Shooter {

    //private final double beltCircumference = 0.0 * Math.PI;
    //private final double magazineTicksPerBall = 0.0 / beltCircumference * 7.5;
    private RobotMap robotMap = null;

    private final double magazineSpeed = -0.60;
    private double intakeSpeed = -1.0;
    private double shooterSpeed = 0.85;
    private boolean shooterActive = false;

    private Timer magazineTimer = new Timer();

    private double ballsStored = 0;
    private IntakeCase intakeCase = IntakeCase.WAITING;
    private ShootCase shootCase = ShootCase.INITIAL;
    private double setPoint = 0;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    private boolean safetyOverride = false;
    private boolean intakeToggle = true;


    /**
     * Constructs a shooter object with the motors for the shooter and the intake/conveyor,
     * as well as the lasershark for the intake.
     * @param robotMap The robotMap instance for this robot.
     */
    public Shooter(RobotMap robotMap) {
        this.robotMap = robotMap;
    }

    /**
     * Sets the intake on or off, uses intakeSpeed for the power if on.
     * @param running Whether the intake wheels should be spinning
     */
    public void PIDInit() {
        // set PID coefficients
        kP = 0.0007;
        kI = 0.0000002;
        kD = 0.1;
        kIz = 0;

        //Max rpm
        kFF = 0.00021;

        kMaxOutput = 1;
        kMinOutput = 0;
        maxRPM = 5600;
        robotMap.setShooterPID(kP, kI, kD, kIz, kFF);
        robotMap.setShooterOutputRange(kMinOutput, kMaxOutput);
    }

    public void PIDPeriodic() {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", kP);
        double i = SmartDashboard.getNumber("I Gain", kI);
        double d = SmartDashboard.getNumber("D Gain", kD);
        double ff = SmartDashboard.getNumber("Feed Forward", kFF);
        shooterSpeed = SmartDashboard.getNumber("Shooter Speed", shooterSpeed);
        robotMap.setShooterPID(p, i, d, kIz, ff);
        robotMap.setShooterOutputRange(kMinOutput, kMaxOutput);

        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("ProcessVariable", robotMap.checkShooterVelocity());

    }
    public void shooterPeriodic() {
        SmartDashboard.putNumber("Balls Stored", ballsStored);
        setShooter(shooterActive);
        PIDPeriodic();
        SmartDashboard.putBoolean("Override", safetyOverride);
    }

    private void setMagazine(boolean running) {
        setMagazine(running, magazineSpeed);
    }

    private void setMagazine(boolean running, double speed) {
        if (running) {
            robotMap.setMagazineMotor(speed);
        } else {
            robotMap.setMagazineMotor(0.0);
        }
    }

    /**
     * Gets wheather there is a ball in the intake
     */
    private boolean getIntakeDectector() {
        if (robotMap.checkIntakeDetector() < 5.0 && robotMap.checkIntakeDetector() != 0.0) {
            return true;
        } else if(robotMap.checkIntakeDetector() == 0.0) {
            DriverStation.reportError("Intake Lasershark Disconnected", false);
            return false;
        } else {
            return false;
        }
    }

    private boolean getShootDetector() {
        if (robotMap.checkShooterDetector() < 5.0 && robotMap.checkShooterDetector() != 0.0) {
            return true;
        } else if(robotMap.checkShooterDetector() == 0.0) {
            DriverStation.reportError("Shooting Lasershark Disconnected", false);
            return false;
        } else {
            return false;
        }
    }

    private void setShooter(boolean running) {
        if (running) {
            robotMap.setShooterOutputRange(kMinOutput, kMaxOutput);
            setPoint = shooterSpeed * maxRPM;
        } else {
            setPoint = 0 * maxRPM;
            robotMap.setShooterOutputRange(0.0, 0.0);
        }
        robotMap.setShooterSetPoint(setPoint);
    }

    /**
     * Sets the amount of balls stored for a user-override.
     */
    public void setBallsStored(int ballsStored) {
        this.ballsStored = ballsStored;
    }

    /**
     * Main update loop for intaking balls automatically.
     */

    //Allow code to control intake motor and solenoid
    private void setIntake(boolean running) {
        setIntakeMotors(running);
        if(running) {
            robotMap.setIntakeExtended(true);
        } else {
            robotMap.setIntakeExtended(false);
        }
    }
    private void setIntakeMotors(boolean running){
        if(running) {
            robotMap.setIntakeMotor(intakeToggle ? intakeSpeed : 0.0);
        } else {
            robotMap.setIntakeMotor(0.0);
        }
    }
    public void intake() {
        switch (intakeCase) {
            case WAITING:
                setIntakeSpeed(-1.0);
                if (ballsStored < 5 || safetyOverride) {
                    setMagazine(false);
                    setIntake(true);
                } else {
                    setMagazine(false);
                    setIntakeMotors(false);
                    //intakeCase = IntakeCase.OFF;
                }

                if (getIntakeDectector() && ballsStored != 5) {
                    intakeCase = IntakeCase.STARTING;
                }

                break;

            case STARTING:
                setIntakeSpeed(-1.0);
                if (ballsStored < 4) {
                    if (getIntakeDectector()) {
                        setMagazine(true);
                    } else {
                        magazineTimer.reset();
                        magazineTimer.start();
                        intakeCase = IntakeCase.RUNNING;
                    }
                } else {
                    magazineTimer.reset();
                    magazineTimer.start();
                    intakeCase = IntakeCase.RUNNING;
                }

                break;

            case RUNNING:
                setIntakeSpeed(-1.0);
                if(ballsStored < 4) {
                    if(magazineTimer.get() < 0.15) {
                        setMagazine(true);
                    } else {
                        ballsStored++;
                        intakeCase = IntakeCase.WAITING;
                    }
                } else {
                    if(magazineTimer.get() < 0.1) {
                        setMagazine(true);
                    } else {
                        ballsStored++;
                        intakeCase = IntakeCase.WAITING;
                    }
                }
                break;
            case REVERSE:
                setIntakeSpeed(1.0);
                setIntake(true);
                break;
            case OFF:
                setIntake(false);
                break;
            default:
                DriverStation.reportError(String.format("Invalid Intake Case: %d", intakeCase), false);
        }
    }

    /**
     * Main update loop for the shooter, when not active, just shuts off the shooter.
     * @param active Whether the shooter should be shooting.
     */
    public void shoot(boolean active) {
        if (active) {
            DriverStation.reportWarning(String.format("Shoot Case: %s", shootCase.toString()), false);
            DriverStation.reportWarning("Shooter LZRSHRK Distance: " + robotMap.checkShooterDetector(), false);
            switch (shootCase) {
                case INITIAL: // Shooter was not active last tick
                    shooterActive = true;
                    shootCase = ShootCase.WARMUP;
                    //DriverStation.reportWarning("Switching to cooldown", false);

                    break;
                case WARMUP: // Shooter speeding up
                    if (robotMap.checkShooterVelocity() >= setPoint) {
                        shootCase = ShootCase.RUNNING;
                    }
                    break;

                case RUNNING: // Shooter running
                    setIntakeMotors(true);
                    setMagazine(true, -1.0);
                    setIntake(true);
                    if (robotMap.checkShooterDetector() < 7.0) {
                        shootCase = ShootCase.BALL_SHOOTING;
                    }

                    break;
                case BALL_SHOOTING:
                    if(robotMap.checkShooterDetector() > 7.0){
                        shootCase = ShootCase.WARMUP;
                        ballsStored--;
                        if (ballsStored < 0) {
                            ballsStored = 0;
                        }

                    }
                    break;
                default:
                    DriverStation.reportError(String.format("Invalid Shoot Case: %s", shootCase.toString()), false);
            }
        } else if (shootCase == ShootCase.WARMUP ||
            shootCase == ShootCase.RUNNING) { // User released the button
            setMagazine(false);
            shooterActive = false;

            shootCase = ShootCase.INITIAL;
        }
    }

    public void setOverride(boolean newOverride) {
        safetyOverride = newOverride;
        if (intakeCase == IntakeCase.OFF && safetyOverride == false) {
            intakeCase = IntakeCase.WAITING;
        }
    }

    public double getBallsStored() {
        return ballsStored;
    }
    public void toggleIntake(){
        if(intakeCase != IntakeCase.OFF){
            intakeCase = IntakeCase.OFF;
        }else{
            intakeCase = IntakeCase.WAITING;
        }

    }
    public void toggleIntake(boolean climbing){
        if(climbing){
            intakeCase = IntakeCase.OFF;
        }else{
            if(intakeCase != IntakeCase.OFF){
                intakeCase = IntakeCase.OFF;
            }else{
                intakeCase = IntakeCase.WAITING;
            }
        }
    }
    // Shoot at different speeds based on distance from wall
    // TODO: Please check that this is the correct way to input the formulas
    public void setShooterSpeed(double distance) {
        // If distance is 0.0 (manual entry), set speed to 0.85
        if(distance != 0.0){
            distance += 17;
            // Sets speed based on distance from wall
            if(distance < 52) {
                shooterSpeed = -1.32 + 0.112*distance + -0.00144*distance*distance;
            } else {
                shooterSpeed = 0.658 + -0.00244*distance + 0.0000161*distance*distance;
            }
        } else {
            shooterSpeed = SmartDashboard.getNumber("Shooter Speed", 0.85);
        }
    }
    public void setIntakeSpeed(double speed){
        intakeSpeed = speed;
    }
    public void reverseIntake(){
        if(intakeCase != IntakeCase.OFF){
            if(intakeCase != IntakeCase.REVERSE){
                intakeCase = IntakeCase.REVERSE;
            }else{
                intakeCase = IntakeCase.WAITING;
            }
        }
        System.out.println("INTAKE CASE: " + intakeCase);
    }
    public void toggleIntakeMotors() {
        intakeToggle = !intakeToggle;
    }
}
