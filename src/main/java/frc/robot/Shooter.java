package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
    private final double magazineSpeed = -0.60;
    private double intakeSpeed = -1.0;
    private double shooterSpeed = 0.85;
    private boolean shooterActive = false;
    private CANSparkMax shooterMotor = null;
    private CANEncoder shooterEncoder = null;
    private CANPIDController shooterController = null;
    private VictorSPX magazineMotor = null;
    private VictorSPX intakeMotor = null;
    private RangeFinder intakeDetector = null;
    private RangeFinder shootDetector = null;
    private DoubleSolenoid intakeControl = null;

    private Timer magazineTimer = new Timer();

    private double ballsStored = 0;
    private IntakeCase intakeCase = IntakeCase.WAITING;
    private ShootCase shootCase = ShootCase.INITIAL;
    private double setPoint = 0;
    private boolean shooting = false;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    private boolean safetyOverride = false;


    /**
     * Constructs a shooter object with the motors for the shooter and the intake/conveyor,
     * as well as the lasershark for the intake.
     * @param shooterMotor The motor that shoots balls
     * @param magazineMotor The magazine motor
     * @param intakeMotor The intake motor
     */
    public Shooter(CANSparkMax shooterMotor, VictorSPX magazineMotor, VictorSPX intakeMotor, DoubleSolenoid intakeControl, CANEncoder shooterEncoder,
        RangeFinder intakeDetector, RangeFinder shootDetector, CANPIDController shooterController) {
        this.shooterMotor = shooterMotor;
        this.magazineMotor = magazineMotor;
        this.intakeMotor = intakeMotor;
        this.shooterEncoder = shooterEncoder;
        this.intakeDetector = intakeDetector;
        this.shootDetector = shootDetector;
        this.shooterController = shooterController;
        this.intakeControl = intakeControl;
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
        shooterController.setP(kP);
        shooterController.setI(kI);
        shooterController.setD(kD);
        shooterController.setIZone(kIz);
        shooterController.setFF(kFF);
        shooterController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Shooter Speed", shooterSpeed);
    }

    public void PIDPeriodic() {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        shooterSpeed = SmartDashboard.getNumber("Shooter Speed", 0);
        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if ((i != kI)) {
            shooterController.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            shooterController.setD(d);
            kD = d;
        }
        if ((p != kP)) {
            shooterController.setP(p);
            kP = p;
        }
        if ((ff != kFF)) {
            shooterController.setFF(ff);
            kFF = ff;
        }

        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("ProcessVariable", shooterEncoder.getVelocity());

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
            magazineMotor.set(ControlMode.PercentOutput, speed);
        } else {
            magazineMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }

    /**
     * Sets the shooter on or off, uses shootSpeed for the power if on.
     *
     * @param running Whether the magazine should be moving
     */
    private boolean getIntakeDectector() {

        if (intakeDetector.getDistance() < 5.0 && intakeDetector.getDistance() != 0.0) {

            return true;

        } else if(intakeDetector.getDistance() == 0.0) {

            DriverStation.reportError("Lasershark Disconnected", false);
            return false;

        } else {
            return false;
        }
    }

    private boolean getShootDetector() {

        if (shootDetector.getDistance() < 5.0 && shootDetector.getDistance() != 0.0) {

            return true;

        } else if(shootDetector.getDistance() == 0.0) {

            DriverStation.reportError("Lasershark Disconnected", false);
            return false;

        } else {
            return false;
        }
    }

    private void setShooter(boolean running) {
        if (running) {
            shooterController.setOutputRange(kMinOutput, kMaxOutput);
            setPoint = shooterSpeed * maxRPM;
        } else {
            setPoint = 0 * maxRPM;
            shooterController.setOutputRange(0, 0);
        }
        shooterController.setReference(setPoint, ControlType.kVelocity);
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
        if(running) {
            intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
            intakeControl.set(Value.kForward);
        }

        else {
            intakeMotor.set(ControlMode.PercentOutput, 0.0);
            intakeControl.set(Value.kReverse);
        }
    }
    private void setIntakeMotors(boolean running){
        if(running){
            intakeMotor.set(ControlMode.PercentOutput, intakeSpeed);
        }else{
            intakeMotor.set(ControlMode.PercentOutput, 0.0);
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
            DriverStation.reportWarning("Shooter LZRSHRK Distance: " + shootDetector.getDistance(), false);
            switch (shootCase) {
                case INITIAL: // Shooter was not active last tick
                    shooterActive = true;
                    shootCase = ShootCase.WARMUP;
                    //DriverStation.reportWarning("Switching to cooldown", false);

                    break;
                case WARMUP: // Shooter speeding up
                    if (shooterEncoder.getVelocity() >= setPoint) {
                        shootCase = ShootCase.RUNNING;
                    }
                    break;

                case RUNNING: // Shooter running
                    setIntakeMotors(true);
                    setMagazine(true, -1.0);
                    if (getShootDetector() && !shooting) {
                        if (ballsStored != 0) {
                            shooting = true;
                        }

                        //shootCase = ShootCase.WARMUP;
                    } else if (shootDetector.getDistance() < 7.0) {
                        shootCase = ShootCase.BALL_SHOOTING;
                    }

                    break;
                case BALL_SHOOTING:
                    if(shootDetector.getDistance() > 7.0){
                        shooting = false;
                        shootCase = ShootCase.WARMUP;
                        ballsStored--;

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

    public double getTemperature() {
        return shooterMotor.getMotorTemperature();
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
    //Shoot at different speeds based on distance from wall
    //TODO: Please check that this is the correct way to input the formulas
    public void setShooterSpeed(double distance) {
        double speed = 0.0;
        //If distance is 0.0 (manual entry), sets speed to 0.85
        if(distance != 0.0){
            distance += 17;
            //Sets speed based on distance from wall
            if(distance < 52){
                speed = -1.32 + 0.112*distance + -0.00144*distance*distance;
            }
            else{
                speed = 0.658 + -0.00244*distance + 0.0000161*distance*distance;
            }
        }else{

            speed = 0.85;
        }

            shooterSpeed = speed;
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
    public void resetBalls(){
        ballsStored = 0;
    }
}
