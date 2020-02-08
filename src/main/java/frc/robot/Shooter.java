package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.Math;
import java.net.Socket;
import java.io.PrintWriter;
import java.io.IOException;

/**
 * Ball shooter code
 *
 * @author Jacob Guglielmin, Ben Heard
 */

enum IntakeCase {
    WAITING,
    STARTING,
    RUNNING,
}

enum ShootCase {
    INITIAL,
    WARMUP,
    RUNNING,
}


/**
 * The shooter class controls the ball intake, storage, and shooter.
 * It will automatically intake, but not too many balls.
 */
public class Shooter {

    private final double beltCircumference = 0.0 * Math.PI; // TODO: Measure belt cercumference
    private final double magazineTicksPerBall = 0.0 / beltCircumference * 7.5; // TODO: Calculate ticks per ball
    private final double intakeSpeed = 1.0;
    private final double shooterSpeed = 0.7; // TODO: Configure shooter speed
    private boolean shooterActive = false;

    private CANSparkMax shooterMotor = null;
    private CANEncoder shooterEncoder = null;
    private CANPIDController shooterController = null;
    private VictorSPX magazineMotor1 = null;
    private VictorSPX magazineMotor2 = null;
    private VictorSPX intakeMotor = null;
    private RangeFinder intakeDetector = null;
    private Socket pidSocket = null;
    private PrintWriter pidStream = null;

    private Timer magazineTimer = new Timer();

    private double ballsStored = 0;
    private IntakeCase intakeCase = IntakeCase.WAITING;
    private ShootCase shootCase = ShootCase.INITIAL;
    private double setPoint = 0;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    private boolean safetyOverride = false;


    /**
     * Constructs a shooter object with the motors for the shooter and the intake/conveyor,
     * as well as the lasershark for the intake.
     * @param shooterMotor The motor that shoots balls
     * @param magazineMotor1 The first motor in the elevator
     * @param magazineMotor2 The second motor in the elevator
     * @param intakeMotor The intake motor
     */
    public Shooter(CANSparkMax shooterMotor, VictorSPX magazineMotor1, VictorSPX magazineMotor2, VictorSPX intakeMotor, CANEncoder shooterEncoder, 
        RangeFinder intakeDetector, CANPIDController shooterController) {
        this.shooterMotor = shooterMotor;
        this.magazineMotor1 = magazineMotor1;
        this.magazineMotor2 = magazineMotor2;
        this.intakeMotor = intakeMotor;
        this.shooterEncoder = shooterEncoder;
        this.intakeDetector = intakeDetector;
        this.shooterController = shooterController;

        try {
            this.pidSocket = new Socket("172.22.11.1", 21);
            this.pidStream = new PrintWriter(pidSocket.getOutputStream(), true);
        } catch(IOException ex) {
            DriverStation.reportError(String.format("Error initializing pidStuff: %s", ex.toString()), false);
        }
    }

    /**
     * Sets the intake on or off, uses intakeSpeed for the power if on.
     * @param running Whether the intake wheels should be spinning
     */
    public void PIDInit() {
        // set PID coefficients
        kP = 0.0015;
        kI = 0;
        kD = 0;
        kIz = 0;

        //Max rpm
        kFF = 0.00018;
        
        kMaxOutput = 1;
        kMinOutput = 0;
        maxRPM = 5600;
        shooterController.setP(kP);
        shooterController.setI(kI);
        shooterController.setD(kD);
        shooterController.setIZone(kIz);
        //shooterController.setFF(kFF);
        shooterController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("Feed Forward", kFF);
    }

    public void PIDPeriodic() {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);

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

        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("ProcessVariable", shooterEncoder.getVelocity());

    }
    public void shooterPeriodic() {
        SmartDashboard.putNumber("Balls Stored", ballsStored);
        setShooter(shooterActive);
        PIDPeriodic();
    }

    private void setMagazine(boolean running) {
        if (running) {
            magazineMotor1.set(ControlMode.PercentOutput, -intakeSpeed);
            magazineMotor2.set(ControlMode.PercentOutput,intakeSpeed);
        } else {
            magazineMotor1.set(ControlMode.PercentOutput,0.0);
            magazineMotor2.set(ControlMode.PercentOutput,0.0);
        }
    }

    /**
     * Sets the shooter on or off, uses shootSpeed for the power if on.
     * 
     * @param running Whether the magazine should be moving
     */
    private boolean getIntakeDectector() {
          
        if(intakeDetector.getDistance() < 5.0 && intakeDetector.getDistance() != 0.0){

            return true;

        }else if(intakeDetector.getDistance() == 0.0){

            DriverStation.reportError("Lasershark Disconnected", false);
            return false;

        }else{
            return false;
        }
      
        
    }

    private void setShooter(boolean running) {
        if (running) {
            setPoint = shooterSpeed * maxRPM;
        } else {
            setPoint = 0 * maxRPM;
        }
        shooterController.setReference(setPoint, ControlType.kVelocity);
    }

    /**
     * Sets the amount of balls stored for a user-override.
     */
    public void setBallsStored(int ballsStored) {
        this.ballsStored = ballsStored;
    }

    public void logPID() {
        double velocity = shooterEncoder.getVelocity()/maxRPM * 100;
        pidStream.println(Double.toString(velocity));
    }

    /**
     * Main update loop for intaking balls automatically.
     */
    public void intake() {
        switch (intakeCase) {
            case WAITING:
                if (ballsStored < 5 || safetyOverride) {
                    setMagazine(false);
                    intakeMotor.set(ControlMode.PercentOutput, -0.5);
                } else {
                    setMagazine(false);
                    intakeMotor.set(ControlMode.PercentOutput, 0.0);
                }

                if (getIntakeDectector() && ballsStored != 5) {
                    intakeCase = IntakeCase.STARTING;
                }

                break;

            case STARTING:
                if (ballsStored < 4) {
                    if (getIntakeDectector()) {
                        setMagazine(true);
                    } else {
                        magazineTimer.reset();
                        magazineTimer.start();
                        intakeCase = IntakeCase.RUNNING;
                    }
                } else {
                    intakeCase = IntakeCase.RUNNING;
                }

                break;

            case RUNNING:
                if(ballsStored < 4) {
                    if(magazineTimer.get() < 0.5) {
                        setMagazine(true);
                    } else {
                        ballsStored++;
                        intakeCase = IntakeCase.WAITING;
                    }
                } else {
                    if(magazineTimer.get() < 0.2) {
                        setMagazine(true);
                    } else {
                        ballsStored++;
                        intakeCase = IntakeCase.WAITING;
                    }
                }
                break;

            default:
                DriverStation.reportError(String.format("Invalid Intake Case: %d", intakeCase), false); // TODO: Pretty print the enum value
        }
    }

    /**
     * Main update loop for the shooter, when not active, just shuts off the shooter.
     * @param active Whether the shooter should be shooting.
     */
    public void shoot(boolean active) {
        if (active) {
            DriverStation.reportWarning(String.format("Shoot Case: %s", shootCase.toString()), false); // TODO: Pretty print the enum value
            switch (shootCase) {
                case INITIAL: // Shooter was not active last tick
                    shooterActive = true;
                    shootCase = ShootCase.WARMUP;
                    DriverStation.reportWarning("Switching to cooldown", false); // TODO: Pretty print the enum value

                    break;
                case WARMUP: // Shooter speeding up
                    if (!(shooterEncoder.getVelocity() < setPoint)) {
                        shootCase = ShootCase.RUNNING;
                    }
                    break;

                case RUNNING: // Shooter running
                
                    setMagazine(true);

                    if (shooterEncoder.getVelocity() < (setPoint - 25)) {
                        if (ballsStored != 0) {
                            ballsStored--;
                        }
                        //shootCase = ShootCase.WARMUP;
                    }

                    break;
                default:
                    DriverStation.reportError(String.format("Invalid Shoot Case: %s", shootCase.toString()), false); // TODO: Pretty print the enum value
            }
        } else if (shootCase == ShootCase.WARMUP ||
            shootCase == ShootCase.RUNNING) { // User released the button

            setMagazine(false);
            shooterActive = false;

            shootCase = ShootCase.INITIAL;
        }
    }

    public void toggleOverride() {
        safetyOverride = !safetyOverride;
    }
}
