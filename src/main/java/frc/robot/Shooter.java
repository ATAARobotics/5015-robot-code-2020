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
    RUNNING,
}

enum ShootCase {
    INITIAL,
    COOLDOWN,
    RUNNING_BEFORE_BALL,
    RUNNING_DURING_BALL,
}


/**
 * The shooter class controls the ball intake, storage, and shooter.
 * It will automatically intake, but not too many balls.
 */
public class Shooter {

    private final double beltCircumference = 0.0 * Math.PI; // TODO: Measure belt cercumference
    private final double magazineTicksPerBall = 0.0 / beltCircumference * 7.5; // TODO: Calculate ticks per ball
    private final double intakeSpeed = 1.0;
    private final double shooterSpeed = 0.65; // TODO: Configure shooter speed
    //TODO: Replace these timers with checking motor speeds
    private final double shooterSpeedup = 0.2;
    private final double shooterCooldown = 0.1; 
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

    private Timer shooterTimer = new Timer();

    private double ballsStored = 3;
    private double shooterStartTime = 0.0;
    private IntakeCase intakeCase = IntakeCase.WAITING;
    private ShootCase shootCase = ShootCase.INITIAL;
    private double setPoint = 0;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    private boolean safetyOverride = false;


    /**
     * Constructs a shooter object with the motors for the shooter and the intake/conveyor,
     * as well as limit switches for the intake and shooter.
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
        kP = 0;
        kI = 0;
        kD = 0;
        kIz = 0;

        //Max rpm
        kFF = 5600;
        
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5600;
        shooterController.setP(kP);
        shooterController.setI(kI);
        shooterController.setD(kD);
        shooterController.setIZone(kIz);
        shooterController.setFF(kFF);
        shooterController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
    }

    public void PIDPeriodic() {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        if ((p != kP)) {
            shooterController.setP(p);
            kP = p;
        }
        if ((i != kI)) {
            shooterController.setI(i);
            kI = i;
        }
        if ((d != kD)) {
            shooterController.setD(d);
            kD = d;
        }
        if ((iz != kIz)) {
            shooterController.setIZone(iz);
            kIz = iz;
        }
        if ((ff != kFF)) {
            shooterController.setFF(ff);
            kFF = ff;
        }
        if ((max != kMaxOutput) || (min != kMinOutput)) {
            shooterController.setOutputRange(min, max);
            kMinOutput = min; kMaxOutput = max;
        }

        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("ProcessVariable", shooterEncoder.getVelocity());

    }
    public void shooterPeriodic() {
        setShooter(shooterActive);
        PIDPeriodic();
    }

    private void setIntake(boolean running) {
        if (running) {
            magazineMotor1.set(ControlMode.PercentOutput, -intakeSpeed);
            magazineMotor2.set(ControlMode.PercentOutput,intakeSpeed);
            intakeMotor.set(ControlMode.PercentOutput, -0.5);
        } else {
            magazineMotor1.set(ControlMode.PercentOutput,0.0);
            magazineMotor2.set(ControlMode.PercentOutput,0.0);
            intakeMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }

    /**
     * Sets the shooter on or off, uses shootSpeed for the power if on.
     * 
     * @param running Whether the shooter wheel should be spinning
     */
    private boolean getIntakeDectector() {
          
        if(intakeDetector.getDistance() < 2.0 || intakeDetector.getDistance() > 245.0 && intakeDetector.getDistance() != 0.0){

            return true;

          }else if(intakeDetector.getDistance() == 0.0){
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
     * @param override Overrides the protection limit of five balls, in case the ballsStored variable is incorrect.
     */
    public void intake(boolean override) {
        switch (intakeCase) {
            case WAITING:
                if ((ballsStored < 5 || override) && getIntakeDectector()) {
                    setIntake(true);
                    intakeCase = IntakeCase.RUNNING;
                }

                break;
            case RUNNING:
                if(!getIntakeDectector()) {
                    setIntake(false);
                    ballsStored++;
                    intakeCase = IntakeCase.WAITING;
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
    /* public void shoot(boolean active) {
        if (active) {
            DriverStation.reportWarning(String.format("Shoot Case: %s", shootCase.toString()), false); // TODO: Pretty print the enum value
            switch (shootCase) {
                case INITIAL: // Shooter was not active last tick
                    shooterTimer.reset();
                    shooterTimer.start();
                    shooterActive = true;
                    shooterStartTime = shooterSpeedup;
                    shootCase = ShootCase.COOLDOWN;
                    DriverStation.reportWarning("Switching to cooldown", false); // TODO: Pretty print the enum value

                    break;
                case COOLDOWN: // Shooter speeding up
                    if (shooterTimer.get() > shooterStartTime) {
                        setIntake(true);
                        shootCase = ShootCase.RUNNING_BEFORE_BALL;
                    }

                    break;
                case RUNNING_BEFORE_BALL: // Shooter running, before sensor sees a ball
                    if (getShooterDectector()) {
                        shootCase = ShootCase.RUNNING_DURING_BALL;
                    }

                    break;
                case RUNNING_DURING_BALL: // Shooter running, while sensor is seeing a ball
                    if (!getShooterDectector()) {
                        setIntake(false);
                        ballsStored--;
                        shooterStartTime = shooterCooldown;
                        shootCase = ShootCase.COOLDOWN;
                    }

                    break;
                default:
                    DriverStation.reportError(String.format("Invalid Shoot Case: %s", shootCase.toString()), false); // TODO: Pretty print the enum value
            }
        } else if (shootCase == ShootCase.COOLDOWN ||
            shootCase == ShootCase.RUNNING_BEFORE_BALL ||
            shootCase == ShootCase.RUNNING_DURING_BALL) { // User released the button

            setIntake(false);
            shooterActive = false;
            shooterTimer.stop();

            shootCase = ShootCase.INITIAL;
        }
    } */
    public void toggleOverride() {
        safetyOverride = !safetyOverride;
    }
}
