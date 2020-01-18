package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;


import java.lang.Math;

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
    private final double shooterSpeed = 0.6; // TODO: Configure shooter speed
    private final double shooterSpeedup = 0.2; // TODO: Configure shooter startup time
    private final double shooterCooldown = 0.1; // TODO: Configure shooter cooldown time

    private CANSparkMax shooterMotor = null;
    private DigitalInput intakeDetector = null;
    private DigitalInput shootDetector = null;

    private Timer shooterTimer = new Timer();

    private double ballsStored = 3;
    private double shooterStartTime = 0.0;
    private IntakeCase intakeCase = IntakeCase.WAITING;
    private ShootCase shootCase = ShootCase.INITIAL;

    /**
     * Constructs a shooter object with the motors for the shooter and the intake/elevator,
     * as well as limit switches for the intake and shooter.
     * @param shooterMotor The motor that shoots balls
     * @param intakeMotor1 The first motor in the elevator
     * @param intakeMotor1 The second motor in the elevator
     * @param intakeDetector The bool of weather there is a ball ready to be intook
     * @param shootDetector The bool of weather there is a ball being shot
     */
    public Shooter(CANSparkMax shooterMotor, DigitalInput intakeDetector, DigitalInput shootDetector) {
        this.shooterMotor = shooterMotor;
        this.intakeDetector = intakeDetector;
        this.shootDetector = shootDetector;
    }

    /**
     * Sets the intake on or off, uses intakeSpeed for the power if on.
     * @param running Whether the intake wheels should be spinning
     */
    private void setIntake(boolean running) { // TODO: Connect this to the Victor SPX motor (not in WPI lib)
        if (running) {
            // intakeMotor1.set(intakeSpeed)
            // intakeMotor2.set(intakeSpeed)
        } else {
            // intakeMotor1.set(0.0)
            // intakeMotor2.set(0.0)
        }
    }

    /**
     * Sets the shooter on or off, uses shootSpeed for the power if on.
     * @param running Whether the shooter wheel should be spinning
     */
    private void setShooter(boolean running) {
        if (running) {
            shooterMotor.set(shooterSpeed);
        } else {
            shooterMotor.set(0);
        }
    }

    /**
     * Sets the amount of balls stored for a user-override.
     */
    public void setBallsStored(int ballsStored) {
        this.ballsStored = ballsStored;
    }

    /**
     * Main update loop for intaking balls automatically.
     * @param override Overrides the protection limit of five balls, in case the ballsStored variable is incorrect.
     */
    public void intake(boolean override) {
        switch (intakeCase) {
            case WAITING:
                if ((ballsStored < 5 || override) && intakeDetector.get()) {
                    setIntake(true);
                    intakeCase = IntakeCase.RUNNING;
                }

                break;
            case RUNNING:
                if(!intakeDetector.get()) {
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
    public void shoot(boolean active) {
        if (active) {
            switch (shootCase) {
                case INITIAL: // Shooter was not active last tick
                    shooterTimer.reset();
                    shooterTimer.start();
                    setShooter(true);
                    shooterStartTime = shooterSpeedup;
                    shootCase = ShootCase.COOLDOWN;

                    break;
                case COOLDOWN: // Shooter speeding up
                    if (shooterTimer.get() > shooterStartTime) {
                        setIntake(true);
                        shootCase = ShootCase.RUNNING_BEFORE_BALL;
                    }

                    break;
                case RUNNING_BEFORE_BALL: // Shooter running, before sensor sees a ball
                    if (shootDetector.get()) {
                        shootCase = ShootCase.RUNNING_DURING_BALL;
                    }

                    break;
                case RUNNING_DURING_BALL: // Shooter running, while sensor is seeing a ball
                    if (!shootDetector.get()) {
                        setIntake(false);
                        ballsStored--;
                        shooterStartTime = shooterCooldown;
                        shootCase = ShootCase.COOLDOWN;
                    }

                    break;
                default:
                    DriverStation.reportError(String.format("Invalid Shoot Case: %d", shootCase), false); // TODO: Pretty print the enum value
            }
        } else if (shootCase == ShootCase.COOLDOWN ||
            shootCase == ShootCase.RUNNING_BEFORE_BALL ||
            shootCase == ShootCase.RUNNING_DURING_BALL) { // User released the button

            setIntake(false);
            setShooter(false);
            shooterTimer.stop();

            shootCase = ShootCase.INITIAL;
        }
    }

}
