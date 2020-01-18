package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;


import java.lang.Math;

/**
 * Ball magazine code
 *
 * @author Jacob Guglielmin
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

public class Shooter {

    private final double beltCircumference = 0.0 * Math.PI; // TODO: Measure belt cercumference
    private final double magazineTicksPerBall = 0.0 / beltCircumference * 7.5; // TODO: Calculate ticks per ball
    private final double intakeSpeed = 1.0;
    private final double shooterSpeed = 0.6; // TODO: Configure shooter speed
    private final double shooterSpeedup = 0.2; // TODO: Configure shooter startup time
    private final double shooterCooldown = 0.1; // TODO: Configure shooter cooldown time

    private CANSparkMax shooter = null;
    private DigitalInput intakeDetector = null;
    private DigitalInput shootDetector = null;

    private Timer shooterTimer = new Timer();

    private double ballsStored = 3;
    private double shooterTimeToStart = 0.0;
    private IntakeCase intakeCase = IntakeCase.WAITING;
    private ShootCase shootCase = ShootCase.INITIAL;

    public Shooter(CANSparkMax shooter, DigitalInput intakeDetector, DigitalInput shootDetector) {
        this.shooter = shooter;
        this.intakeDetector = intakeDetector;
        this.shootDetector = shootDetector;
    }

    private void setIntake(boolean running) { // TODO: Connect this to the Victor SPX motor (not in WPI lib)
        if (running) {
            // intakeMotor1.set(intakeSpeed)
            // intakeMotor2.set(intakeSpeed)
        } else {
            // intakeMotor1.set(0.0)
            // intakeMotor2.set(0.0)
        }
    }

    private void setShooter(boolean running) {
        if (running) {
            shooter.set(shooterSpeed);
        } else {
            shooter.set(0);
        }
    }

    public void setBallsStored(int ballsStored) {
        this.ballsStored = ballsStored;
    }

    public void intake() {
        switch (intakeCase) {
            case WAITING:
                if (ballsStored < 5 && intakeDetector.get()) {
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

    public void shoot(boolean active) {
        if (active) {
            switch (shootCase) {
                case INITIAL: // Shooter was not active last tick
                    shooterTimer.reset();
                    shooterTimer.start();
                    setShooter(true);
                    shooterTimeToStart = shooterSpeedup;
                    shootCase = ShootCase.COOLDOWN;

                    break;
                case COOLDOWN: // Shooter speeding up
                    if (shooterTimer.get() > shooterCooldown) {
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
                        shooterTimeToStart = shooterCooldown;
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
            shootCase = ShootCase.INITIAL;
        }
    }

}
