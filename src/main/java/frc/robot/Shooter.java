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
    RESETING,
    RUNNING,
}

public class Shooter {

    private final double beltCircumference = 0.0 * Math.PI; // TODO: Measure belt cercumference
    private final double magazineTicksPerBall = 0.0 / beltCircumference * 7.5; // TODO: Calculate ticks per ball
    private final double intakeSpeed = 1.0;
    private final double shooterSpeed = 0.6;

    private CANSparkMax shooter = null;
    private DigitalInput intakeDetector = null;
    private DigitalInput shooterDetector = null;

    private Timer shooterCooldown = new Timer();

    private double ballsStored = 3;
    private IntakeCase intakeCase = IntakeCase.WAITING;
    private ShootCase shootCase = ShootCase.RESETING;

    public Shooter(CANSparkMax shooter, DigitalInput intakeDetector, DigitalInput shooterDetector) {
        this.shooter = shooter;
        this.intakeDetector = intakeDetector;
        this.shooterDetector = shooterDetector;
    }

    private void setIntake(boolean running) { // TODO: Connect this to the Victor SPX motor
        if (running) {
            // intakeMotor1.set(intakeSpeed)
            // intakeMotor2.set(intakeSpeed)
        } else {
            // intakeMotor1.set(0.0)
            // intakeMotor2.set(0.0)
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
        switch (shootCase) {
            default:
                DriverStation.reportError(String.format("Invalid Shoot Case: %d", shootCase), false); // TODO: Pretty print the enum value
        }
    }

}
