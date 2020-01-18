package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.DigitalInput;

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

    //Creates ball magazine encoder object
    private CANEncoder magazineEncoder = null;
    private CANSparkMax magazineMotor = null;
    private CANSparkMax shooter = null;
    private DigitalInput ballDetector = null;

    private final double beltCircumference = 0.0 * Math.PI; // TODO: Measure belt cercumference
    private final double magazineTicksPerBall = 0.0 / beltCircumference * 7.5; // TODO: Calculate ticks per ball
    private final double intakeSpeed = 1.0;
    private final double shooterSpeed = 0.6;

    private double ballsStored = 3;
    private IntakeCase intakeCase = IntakeCase.WAITING;
    private ShootCase shootCase = ShootCase.RESETING;

    public Shooter(CANEncoder magazineEncoder, CANSparkMax magazineMotor, CANSparkMax shooter, DigitalInput ballDetector) {
        this.magazineEncoder = magazineEncoder;
        this.magazineMotor = magazineMotor;
        this.shooter = shooter;
        this.ballDetector = ballDetector;

        this.magazineEncoder.setPosition(0);
    }

    public double getMagazine() {
        return magazineEncoder.getPosition();
    }

    public boolean magazineTestNewBall() {
        return magazineEncoder.getPosition() / magazineTicksPerBall > 1.0;
    }

    public void resetEncoder() {
        magazineEncoder.setPosition(0);
    }

    public void setBallsStored(Double ballsStored) {
        this.ballsStored = ballsStored;
    }

    public void intake() {
        switch (intakeCase) {
            case WAITING:
                if (ballsStored < 5 && ballDetector.get()) {
                    magazineMotor.set(1.0);
                    resetEncoder();
                    intakeCase = IntakeCase.RUNNING;
                }

                break;
            case RUNNING:
                if(magazineTestNewBall()) {
                    magazineMotor.set(0.0);
                    ballsStored++;
                    intakeCase = IntakeCase.WAITING;
                }

                break;
            default:
                DriverStation.reportError(String.format("Invalid Intake Case: %d", intakeCase), false); // TODO: Pretty print the enum value
        }
    }

    public void shoot() {
        switch (shootCase) {
            case RESETING:
                magazineMotor.set(intakeSpeed);
                shooter.set(shooterSpeed);
                resetEncoder();
                shootCase = ShootCase.RUNNING;

                break;
            case RUNNING:
                if(magazineTestNewBall()) {
                    magazineMotor.set(0.0);
                    shooter.set(0.0);
                    ballsStored--;
                    shootCase = ShootCase.RESETING;
                }

                break;
            default:
                DriverStation.reportError(String.format("Invalid Shoot Case: %d", shootCase), false); // TODO: Pretty print the enum value
        }
    }

}
