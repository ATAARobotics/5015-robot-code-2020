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

    private double magazineTicksPerInch;

    private double ballsStored = 3;
    private IntakeCase intakeCase = IntakeCase.WAITING;
    private ShootCase shootCase = ShootCase.RESETING;

    private double beltCircumference = 0.0 * Math.PI;

    public Shooter(CANEncoder magazineEncoder, CANSparkMax magazineMotor, CANSparkMax shooter, DigitalInput ballDetector) {

        this.magazineEncoder = magazineEncoder;
        this.magazineMotor = magazineMotor;
        this.shooter = shooter;
        this.ballDetector = ballDetector;

        this.magazineEncoder.setPosition(0);

        //TODO Calculate ticks per inch

        magazineTicksPerInch = 0.0 / beltCircumference;

    }

    public double getMagazine() {
        return magazineEncoder.getPosition();
    }

    public double getMagazineDistance() {
        return (magazineEncoder.getPosition()) / magazineTicksPerInch;
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
                    resetEncoder();
                    magazineMotor.set(1.0);
                    intakeCase = IntakeCase.RUNNING;
                }

                break;
            case RUNNING:
                if(getMagazineDistance() > 7.5) {
                    resetEncoder();
                    magazineMotor.set(0.0);
                    intakeCase = IntakeCase.WAITING;
                    ballsStored++;
                }

                break;
            default:
                DriverStation.reportError(String.format("Invalid Intake Case %d", intakeCase), false);
                break;
        }
    }

    public void shoot() {
        switch (shootCase) {
            case RESETING:
                resetEncoder();
                shooter.set(0.6);
                magazineMotor.set(1.0);
                shootCase = ShootCase.RUNNING;

                break;
            case RUNNING:
                if(getMagazineDistance() > 7.5) {
                    resetEncoder();
                    magazineMotor.set(0.0);
                    shooter.set(0.0);
                    shootCase = ShootCase.RESETING;
                    ballsStored--;
                }

                break;
            default:
                DriverStation.reportError(String.format("Invalid Shoot Case %d", shootCase), false);
                break;
        }
    }

}
