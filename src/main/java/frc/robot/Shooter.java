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

public class Shooter {

    //Creates ball magazine encoder object
    private CANEncoder magazineEncoder = null;
    private CANSparkMax magazineMotor = null;
    private CANSparkMax shooter = null;
    private DigitalInput ballDetector = null;

    private double magazineTicksPerInch;

    private double ballsStored = 3;
    private Integer intakeCase = 0;
    private Integer shootCase = 0;

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

    public void reset() {
        magazineEncoder.setPosition(0);
    }

    public void setBallsStored(Double ballsStored) {
        this.ballsStored = ballsStored;
    }

    public void intake() {
        switch (intakeCase) {
            case 0:

                if (ballsStored < 5 && ballDetector.get()) {
                    reset();
                    magazineMotor.set(1.0);
                    intakeCase++;
                }

                break;

            case 1:

                if(getMagazineDistance() > 7.5) {
                    magazineMotor.set(0.0);
                    intakeCase = 0;
                    ballsStored++;
                    reset();
                }

                break;

            default:
                DriverStation.reportError("Invalid intakeCase", false);
                break;
        }
    }

    public void shoot() {
        switch (shootCase) {
            case 0:

                reset();
                shooter.set(0.6);
                magazineMotor.set(1.0);
                shootCase++;

                break;

            case 1:

                if(getMagazineDistance() > 7.5) {
                    magazineMotor.set(0.0);
                    shooter.set(0.0);
                    shootCase = 0;
                    ballsStored--;
                    reset();
                }

            default:
                DriverStation.reportError("Invalid shootCase", false);
                break;
        }
    }

}
