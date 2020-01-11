package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

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
    private CANSparkMax shooter = null;
    private DigitalInput ballDetector = null; 

    private double magazineTicksPerInch;

    private double beltCircumference = 0.0 * Math.PI;

    public Shooter(CANEncoder magazineEncoder, CANSparkMax magazineMotor, CANSparkMax shooter, DigitalInput ballDetector) {

        this.magazineEncoder = magazineEncoder;
        this.shooter = shooter;
        this.ballDetector = ballDetector;

        this.magazineEncoder.setPosition(0);

        //TODO Calculate ticks per inch

        magazineTicksPerInch = 0.0 / beltCircumference;

    }

    public void intake() {
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
}
