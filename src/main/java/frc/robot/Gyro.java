package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

public class Gyro {
    private AHRS navX;

    public Gyro() {
        try {
            // Initializes the navX object on the roboRIO's MXP port and resets it.
            navX = new AHRS(SPI.Port.kMXP);
            navX.reset();
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
    }

    public void reset() {
        navX.reset();
    }

    public double getAngle() {
        return navX.getAngle();
    }
}
