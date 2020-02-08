package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.cuforge.libcu.Lasershark;

public class RangeFinder {

    private Lasershark lasershark = null;

    public RangeFinder(Lasershark lasershark) {
        this.lasershark = lasershark;
    }
    
    public double getDistance() {
        return lasershark.getDistanceInches();
    }
}