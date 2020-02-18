package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;

public class Climber {

    private boolean climbing = false;

    private int climberState = 0;

    // TODO: Actually figure out what this value should be - encoder ticks in climb
    private final int CLIMB_DISTANCE = 0;

    // Declare motors/pneumatics/encoder
    private CANSparkMax climberMotors = null;

    //TODO: Add Solenoid when added to hardware
    private DoubleSolenoid climberSolenoid = null;

    private CANEncoder climbEncoder = null;

    //TODO: Manual control of elevator
    private boolean climbButton = false;

    private DigitalInput limitSwitch;

    public Climber(RobotMap robotMap) {
        this.climberMotors = robotMap.getClimberMotors();
        //this.climberSolenoid = robotMap.getClimberSolenoid();
        this.climbEncoder = robotMap.getClimbEncoder();
        this.limitSwitch = robotMap.getClimbLimit();
    }

    public void moveClimber() {
        if (climbing) {
            climberMotors.set(-1.0);
            if(!limitSwitch.get()) {
                climberMotors.set(0);
            }
        }
    }

    public void manualClimb(boolean pulling) {
        climbButton = pulling;
    }


    public void toggleClimb() {
        if (!climbing) {
            climbing = true;
        } else {
            // ABORTS CLIMB AT ANY STAGE OF CLIMB
            climbing = false;
            climberMotors.set(0);
            DriverStation.reportError("CLIMB ABORTED BY DRIVER", false);
        }
    }

    public boolean getClimbing() {
        return climbing;
    }

    public void release() {
        climberMotors.set(0.5);
    }
}
