package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;

public class Climber {

    private boolean climbing = false;

    private int climberState = 0;

    // TODO: Actually figure out what this value should be - encoder ticks in climb
    private final int CLIMB_DISTANCE = 0;

    // Declare motors/pneumatics/encoder
    private CANSparkMax climberMotors = null;

    private DoubleSolenoid climberSolenoid = null;

    private CANEncoder climbEncoder = null;

    //TODO: Manual control of elevator
    private boolean climbButton = false;

    public Climber(RobotMap robotMap) {
        this.climberMotors = robotMap.getClimberMotors();
        //this.climberSolenoid = robotMap.getClimberSolenoid();
        this.climbEncoder = robotMap.getClimbEncoder();
    }

    public void moveClimber() {
        if (true) {
            System.out.println(climberState);
            switch (climberState) {
                //Release Spring
                case 0:

                    //climberSolenoid.set(DoubleSolenoid.Value.kReverse);
                    climberState++;
                    break;
                
                //Pulls until target encoder distance
                case 1:

                    climbEncoder.setPosition(0);
                    climberMotors.set(-1.0);
                    if(false/*climbEncoder.getPosition() <= CLIMB_DISTANCE*/) {
                        climberState++;
                    }

                    break;
                //
                case 2:

                    climberMotors.set(0);
                    //climberSolenoid.set(DoubleSolenoid.Value.kForward);
                    climbing = false;
                    break;

                default:

                    DriverStation.reportError("Invalid climberState of " + climberState, false);
                    break;
                }
        }
    }

    public void manualClimb(boolean pulling) {
        climbButton = pulling;
    }


    public void toggleClimb() {
        if (!climbing) {
            climbing = true;
            moveClimber();
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
}
