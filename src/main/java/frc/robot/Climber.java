package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;

public class Climber {

    private boolean climbing = false;

    private int climberState = 0;
    private int step = 0;

    //TODO: Actually figure out what this value should be - encoder ticks in climb
    private final int CLIMB_DISTANCE = 0;

    //Declare motors/pneumatics/encoder
    private CANSparkMax climberMotors = null;

    private DoubleSolenoid climberSolenoid = null;

    private CANEncoder climbEncoder = null;

    public Climber(RobotMap robotMap) {
        this.climberMotors = robotMap.getClimberMotors();
        this.climberSolenoid = robotMap.getClimberSolenoid();
        this.climbEncoder = robotMap.getClimbEncoder();
    }

    public void moveClimber() {
        if (climbing) {
            switch (climberState) {
                case 0:

                    climberSolenoid.set(DoubleSolenoid.Value.kReverse);
                    climberState++;
                    climbing = false;
                    break;

                case 1:

                    switch (step) {
                        case 0:

                            climberMotors.set(-1.0);
                            step++;
                            climbEncoder.setPosition(0);

                            break;

                        case 1:

                            climberMotors.set(-1.0);
                            if(climbEncoder.getPosition() <= CLIMB_DISTANCE) {
                                climberState++;
                            }

                            break;

                        default:

                            DriverStation.reportError("Invalid climb step of " + step, false);
                            break;
                    }

                    break;

                case 2:

                    climberMotors.set(0);
                    climberSolenoid.set(DoubleSolenoid.Value.kForward);
                    climbing = false;

                default:

                    DriverStation.reportError("Invalid climberState of " + climberState, false);
                    break;
            }
        }
    }

    public void toggleClimb() {
        if (!climbing) {
            climbing = true;
            moveClimber();
        } else {
            //ABORTS CLIMB AT ANY STAGE OF CLIMB
            climbing = false;
            climberMotors.set(0);
            DriverStation.reportError("CLIMB ABORTED BY DRIVER", false);
        }
    }

    public boolean getClimbing() {
        return climbing;
    }
}
