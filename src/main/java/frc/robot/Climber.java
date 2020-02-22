package frc.robot;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

public class Climber {

    private boolean climbing = false;

    private int climberState = 0;

    private int climbTickCounter = 0;

    // Declare motors/pneumatics/encoder
    private CANSparkMax climberMotors = null;

    private DigitalInput climbLimit = null;

    public Climber(RobotMap robotMap) {
        this.climberMotors = robotMap.getClimberMotors();
        this.climbLimit = robotMap.getClimbLimit();
    }

    public void moveClimber() {
        if (climbing) {
            System.out.println(climberState);
            switch (climberState) {
                //Release climber
                case 0:

                    climberMotors.set(-1.0);

                    if (!climbLimit.get()) {
                        climbTickCounter++;
                    } else {
                        climbTickCounter = 0;
                    }

                    if (climbTickCounter >= 50) {
                        climberState++;
                        climbTickCounter = 0;
                    }
                    break;
                
                //Pulls until limit switch is contacted
                case 1:

                    if (climbLimit.get()) {
                        climbTickCounter++;
                    } else {
                        climbTickCounter = 0;
                    }

                    if (climbTickCounter >= 2) {
                        climberState++;
                    }

                    break;

                //Stops climber
                case 2:

                    climberMotors.set(0);
                    climbing = false;
                    break;

                default:

                    DriverStation.reportError("Invalid climberState of " + climberState, false);
                    break;
                }
        }
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

    public void release(boolean active) {
        if (active) {
            switch (climberState) {
                case 0:

                    climberMotors.set(0.2);
                    if (!climbLimit.get()) {
                        climberState++;
                    }

                    break;

                case 1:

                    if (climbLimit.get()) {
                        climberMotors.set(0.0);
                    }

                default:

                    break;
            }
        } else {
            climberMotors.set(0.0);
            climberState = 0;
        }
    }
}
