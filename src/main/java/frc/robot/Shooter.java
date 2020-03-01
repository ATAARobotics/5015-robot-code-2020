package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Ball shooting code
 *
 * @author Jacob Guglielmin, Ben Heard, Alexander Greco
 */

/**
 * The current state the intake is in.
 */
enum IntakeCase {
    WAITING,
    STARTING,
    RUNNING,
    EJECTING_INTAKE,
    EJECTING_MAGAZINE,
    DOWN_STOPPED,
    OFF
}

/**
 * The current state the shooter is in.
 */
enum ShootCase {
    SPEED_UP,
    RUNNING,
    BALL_SHOOTING
}

/**
 * The shooter class controls the ball intake, magazine, and shooter.
 * It will automatically intake, but not too many balls.
 */
public class Shooter {
    private RobotMap robotMap = null;
    private Timer magazineTimer = new Timer();

    private final double magazineSpeed = -0.6;
    private final double magazineFullSpeed = -1.0;
    private final double intakeSpeed = -1.0;
    private double shooterSpeed = 0.85;

    private double setPoint = 0;
    private final double kP = 0.0007;
    private final double kI = 0.0000002;
    private final double kD = 0.1;
    private final double kIz = 0;
    private final double kFF = 0.00021;
    private final double kMinOutput = 0.0;
    private final double kMaxOutput = 1.0;
    private final double maxRPM = 5600.0;

    private int ballsStored = 0;

    private IntakeCase intakeCase = IntakeCase.WAITING;
    private ShootCase shootCase = ShootCase.SPEED_UP;

    public Shooter(RobotMap robotMap) {
        this.robotMap = robotMap;
        PIDInit();
    }

    //// Internal PID Functions ////

    private void PIDInit() {
        robotMap.setShooterPID(kP, kI, kD, kIz, kFF);
        robotMap.setShooterOutputRange(kMinOutput, kMaxOutput);
    }

    private void PIDPeriodic() {
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", kP);
        double i = SmartDashboard.getNumber("I Gain", kI);
        double d = SmartDashboard.getNumber("D Gain", kD);
        double ff = SmartDashboard.getNumber("Feed Forward", kFF);
        shooterSpeed = SmartDashboard.getNumber("Shooter Speed", shooterSpeed);
        robotMap.setShooterPID(p, i, d, kIz, ff);
        robotMap.setShooterOutputRange(kMinOutput, kMaxOutput);

        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("ProcessVariable", robotMap.checkShooterVelocity());
    }

    //// Internal Util Functions ////

    /**
     * Gets wheather there is a ball in the intake.
     */
    private boolean checkIntakeBall() {
        if (robotMap.checkIntakeDetector() < 7.0) {
            if (robotMap.checkIntakeDetector() != 0.0) {
                return true;
            } else {
                DriverStation.reportError("Intake Lasershark Disconnected", false);
            }
        }
        return false;
    }

    /**
     * Gets wheather there is a ball being shot.
     */
    private boolean checkShootingBall() {
        if (robotMap.checkShooterDetector() < 7.0) {
            if (robotMap.checkShooterDetector() != 0.0) {
                return true;
            } else {
                DriverStation.reportError("Shooting Lasershark Disconnected", false);
            }
        }
        return false;
    }

    /**
     * Set the shooter on or off, including the solenoids.
     * @param running Whether the shooter wheel should be spinning
     */
    private void setShooter(boolean running) {
        if (running) {
            robotMap.setShooterOutputRange(kMinOutput, kMaxOutput);
            setPoint = shooterSpeed * maxRPM;
        } else {
            robotMap.setShooterOutputRange(0.0, 0.0);
            setPoint = 0;
        }
        robotMap.setShooterSetPoint(setPoint);
    }

    /**
     * Set the intake on or off, including the solenoids.
     * @param running Whether the intake wheels should be spinning
     */
    private void setIntake(boolean running) {
        setIntakeMotors(running);
        robotMap.setIntakeExtended(running);
    }

    /**
     * Set the intake motors on or off.
     * @param running Whether the intake wheels should be spinning
     */
    private void setIntakeMotors(boolean running) {
        if (running) {
            if (intakeCase == IntakeCase.EJECTING_INTAKE) {
                robotMap.setIntakeMotor(intakeSpeed*-1.0);
            } else {
                robotMap.setIntakeMotor(intakeSpeed);
            }
        } else {
            robotMap.setIntakeMotor(0.0);
        }
    }

    private void setMagazine(boolean running, boolean fullSpeed) {
        double speed = fullSpeed ? magazineFullSpeed : magazineSpeed;
        if (running) {
            if (intakeCase == IntakeCase.EJECTING_MAGAZINE) {
                robotMap.setMagazineMotor(speed*-0.5);
            } else {
                robotMap.setMagazineMotor(speed);
            }
        } else {
            robotMap.setMagazineMotor(0.0);
        }
    }

    private void setMagazine(boolean running) {
        setMagazine(running, false);
    }

    //// Main looping functions ////

    /**
     * Main update loop for intaking balls automatically.
     */
    public void intake() {
        switch (intakeCase) {
            case WAITING:
                if (ballsStored < 5) {
                    setMagazine(false);
                    setIntake(true);
                    if (checkIntakeBall()) {
                        intakeCase = IntakeCase.STARTING;
                    }
                } else {
                    setMagazine(false);
                    setIntake(false);
                }

                break;
            case STARTING:
                setIntake(true);
                if (ballsStored < 4 && checkIntakeBall()) {
                    setMagazine(true);
                } else {
                    magazineTimer.reset();
                    magazineTimer.start();
                    intakeCase = IntakeCase.RUNNING;
                }

                break;
            case RUNNING:
                setIntake(true);
                double waitingTime = 0.1;
                if (ballsStored < 4) {
                    waitingTime = 0.15;
                }
                if(magazineTimer.get() < waitingTime) {
                    setMagazine(true);
                } else {
                    ballsStored++;
                    if (ballsStored > 5) {
                        ballsStored = 5;
                    }
                    intakeCase = IntakeCase.WAITING;
                }

                break;
            case EJECTING_INTAKE:
                setIntake(true);
                setMagazine(false);
                break;
            case EJECTING_MAGAZINE:
                setIntake(false);
                setMagazine(true);
                break;
            case DOWN_STOPPED:
                setIntake(true);
                setIntakeMotors(false);
            case OFF:
                setIntake(false);
                break;
            default:
                DriverStation.reportError(String.format("Invalid Intake Case: %s", intakeCase.toString()), false);
        }
    }

    /**
     * Main update loop for the shooter, when not active, just shuts off the shooter.
     * @param active Whether the shooter should be shooting.
     */
    public void shoot(boolean active) {
        if (active) {
            DriverStation.reportWarning(String.format("Shoot Case: %s", shootCase.toString()), false);
            setShooter(false);
            switch (shootCase) {
                case SPEED_UP: // Shooter speeding up
                    if (robotMap.checkShooterVelocity() >= setPoint) {
                        shootCase = ShootCase.RUNNING;
                    }
                    break;

                case RUNNING: // Shooter running
                    setIntake(true);
                    setMagazine(true, true);
                    if (checkShootingBall()) {
                        shootCase = ShootCase.BALL_SHOOTING;
                    }

                    break;
                case BALL_SHOOTING:
                    if(!checkShootingBall()){
                        shootCase = ShootCase.SPEED_UP;
                        ballsStored--;
                        if (ballsStored < 0) {
                            ballsStored = 0;
                        }
                    }

                    break;
                default:
                    DriverStation.reportError(String.format("Invalid Shoot Case: %s", shootCase.toString()), false);
            }
        } else if (shootCase == ShootCase.SPEED_UP || shootCase == ShootCase.RUNNING) {
            setMagazine(false);
            setShooter(false);

            // Reset shoot case for next time
            shootCase = ShootCase.SPEED_UP;
        }
    }

    public void shooterPeriodic() {
        SmartDashboard.putNumber("Balls Stored", ballsStored);
        PIDPeriodic();
    }

    //// Other Public interfaces ////

    /**
     * Gets the amount of balls currently stored in the magazine.
     */
    public double getBallsStored() {
        return ballsStored;
    }

    /**
     * Sets the amount of balls stored for a user-override.
     */
    public void setBallsStored(int ballsStored) {
        this.ballsStored = ballsStored;
    }

    /**
     * Shoot at different speeds based on distance from wall
     */
    public void setShooterSpeed(double distance) {
        // If distance is 0.0 (manual entry), set speed to the manual speed.
        if(distance != 0.0){
            distance += 17;
            // Sets speed based on distance from wall
            if(distance < 52) {
                shooterSpeed = -1.32 + 0.112*distance + -0.00144*distance*distance;
            } else {
                shooterSpeed = 0.658 + -0.00244*distance + 0.000019*distance*distance;
            }
        } else {
            shooterSpeed = SmartDashboard.getNumber("Shooter Speed", 0.85);
        }
    }

    public void toggleIntake() {
        toggleIntake(false);
    }

    public void toggleIntake(boolean climbing){
        if (climbing) {
            intakeCase = IntakeCase.OFF;
        } else {
            if (intakeCase == IntakeCase.OFF) {
                intakeCase = IntakeCase.WAITING;
            } else {
                intakeCase = IntakeCase.OFF;
            }
        }
    }

    public void toggleIntakeMotors() {
        if (intakeCase == IntakeCase.DOWN_STOPPED) {
            intakeCase = IntakeCase.WAITING;
        } else {
            intakeCase = IntakeCase.DOWN_STOPPED;
        }
    }

    public void ejectIntake() {
        if (intakeCase == IntakeCase.EJECTING_INTAKE) {
            intakeCase = IntakeCase.WAITING;
        } else {
            intakeCase = IntakeCase.EJECTING_INTAKE;
        }
    }

    public void ejectMagazine(boolean ejecting) {
        if (ejecting) {
            intakeCase = IntakeCase.EJECTING_MAGAZINE;
        } else if (intakeCase == IntakeCase.EJECTING_MAGAZINE) {
            intakeCase = IntakeCase.WAITING;
        }
    }
}
