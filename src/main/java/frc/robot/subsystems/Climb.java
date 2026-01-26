package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.smartmotorcontrollersubsystem.SmartMotorControllerGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.RobotMap;

public class Climb extends SmartMotorControllerGenericSubsystem {

    private static final String NAMESPACE_NAME = "climb";

    private static final boolean LEFT_MOTOR_INVERTED = false;

    private final TalonFXWrapper leftMotor;
    private final TalonFXWrapper rightMotor;
    private final DigitalInput infrared;

    private static Climb instance;

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.CLIMB_TALON_FX_LEFT_ID),
                    new TalonFXWrapper(RobotMap.CAN.CLIMB_TALON_FX_RIGHT_ID),
                    new DigitalInput(RobotMap.DIO.CLIMB_INFRARED),
                    LEFT_MOTOR_INVERTED
            );
        }
        return instance;
    }

    private Climb(String namespaceName, TalonFXWrapper leftMotor,
                  TalonFXWrapper rightMotor, DigitalInput infrared, boolean leftMotorInverted) {
        super(namespaceName, leftMotor);
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.infrared = infrared;
        rightMotor.follow(leftMotor, leftMotorInverted);
        configureDashboard();
    }

    @Override
    public boolean canMove(double speed) {
        return !(infrared.get() && speed > 0);
    }

    @Override
    public void configureDashboard() {
        namespace.putNumber("right velocity", rightMotor::getVelocity);
        namespace.putNumber("left velocity", leftMotor::getVelocity);
        namespace.putBoolean("infrared", infrared::get);
    }
}
