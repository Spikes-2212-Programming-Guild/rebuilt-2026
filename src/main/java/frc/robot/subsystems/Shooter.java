package frc.robot.subsystems;

import com.spikes2212.command.genericsubsystem.MotoredGenericSubsystem;
import com.spikes2212.util.smartmotorcontrollers.TalonFXWrapper;
import frc.robot.RobotMap;

public class Shooter extends MotoredGenericSubsystem {

    private static final String NAMESPACE_NAME = "shooter";
    private static Shooter instance;

    private final TalonFXWrapper upperMotor;
    private final TalonFXWrapper lowerMotor;

    public static Shooter getInstance(){
        if(instance == null){
            instance = new Shooter(NAMESPACE_NAME,
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_UPPER_TALON_FX_ID),
                    new TalonFXWrapper(RobotMap.CAN.SHOOTER_LOWER_TALON_FX_ID));
        }
        return instance;
    }

    private Shooter(String namespaceName, TalonFXWrapper upperMotor, TalonFXWrapper lowerMotor) {
        super(namespaceName, upperMotor, lowerMotor);
        this.upperMotor = upperMotor;
        this.lowerMotor = lowerMotor;
    }

    public void calculateAngleToShoot(){
        //@TODO fill this method when i know the physics of it
        //the void is intentional so it gives no error when it doesnt return anything
    }
}
