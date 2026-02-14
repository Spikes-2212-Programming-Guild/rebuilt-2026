package frc.robot;

public class RobotMap {

    public interface CAN {
        
        int SHOOTER_UPPER_TALON_FX_ID = -1;
        int SHOOTER_MIDDLE_TALON_FX_ID = -1;
        int SHOOTER_LOWER_TALON_FX_ID = -1;

        int HOOD_SPARK_MAX = -1;
      
        int COLLECTION_SPARK_MAX_ID = -1;
      
        int COLLECTION_MOVEMENT_TALON_FX_ID = -1;

        int SPINNING_MAGAZINE_SPARK_MAX_ID = -1;

        int TRANSPORT_SPARK_MAX_ID = -1;
    }

    public interface DIO {

        int HOOD_ABSOLUTE_ENCODER = -1;
      
        int COLLECTION_MOVEMENT_THROUGH_BORE_ID = -1;
    }

    public interface PWM {

    }

    public interface AIN {

    }

    public interface PCM {

    }
}
