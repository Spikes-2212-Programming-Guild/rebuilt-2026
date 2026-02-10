package frc.robot;

import frc.robot.com.spikes2212.util.PlaystationControllerWrapper;

public class OI /*GEVALD*/{

    private final PlaystationControllerWrapper driverPlaystation = new PlaystationControllerWrapper(0);

    public OI() {

    }

    public double getLeftX(){
        return driverPlaystation.getLeftX();
    }

    public double getLeftY(){
        return driverPlaystation.getLeftY();
    }

    public double getRightX(){
        return driverPlaystation.getRightX();
    }

    public double getRightY(){
        return driverPlaystation.getRightY();
    }
}
