package frc.robot.oi.devices;

import frc.robot.oi.Actions;

public interface InputDevice {

    void bindActions(Actions actions);

    double getX();

    double getY();

    double getZ();

    String getName();
}
