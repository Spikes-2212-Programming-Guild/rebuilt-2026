package frc.robot.oi.devices;

import frc.robot.oi.Actions;

public interface InputDevice {

    void initActions(Actions actions);

    double getX();

    double getY();

    double getZ();

    String getName();
}
