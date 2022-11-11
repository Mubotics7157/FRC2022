package frc.auton.guiauto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NetworkAuto extends AbstractGuiAuto {

    static final NetworkTableInstance instance = NetworkTableInstance.getDefault();
    static final NetworkTable table = instance.getTable("autodata");
    static final NetworkTableEntry autoPath = table.getEntry("autoPath");

    public NetworkAuto() {
        super(autoPath.getString(null));
    }
}