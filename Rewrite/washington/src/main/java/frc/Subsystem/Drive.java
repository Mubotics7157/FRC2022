package frc.Subsystem;

import frc.util.AbstractSubsystem;

public class Drive extends AbstractSubsystem{
    
    public enum DriveState{
        FIELD_ORIENTED,
        ROBOT_ORIENTED,
        VISION,
        AUTO,
        DONE
    }

    private static final DriveState driveState = DriveState.ROBOT_ORIENTED;
    private static final Drive driveInstance = new Drive();


    
    @Override
    public void update() {
        DriveState snapDriveState;
        synchronized(this){
            snapDriveState = driveState;
        }

        switch(snapDriveState){
            case FIELD_ORIENTED:
                break;
            case ROBOT_ORIENTED:
                break;
            case VISION:
                break;
            case AUTO:
                break;
            case DONE:
                break;
        }
    }


}
