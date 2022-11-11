package frc.auton;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

public  class TwoBall extends AbstractGuiAuto {
    
    public TwoBall(){
        super(new File(Filesystem.getDeployDirectory().getPath()+"/auto/TwoBall.json"));
    }
}
