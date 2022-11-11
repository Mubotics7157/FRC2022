package frc.auton;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

public  class WeakSide extends AbstractGuiAuto {
    
    public WeakSide(){
        super(new File(Filesystem.getDeployDirectory().getPath()+"/auto/WeakSide.json"));
    }
}

