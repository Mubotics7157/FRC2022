package frc.auton;

import java.io.File;
import java.nio.file.FileSystem;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

public  class ThreeBall extends AbstractGuiAuto {
    
    public ThreeBall(){
        super(new File(Filesystem.getDeployDirectory().getPath()+"/auto/ThreeBall.json"));
    }
}
