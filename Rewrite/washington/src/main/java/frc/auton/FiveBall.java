package frc.auton;

import java.io.File;
import java.nio.file.FileSystem;

import edu.wpi.first.wpilibj.Filesystem;
import frc.auton.guiauto.AbstractGuiAuto;

public  class FiveBall extends AbstractGuiAuto {
    
    public FiveBall(){
        super(new File(Filesystem.getDeployDirectory().getPath()+"/auto/FiveBallAuto.json"));
    }
}