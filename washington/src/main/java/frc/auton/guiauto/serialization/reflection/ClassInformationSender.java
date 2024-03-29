package frc.auton.guiauto.serialization.reflection;

import edu.wpi.first.wpilibj.Filesystem;
import frc.util.Serializer;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static frc.auton.guiauto.serialization.reflection.ReflectionUtils.findClasses;
public final class ClassInformationSender {
    public ClassInformationSender() {

    }

    public static void updateReflectionInformation(File file) {
        try {
            List<Class<?>> classes = findClasses(new File(Filesystem.getLaunchDirectory() + "/build/classes/java/main"), "");
            ArrayList<ReflectionClassData> reflectionClassData = new ArrayList<>();
            for (Class<?> aClass : classes) {
                reflectionClassData.add(new ReflectionClassData(aClass));
            }
            System.out.println(Serializer.serializeToString(reflectionClassData));


            if(file != null) {
                file.getParentFile().mkdir();
                Serializer.serializeToFile(reflectionClassData, file);
            }
        } catch (ClassNotFoundException | IOException e) {
            System.out.println("------------------------------------------------------------------------------------------------------------------------");
            e.printStackTrace();
        }
    }


}
