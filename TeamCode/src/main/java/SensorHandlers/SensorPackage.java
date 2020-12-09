package SensorHandlers;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SensorPackage {
//    private List<T> sensors = new ArrayList<>();
    private List<Sensor> sensors = new ArrayList<>();

    public SensorPackage(Sensor... sensors) {
        this.sensors.addAll(Arrays.asList(sensors));
    }

    public <T> T getSensor(Class<? extends T> classOrInterface, int id) {
        for(Sensor s : sensors) {
            if(classOrInterface.isInstance(s) && s.getId() == id) return classOrInterface.cast(s);
        }
        return null;
    }

    public <T> T getSensor(Class<? extends T> classOrInterface, String name) {
        for(Sensor s : sensors) {
            if(classOrInterface.isInstance(s) && s.getName().equals(name)) return classOrInterface.cast(s);
        }
        return null;
    }

    public void kill() {
        for(Sensor s : sensors) {
            s.kill();
        }
    }
}
