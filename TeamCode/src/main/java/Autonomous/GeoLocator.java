package Autonomous;

/**
 * Created by root on 8/21/17.
 */

/*
    An interface for our location based classes
 */
public interface GeoLocator {
    //meant to contain all code necessary to determine location of robot
    Location getLocation();
    void setBaseLocation(Location location);
}
