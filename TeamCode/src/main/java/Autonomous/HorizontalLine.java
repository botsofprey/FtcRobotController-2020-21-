package Autonomous;

public class HorizontalLine extends Line {

    public HorizontalLine(Location location, int width) {
        super(location, new Location(location.getX() + width, location.getY()));
    }

    public double getY() { return one.getY(); }

    @Override
    public Location getClosestLocationOnLine(Location robotLocation) {
        return new Location(robotLocation.getX(), one.getY());
    }
}
