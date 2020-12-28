package Autonomous;

public class Line {

    protected Location one;
    protected Location two;

    public Line(Location one, Location two) {
        this.one = one;
        this.two = two;
    }

    public double getLineSlope() {
        double height = two.getY() - one.getY();
        double width = two.getX() - one.getX();
        return Math.atan(height / width);
    }

    public Location[] getLocations() { return new Location[] { one, two }; }

    public Location getClosestLocationOnLine(Location robotLocation) {
        // TODO check this
        double slope = getLineSlope();
        double perpendicularSlope = -1 / slope;

        // don't question my proofs, just test it out
        double xIntersect = (slope * one.getX() - perpendicularSlope * robotLocation.getX() + robotLocation.getY() - one.getY()) / (slope - perpendicularSlope);
        double yIntersect = slope * (xIntersect - one.getX()) + one.getY();

        return new Location(xIntersect, yIntersect);
    }

}
