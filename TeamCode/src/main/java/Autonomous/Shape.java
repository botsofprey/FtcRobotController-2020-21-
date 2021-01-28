package Autonomous;

public abstract class Shape {
    public double x, y;
    public double top, right, bottom, left;
    public double width, height;

    public Shape() {}
    public Shape(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public Shape(double x, double y, double width, double height) {
        this(x, y);
        this.width = width;
        this.height = height;
        calculateSides();
    }
    public abstract double getArea();
    public abstract double getPerimeter();
    private void calculateSides() {
        top = y - (height / 2.0);
        bottom = y + (height / 2.0);
        left = x - (width / 2.0);
        right = x + (width / 2.0);
    }
}
