package Autonomous;

public class Rectangle extends Shape {

    public Rectangle() { super(); }
    public Rectangle(double x, double y, double width, double height) { super(x, y, width, height); }

    @Override
    public double getArea() {return width * height; }

    @Override
    public double getPerimeter() { return 2*width + 2*height; }
}
