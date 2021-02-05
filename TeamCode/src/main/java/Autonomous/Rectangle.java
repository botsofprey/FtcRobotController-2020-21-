package Autonomous;

public class Rectangle extends Shape {

    public Rectangle() { super(Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE, Double.MIN_VALUE); }
    public Rectangle(double x, double y, double width, double height) { super(x, y, width, height); }

    @Override
    public double getArea() {return width * height; }

    @Override
    public double getPerimeter() { return 2*width + 2*height; }

    @Override
    public String toString() {
        return "X: " + this.x + ", Y: " + this.y + ", width: " + this.width + ", height: " + this.height;
    }
}
