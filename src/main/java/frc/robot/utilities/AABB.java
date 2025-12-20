package frc.robot.utilities;

public record AABB(double x, double y, double width, double height) {

    public boolean intersectingAABB(AABB other) {
        return x - width <= other.x + other.width
                && x + width >= other.x - other.width
                && y - height <= other.y + other.height
                && y + height >= other.y - height;
    }
}
