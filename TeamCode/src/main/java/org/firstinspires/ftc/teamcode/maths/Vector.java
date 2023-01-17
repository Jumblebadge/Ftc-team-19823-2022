package org.firstinspires.ftc.teamcode.maths;

public class Vector {
    public double x;
    public double y;

    public Vector(double x,double y){
        this.x = x;
        this.y = y;
    }

    public Vector rotateVector(Vector vec, double degrees){
        double x = Math.cos(Math.toRadians(degrees)) * vec.x - Math.sin(Math.toRadians(degrees)) * vec.y;
        double y = Math.sin(Math.toRadians(degrees)) * vec.x + Math.cos(Math.toRadians(degrees)) * vec.y;
        return new Vector(x,y);
    }

    public double magnitude(Vector vec){
        return Math.abs(Math.hypot(vec.x,vec.y));
    }

    public Vector addVectors(Vector vec1,Vector vec2){
        double x = vec1.x + vec2.x;
        double y = vec1.y + vec2.y;
        return new Vector(x,y);
    }

    public Vector subtractVectors(Vector vec1,Vector vec2){
        double x = vec1.x - vec2.x;
        double y = vec1.y - vec2.y;
        return new Vector(x,y);
    }

    public double angleBetweenVectors(Vector vec1, Vector vec2){
        return Math.acos((vec1.x*vec2.x+vec1.y*vec2.y)/(Math.sqrt(vec1.x*vec1.x+vec1.y*vec1.y))*(Math.sqrt(vec2.x*vec2.x+vec2.y*vec2.y)));
    }

    public double dotProduct(Vector vec1, Vector vec2){
        return magnitude(vec1)*magnitude(vec2)*Math.cos(angleBetweenVectors(vec1,vec2));
    }

}
