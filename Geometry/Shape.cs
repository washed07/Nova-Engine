using Nova.Numerics;

namespace Nova.Geometry;

public struct Shape
{
    public static Polygon Rect(int width, Num height)
    {
        // Center the rectangle vertices around (0,0)
        float halfWidth = width / 2f;
        float halfHeight = height / 2f;
        Vect[] vertices = 
        [
            new Vect(-halfWidth, -halfHeight), // top left
            new Vect(halfWidth, -halfHeight),  // top right
            new Vect(halfWidth, halfHeight),   // bottom right
            new Vect(-halfWidth, halfHeight)   // bottom left
        ];
        return new Polygon(vertices);
    }

    public static Polygon Triangle(Num sideLength)
    {
        Vect[] vertices = [new Vect(sideLength / 2, 0), new Vect(sideLength, sideLength), new Vect(0, sideLength)];
        return new Polygon(vertices);
    }

    public static Polygon IsoTriangle(Num width, Num height)
    {
        Vect[] vertices = [new Vect(width / 2, 0), new Vect(width, height), new Vect(0, height)];
        return new Polygon(vertices);
    }

    public static Polygon Circle(Num radius, int sides = 16) // Very performance heavy
    {
        Vect[] vertices = new Vect[sides];
        for (int i = 0; i < sides; i++)
        {
            float angle = Num.PI                  * 2                      * i / sides;
            vertices[i] = new Vect(Num.Cos(angle) * radius, Num.Sin(angle) * radius);
        }

        return new Polygon(vertices);
    }
}