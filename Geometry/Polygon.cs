namespace Nova.Geometry;
using System;
using System.Linq;
using Nova.Numerics;

public class Polygon(Vect[] vertices, Vect position = default)
{
    public Vect[] Vertices { get; private set; } = vertices;
    public Vect Position { get; set; } = position;
    private Num Rotation { get; set; } = 0f;
    public Num Radius => Vertices.Select(v => v.Magnitude()).Max();

    public Vect[] GetAxes()
    {
        Vect[] axes = new Vect[Vertices.Length];
        Vect[] transformedVerts = TransformedVertices();

        for (int i = 0; i < transformedVerts.Length; i++)
        {
            Vect current = transformedVerts[i];
            Vect next = transformedVerts[(i + 1) % transformedVerts.Length];

            // Calculate edge vector
            Vect edge = next - current;
            // Get perpendicular vector (normal)
            Vect normal = new(-edge.y, edge.x);
            axes[i] = Vect.Normalize(normal);
        }

        return axes;
    }

    public (float min, float max) Project(Vect axis)
    {
        if (axis.SqrMagnitude() < 0.0001f)
        { return (0, 0); }

        Vect[] vertices = TransformedVertices();
        float min = float.MaxValue;
        float max = float.MinValue;

        // Project first vertex
        float projection = Vect.Dot(vertices[0], axis);
        min = max = projection;

        // Project remaining vertices
        for (int i = 1; i < vertices.Length; i++)
        {
            projection = Vect.Dot(vertices[i], axis);
            min = Math.Min(min, projection);
            max = Math.Max(max, projection);
        }

        return (min, max);
    }

    public Vect[] TransformedVertices()
    {
        Vect[] transformedVertices = new Vect[Vertices.Length];

        // Cache trigonometric calculations
        Num cos = Num.Cos(Rotation);
        Num sin = Num.Sin(Rotation);

        for (int i = 0; i < Vertices.Length; i++)
        {
            // Translate to origin
            Vect vertex = Vertices[i];

            // Rotate
            Vect rotated = new((vertex.x * cos) - (vertex.y * sin), (vertex.x * sin) + (vertex.y * cos));

            // Translate back and apply position
            transformedVertices[i] = rotated + Position;
        }

        return transformedVertices;
    }

    public Vect GetCentroid()
    {
        float centroidX = 0, centroidY = 0;
        foreach (Vect vertex in Vertices)
        {
            centroidX += vertex.x;
            centroidY += vertex.y;
        }

        return new Vect(centroidX / Vertices.Length, centroidY / Vertices.Length);
    }

    public static double CalculateMomentOfInertia(Vect[] vertices, double mass)
    {
        if (vertices.Length < 3)
        { throw new ArgumentException("Polygon must have at least 3 vertices"); }

        Vect centroid = CalculateCentroid(vertices);
        double totalMomentOfInertia = 0;
        double totalArea = CalculatePolygonArea(vertices);

        // Handle concave polygons by ensuring positive area triangles
        for (int i = 1; i < vertices.Length - 1; i++)
        {
            Vect[] triangle =
            {
                vertices[0],
                vertices[i],
                vertices[i + 1]
            };

            double triangleArea = Math.Abs(CalculateTriangleArea(triangle));
            double triangleMass = mass * (triangleArea / totalArea);

            Vect triangleCentroid = CalculateTriangleCentroid(triangle);
            double triangleMoment = CalculateTriangleMomentOfInertia(triangle, triangleMass);

            double distanceSquared = Vect.DistanceSqr(triangleCentroid, centroid);
            totalMomentOfInertia += (triangleMass * distanceSquared) + triangleMoment;
        }

        return totalMomentOfInertia;
    }

    private static Vect CalculateCentroid(Vect[] vertices)
    {
        double sumX = 0, sumY = 0;
        foreach (Vect vertex in vertices)
        {
            sumX += vertex.x;
            sumY += vertex.y;
        }

        return new Vect((float)(sumX / vertices.Length), (float)(sumY / vertices.Length));
    }

    private static Vect CalculateTriangleCentroid(Vect[] triangle) => new((triangle[0].x + triangle[1].x + triangle[2].x) / 3, (triangle[0].y + triangle[1].y + triangle[2].y) / 3);

    private static double CalculatePolygonArea(Vect[] vertices)
    {
        if (vertices == null || vertices.Length < 3)
        {
            throw new ArgumentException("Invalid polygon: needs at least 3 vertices");
        }

        double area = 0;
        int j = vertices.Length - 1;

        // Shoelace formula (aka surveyor's formula)
        for (int i = 0; i < vertices.Length; j = i++)
        {
            area += (vertices[j].x * vertices[i].y) - (vertices[i].x * vertices[j].y);
        }

        return Math.Abs(area / 2);
    }

    private static double CalculateTriangleArea(Vect[] triangle)
    {
        if (triangle == null || triangle.Length != 3)
        { throw new ArgumentException("Invalid triangle array"); }

        // Using vector cross product for better numerical stability
        Vect v1 = triangle[1] - triangle[0];
        Vect v2 = triangle[2] - triangle[0];

        return Num.Abs((v1.x * v2.y) - (v1.y * v2.x)) / 2;
    }

    private static double CalculateTriangleMomentOfInertia(Vect[] triangle, double triangleMass)
    {
        if (triangle == null || triangle.Length != 3)
        { throw new ArgumentException("Invalid triangle array"); }

        if (triangleMass <= 0)
        { return 0; }

        // Calculate triangle sides
        double a = Vect.Distance(triangle[0], triangle[1]);
        double b = Vect.Distance(triangle[1], triangle[2]);
        double c = Vect.Distance(triangle[2], triangle[0]);

        // Calculate area
        double s = (a + b + c) / 2; // semi-perimeter
        double area = Num.Sqrt(s * (s - a) * (s - b) * (s - c));

        // Moment of inertia about centroid
        // Formula: (mass * (a² + b² + c²)) / 36
        return triangleMass * ((a * a) + (b * b) + (c * c)) / 36.0;
    }

    public Vect GetEdgeAtPoint(Vect point)
    {
        Vect[] vertices = TransformedVertices();
        float minDistSq = float.MaxValue;
        Vect edge = default;

        for (int i = 0; i < vertices.Length; i++)
        {
            Vect v1 = vertices[i];
            Vect v2 = vertices[(i + 1) % vertices.Length];

            // Calculate point-to-line-segment distance squared
            Vect lineVec = v2 - v1;
            Vect pointVec = point - v1;
            Num lineLengthSq = lineVec.SqrMagnitude();

            // Project point onto line segment
            Num t = Num.Max(0, Num.Min(1, Vect.Dot(pointVec, lineVec) / lineLengthSq));
            Vect projection = v1 + (lineVec * t);
            Num distSq = Vect.DistanceSqr(point, projection);

            if (distSq < minDistSq)
            {
                minDistSq = distSq;
                edge = lineVec;
            }
        }

        return edge;
    }
}
