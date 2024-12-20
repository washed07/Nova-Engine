namespace Nova.Geometry;
#nullable enable
using System.Collections.Generic;
using System.Linq;
using Nova.Numerics;
using Nova.Bodies;
using System;


public static class Collision
{
    /*
        * The Separating Axis Theorem (SAT) is a method used to determine if two convex shapes are colliding.
        * The theorem states that if there exists an axis along which the projections of the two shapes do not overlap, the shapes are not colliding.
        * If all axes overlap, the shapes are colliding.
        * The Minimum Translation Vector (MTV) is the smallest vector required to separate the two shapes.

        * The SAT is used in the 'CheckSat' method to determine if two shapes are colliding.
        * The 'GetMtv' method is used to get the MTV of two shapes.

        * All collision detection code is free use and can be modified to suit your needs, credits are not required. NO HELP WILL BE PROVIDED.
    */
    public class Mtv // Minimum Translation Vector
        (Vect axis, float overlap)
    {
        public Vect Axis { get; set; } = axis;
        public float Depth { get; set; } = overlap;
    }

    public static Mtv? GetMtv(Vect[] verts1, Vect[] verts2) // Separating Axis Theorem, returns the MTV (Minimum Translation Vector)
    { // NOTE: The vertices passed through the method should be in world space, not local space. Transform the vertices before passing them through the method.
        // More information on the Separating Axis Theorem found in the Method 'CheckSat'.


        Vect direction = GetCenter(verts2) - GetCenter(verts1); // Get the direction vector between the two shapes
        
        Vect[] axes = [.. GetAxes(verts1), .. GetAxes(verts2)];
        
        float minOverlap = float.MaxValue; // Set the min overlap to the highest possible value to ensure it is overwritten.
        Vect minAxis = Vect.Zero; // Set the min axis to the zero vector to ensure it is overwritten.
        
        foreach (Vect axis in axes)
        {
            (float min1, float max1) = Project(axis, verts1);
            (float min2, float max2) = Project(axis, verts2);

            if (min1 > max2 || min2 > max1)
                return null;

            float overlap = Math.Min(max1 - min2, max2 - min1); // Get the overlap of the projections
            if (overlap < minOverlap) // If the overlap is less than the current min overlap, set the min overlap and axis to the current values
            {
                minOverlap = overlap;
                minAxis = axis;
                if (Vect.Dot(direction, axis) < 0) // If the direction is opposite to the axis, negate the axis. ensures the axis points towards the other shape.
                    minAxis = -axis;
            }
        }

        //DebugSAT(verts1, verts2, GetCenter(verts1), GetCenter(verts2)); // Uncomment to debug the SAT method

        return new Mtv(minAxis, minOverlap); // Return the MTV (Minimum Translation Vector)
    }

    public static bool CheckSat(Vect[] vertsA, Vect[] vertsB) // Separating Axis Theorem, does not return the MTV (Minimum Translation Vector)
    { // NOTE: The vertices passed through the method should be in world space, not local space. Transform the vertices before passing them through the method.
        /*
            * The Separating Axis Theorem (SAT) is a method used to determine if two convex shapes are colliding.
            * The theorem states that if there exists an axis along which the projections of the two shapes do not overlap, the shapes are not colliding.
            * If all axes overlap, the shapes are colliding.
            * The Minimum Translation Vector (MTV) is the smallest vector required to separate the two shapes.

            * The SAT is used in the 'CheckSat' method to determine if two shapes are colliding.
            * The 'GetMtv' method is used to get the MTV of two shapes.
        */

        // Get all axes to test
        Vect[] axes = [.. GetAxes(vertsA), .. GetAxes(vertsB)];
        
        foreach (Vect axis in axes)
        {     
            (float min1, float max1) = Project(axis, vertsA); // Project the vertices of the shapes onto the axis
            (float min2, float max2) = Project(axis, vertsB);

            if (min1 > max2 || min2 > max1) // If the projections do not overlap, the shapes are not colliding
                return false;
        }

        return true; // Return true if all axes overlap
    }

    public static Vect[] GetAxes(Vect[] vertices) // Gets each vector perpendicular to an edge of the shape
    {
        var axes = new List<Vect>(); // Initialize for use

        for (int i = 0; i < vertices.Length; i++) // Loop through each vertex and get the next vertex to create an edge. Always loop back to the first vertex to close the shape.
        {
            Vect current = vertices[i]; //  The current vertex of the iteration
            Vect next = vertices[(i + 1) % vertices.Length]; //  The next vertex of the iteration
            
            Vect edge = next - current; // Get the edge vector

            Vect normal = edge.Perpendicular().Normalize(); // Get the perpendicular normal to the edge vector
            
            if (normal.SqrMagnitude() > 0) // If the normal is not a zero vector, add it to the list of axes
                axes.Add(normal);
        }
        return [.. axes]; // Returns the list of axes
    }

    private static Vect GetCenter(Vect[] vertices) // Basic centroid calculation
    {
        Vect sum = vertices.Aggregate(Vect.Zero, (acc, v) => acc + v);
        return sum / vertices.Length;
    }

    private static (float min, float max) Project(Vect axis, Vect[] vertices) // Gets the min and max projection of a shapes vertices onto an axis
    {
        /*
            * Gets each individual vertex of the shape of the shape, 
            * and finds the position of the vertex relative to the vectors/axes plane.
        */

        float min = float.MaxValue; // Set the min and max to the highest and lowest possible values to ensure they are overwritten.
        float max = float.MinValue;

        foreach (Vect v in vertices)
        {
            float projection = Vect.Dot(v, axis); // Project the vertex onto the axis

            // First iteration will always set the min and max.
            min = Math.Min(min, projection); // Set the projection to the min or max value if it is lower or higher than the current value.
            max = Math.Max(max, projection); // This is done to find the min and max projection of the shape onto the axis.
        }

        return (min, max);  // Returns min and max values.
    }

    public static void ResolveList(List<RigidBody> Bodies)
    {
        List<Vect> resolved = [];
        foreach (RigidBody bodyA in Bodies)
        {
            foreach (RigidBody bodyB in Bodies)
            {
                if (bodyA == bodyB)
                { continue; }
                Vect pair = new(bodyA.GetHashCode(), bodyB.GetHashCode());
                Vect reversePair = new(bodyB.GetHashCode(), bodyA.GetHashCode());

                if (resolved.Contains(pair) || resolved.Contains(reversePair))
                { continue; }
                resolved.Add(pair);
                resolved.Add(reversePair);

                if (!CheckSat(bodyA.GlobalVerticies, bodyB.GlobalVerticies))
                { continue; }

                Resolve(bodyA, bodyB);
            }
        }
    }

    public static void Resolve(RigidBody bodyA, RigidBody bodyB)
    {
        Collision.Mtv? collision = Collision.GetMtv(bodyA.GlobalVerticies, bodyB.GlobalVerticies);
        if (collision != null)
        {
            //DebugCollision(bodyA, bodyB, collision); // Uncomment to debug the collision resolution

            Vect relativeVelocity = bodyB.Velocity - bodyA.Velocity;
            float normalVelocity = Vect.Dot(relativeVelocity, collision.Axis);

            if (normalVelocity > 0)
            { return; }

            if (!bodyA.IsMassInf() && !bodyB.IsMassInf())
            {
                bodyA.Move(collision.Axis * collision.Depth / 2);
                bodyB.Move(-collision.Axis * collision.Depth / 2);
            }
            else if (!bodyA.IsMassInf() && bodyB.IsMassInf())
            {
                bodyA.Move(collision.Axis * collision.Depth);
            }
            else if (bodyA.IsMassInf() && !bodyB.IsMassInf())
            {
                bodyB.Move(-collision.Axis * collision.Depth);
            }
        }
    }

    public static void DebugCollision(RigidBody bodyA, RigidBody bodyB, Mtv collision)
    {
        System.Diagnostics.Debug.WriteLine("\n=== COLLISION DEBUG ===");
        System.Diagnostics.Debug.WriteLine($"Time: {DateTime.Now:HH:mm:ss.fff}");
        
        // Body A info
        System.Diagnostics.Debug.WriteLine("\nBody A:");
        System.Diagnostics.Debug.WriteLine($"  Mass: {(bodyA.IsMassInf() ? "Infinite" : bodyA.Mass)}");
        System.Diagnostics.Debug.WriteLine($"  Position: {bodyA.Position}");
        System.Diagnostics.Debug.WriteLine($"  Velocity: {bodyA.Velocity}");
        System.Diagnostics.Debug.WriteLine($"  Forces: {bodyA.AccumulatedForce}");
        
        // Body B info
        System.Diagnostics.Debug.WriteLine("\nBody B:");
        System.Diagnostics.Debug.WriteLine($"  Mass: {(bodyB.IsMassInf() ? "Infinite" : bodyB.Mass)}");
        System.Diagnostics.Debug.WriteLine($"  Position: {bodyB.Position}");
        System.Diagnostics.Debug.WriteLine($"  Velocity: {bodyB.Velocity}");
        System.Diagnostics.Debug.WriteLine($"  Forces: {bodyB.AccumulatedForce}");
        
        // Collision info
        System.Diagnostics.Debug.WriteLine("\nCollision Details:");
        System.Diagnostics.Debug.WriteLine($"  Normal: {collision.Axis}");
        System.Diagnostics.Debug.WriteLine($"  Depth: {collision.Depth}");
        System.Diagnostics.Debug.WriteLine($"  Relative Velocity: {bodyB.Velocity - bodyA.Velocity}");
        System.Diagnostics.Debug.WriteLine($"  Normal Velocity: {Vect.Dot(bodyB.Velocity - bodyA.Velocity, collision.Axis)}");
        
        System.Diagnostics.Debug.WriteLine("===================\n");
    }

    public static void DebugSAT(Vect[] verts1, Vect[] verts2, Vect position1, Vect position2)
    {
        System.Diagnostics.Debug.WriteLine("\n=== SAT DEBUG ===");
        System.Diagnostics.Debug.WriteLine($"Time: {DateTime.Now:HH:mm:ss.fff}");
        
        // Object positions
        System.Diagnostics.Debug.WriteLine("\nPositions:");
        System.Diagnostics.Debug.WriteLine($"  Object 1: {position1}");
        System.Diagnostics.Debug.WriteLine($"  Object 2: {position2}");
        
        // Vertices
        System.Diagnostics.Debug.WriteLine("\nVertices:");
        System.Diagnostics.Debug.WriteLine("  Object 1:");
        foreach (var v in verts1)
            System.Diagnostics.Debug.WriteLine($"    {v}");
        System.Diagnostics.Debug.WriteLine("  Object 2:");
        foreach (var v in verts2)
            System.Diagnostics.Debug.WriteLine($"    {v}");
        
        // Axes and Projections
        System.Diagnostics.Debug.WriteLine("\nAxes and Projections:");
        Vect[] axes = [.. GetAxes(verts1), .. GetAxes(verts2)];
        foreach (var axis in axes)
        {
            if (axis.SqrMagnitude() == 0) continue;
            
            var normalizedAxis = axis.Normalize();
            (float min1, float max1) = Project(normalizedAxis, verts1);
            (float min2, float max2) = Project(normalizedAxis, verts2);
            
            System.Diagnostics.Debug.WriteLine($"  Axis: {normalizedAxis}");
            System.Diagnostics.Debug.WriteLine($"    Shape1 Projection: [{min1}, {max1}]");
            System.Diagnostics.Debug.WriteLine($"    Shape2 Projection: [{min2}, {max2}]");
            System.Diagnostics.Debug.WriteLine($"    Overlap: {(min1 > max2 || min2 > max1 ? "None" : Math.Min(max1 - min2, max2 - min1))}");
        }
        
        System.Diagnostics.Debug.WriteLine("===================\n");
    }
}
