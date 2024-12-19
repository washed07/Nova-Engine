namespace Nova.Geometry;
#nullable enable
using System.Collections.Generic;
using System.Linq;
using Nova.Numerics;
using Nova.Bodies;
using System.ComponentModel;
using System;

public static class Collision
{
    public class Mtv // Minimum Translation Vector
        (Vect axis, float overlap)
    {
        public Vect Axis { get; set; } = axis;
        public float Depth { get; set; } = overlap;
    }

    public static Mtv? Resolution(Polygon poly1, Polygon poly2)
    {
        Vect[] axes = poly1.GetAxes().Concat(poly2.GetAxes()).ToArray();
        Num minOverlap = float.MaxValue;
        Vect separationAxis = Vect.Zero;

        foreach (Vect axis in axes)
        {
            Vect normalizedAxis = axis.Normalize();
            (Num min1, Num max1) = poly1.Project(normalizedAxis);
            (Num min2, Num max2) = poly2.Project(normalizedAxis);

            // Check for gap
            if (min1 > max2 || min2 > max1)
            {
                return null; // Separating axis found - no collision
            }

            // Calculate overlap
            Num overlap = Num.Min(max1 - min2, max2 - min1);

            // Keep track of minimum overlap
            if (overlap < minOverlap)
            {
                minOverlap = overlap;
                separationAxis = normalizedAxis;
            }
        }

        // Ensure correct direction
        Vect centerDiff = poly2.GetCentroid() + poly2.Position - (poly1.GetCentroid() + poly1.Position);
        if (Vect.Dot(centerDiff, separationAxis) < 0)
        {
            separationAxis = -separationAxis;
        }

        return new Mtv(separationAxis, minOverlap);
    }

    public static void ResolveAll(List<RigidBody> Bodies)
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

                Resolve(bodyA, bodyB);
            }
        }
    }

    public static void Resolve(RigidBody bodyA, RigidBody bodyB)
    {
        Collision.Mtv? translation = Collision.Resolution(bodyA.Polygon, bodyB.Polygon);
        if (translation != null)
        {
            Num f = 0.5f;
            if (!bodyA.IsMassInf() && !bodyB.IsMassInf())
            {
                bodyA.Move(translation.Axis * translation.Depth * f);
                bodyB.Move(-translation.Axis * translation.Depth * f);
            }
            else if (!bodyA.IsMassInf() && bodyB.IsMassInf())
            {
                bodyA.Move(translation.Axis * translation.Depth * f * f);
            }
            else if (bodyA.IsMassInf() && !bodyB.IsMassInf())
            {
                bodyB.Move(-translation.Axis * translation.Depth * f * f);
            }
            bodyA.ResetVelocity();
            bodyB.ResetVelocity();
        }
    }

    public static bool Check(Vect[] polyVertsA, Vect[] polyVertsB)
    {
        for (int i = 0; i < polyVertsA.Length; i++)
        {
            Vect p1 = polyVertsA[i];
            Vect p2 = polyVertsA[(i + 1) % polyVertsA.Length];
            Vect normal = (p2 - p1).Perpendicular().Normalize();

            Num minA = Num.MaxF;
            Num maxA = Num.MinF;
            foreach (Vect vert in polyVertsA)
            {
                Num projection = Vect.Dot(vert, normal);
                minA = Num.Min(minA, projection);
                maxA = Num.Max(maxA, projection);
            }

            Num minB = Num.MaxF;
            Num maxB = Num.MinF;
            foreach (Vect vert in polyVertsB)
            {
                Num projection = Vect.Dot(vert, normal);
                minB = Num.Min(minB, projection);
                maxB = Num.Max(maxB, projection);
            }

            if (maxA < minB || maxB < minA)
            {
                return false;
            }
        }
        return true;
    }

    public static bool CheckAll(List<Vect[]> Verts)
    {
        for (int i = 0; i < Verts.Count; i++)
        {
            for (int j = i + 1; j < Verts.Count; j++)
            {
                if (Check(Verts[i], Verts[j]))
                {
                    return true;
                }
            }
        }
        return false;
    }

    public static bool CheckAllObjects(List<object> objects)
    {
        List<Vect[]> Verts = [];
        foreach (object obj in objects)
        {
            if (obj is RigidBody body)
            {
                Verts.Add(body.Polygon.Vertices);
            }
        }
        return CheckAll(Verts);
    }
}
