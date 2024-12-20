using System;
using System.Collections.Generic;
using System.Net.Security;
using Microsoft.Xna.Framework;
using Nova.Geometry;
using Nova.Numerics;
using Nova.Physics;

namespace Nova.Bodies;

public class RigidBody(Polygon shape, Vect position, Num mass, Num restitution, float damping = 1, Num initialLinearVelocity = default, Vect direction = default)
{
    public Polygon Polygon { get; set; } = shape;
    public Vect Position { get; set; } = position;
    public Vect Velocity { get; set; } = initialLinearVelocity * direction;
    public Vect Acceleration { get; set; }
    public Num Mass { get; set; } = mass;
    public Num InverseMass { get; set; }
    public Num Restitution { get; set; } = restitution;
    private Num Damping { get; set; } = damping;

    public Vect AccumulatedForce = Vect.Zero;

    public Vect[] GlobalVerticies => Polygon.TransformedVertices();
    public Vect[] LocalVerticies => Polygon.Vertices;
    public Vect CenterPosition => Polygon.GetCentroid() + Position;



    public void Initialize()
    {
        InverseMass = 1 / Mass;
        Polygon.Position = Position;
    }
    public void LoadContent() { }

    public void Move(Vect displacement) { Position -= displacement; }

    public void SetVelocity(Vect velocity) { Velocity = velocity; }

    public void ApplyForce(Force force) { AccumulatedForce += force.Sum * InverseMass; }

    public void ClearForces()
    {
        AccumulatedForce = Vect.Zero;
        Acceleration = Vect.Zero;
    }

    public void Damp() { Velocity *= Damping; }

    public void Update() { Polygon.Position = Position; }

    public void ResetVelocity() { Velocity = Vect.Zero; }

    public bool IsMassInf() { return (Mass <= 0); }
}