using System;
using System.Collections.Generic;
using System.Net.Security;
using Microsoft.Xna.Framework;
using Nova.Geometry;
using Nova.Numerics;
using Nova.Physics;

namespace Nova.Bodies;

public class RigidBody(Polygon shape, Vect position, Num mass, Num restitution, float damping = 1, Num initialLinearVelocity = default(Num), Vect direction = default(Vect))
{
    public Polygon Polygon { get; set; } = shape;
    public Vect Position { get; set; } = position;
    public Vect Velocity { get; set; } = initialLinearVelocity * direction;
    private Vect Acceleration { get; set; }
    private Num Mass { get; set; } = mass;
    public Num InverseMass { get; set; }
    public Num Restitution { get; set; } = restitution;
    private Num Damping { get; set; } = damping;



    public void Initialize()
    {
        InverseMass = 1 / Mass;
        Polygon.Position = Position;
    }
    public void LoadContent() { }

    public void Integrate(TimeSpan deltaTime)
    {
        if (IsMassInf()) { return; }

        Velocity += Acceleration * deltaTime.TotalSeconds;
        Position += Velocity * deltaTime.TotalSeconds;

        Reset();
    }

    public void Move(Vect displacement) { Position -= displacement; }

    public void SetVelocity(Vect velocity) { Velocity = velocity; }

    public void Impose(Force force) { Acceleration += force.Sum * InverseMass; }

    private void Damp() { Velocity *= Damping; }

    public void Update() { Polygon.Position = Position; }

    public void ResetVelocity() { Velocity = Vect.Zero; }
    public void Reset() { Acceleration = Vect.Zero; }

    public bool IsMassInf() { return (Mass <= 0); }
}