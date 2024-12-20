using Nova.Bodies;
using Nova.Geometry;
using Nova.Numerics;
using Nova.physics;
using Nova.Physics;
using Nova.Physics.Generators;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Nova;

public class NovaEngine // TODO: fix Tickrate not affecting the actual framerate of the physics engine, implement continuous collision detection, rungue-kutta integration, and a more robust collision resolution system
{
    private static readonly Num TicksPerSecond = 1;
    private static readonly Num Speed = 1;
    private static readonly Num Iterations = 1;

    private NovaPhysics _physics;
    private Repository _repository;

    private Num _runtime = 0;
    private Num _frametime = 0;

    private Controller _controller;

    public static Num TickRate => 1 / TicksPerSecond / Iterations * Speed;

    public List<RigidBody> Bodies { get; set; } =
        [
            new RigidBody(Shape.Rect(20, 20), (50, 0), 100, 1),
            new RigidBody(Shape.Rect(20, 20), (50, 50), 500, 1),
            new RigidBody(Shape.Rect(20, 20), (50, 100), 500, 1),
            new RigidBody(Shape.Rect(500, 20), (100, 400), -1, 1),
        ];

    private List<RigidBody> RemoveQueue => [];

    public void Initialize()
    {
        _physics = new NovaPhysics(this);
        _repository = new Repository();
        _controller = new Controller(Bodies[0], 100);

        foreach (RigidBody body in Bodies) { body.Initialize(); }
    }

    public void LoadContent()
    {
        // Load content
    }

    public void Update(Num elapsedTime)
    {
        _frametime += elapsedTime;
        if (!(_frametime > TickRate)) { return; }

        _frametime = 0;
        _runtime += TickRate;

        for (int _Iteration = 0; _Iteration < Iterations; _Iteration++)
        {
            // Clear all forces at start of physics step
            foreach (RigidBody body in Bodies)
            {
                body.ClearForces();
            }

            // Apply new forces
            _physics.Tick(_repository, Bodies);

            _repository.Commit();
            _repository.Clear();

            _controller.Move();
            UpdateBodies();
            

            // Resolve collisions
            Collision.ResolveList(Bodies);
        }

        foreach (RigidBody body in RemoveQueue)
        {
            Bodies.Remove(body);
        }
        RemoveQueue.Clear();
    }

    public void UpdateBodies()
    {
        foreach (RigidBody body in Bodies)
        {
            body.Acceleration += body.AccumulatedForce * TickRate;
            body.Velocity += body.Acceleration * TickRate;
            body.Position += body.Velocity * TickRate;

            body.Damp();
            body.ClearForces();

            body.Update();
        }
    }

    public void RomoveBody(RigidBody body) { RemoveQueue.Add(body); }
}