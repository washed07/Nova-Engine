using System;
using System.Collections.Generic;
using System.Linq;
using Nova.physics;
using Nova.Bodies;
using Nova.Geometry;
using Nova.Numerics;
using Nova.Physics;
using Nova.Physics.Generators;
using System.ComponentModel.DataAnnotations;

namespace Nova;

public class NovaEngine
{
    private static readonly Num TicksPerSecond = 60;
    private static readonly Num Speed          =  1;
    private static readonly Num Iterations     = 1;

    private NovaPhysics _physics;
    private Repository _repository;
    private readonly TimeSpan _runtime = TimeSpan.Zero;
    private TimeSpan _timer   = TimeSpan.Zero;

    private static TimeSpan TickRate => new(10000 / TicksPerSecond / Iterations * Speed);

    public List<RigidBody> Bodies { get; set; } = 
        [
            new RigidBody(Shape.Rect(20, 20), (50, 00), 100, 1),
            new RigidBody(Shape.Rect(20, 20), (200, 100), 100, 1),
            new RigidBody(Shape.Rect(20, 20), (300, 100), 100, 1),
            new RigidBody(Shape.Rect(500, 20), (100, 400), -1, 1),
        ];

    private List<RigidBody> RemoveQueue => [];

    public void Initialize()
    {
        _physics = new NovaPhysics(this);
        _repository = new Repository();
       
        foreach (RigidBody body in Bodies) { body.Initialize(); }
    }

    public void LoadContent()
    {
        // Load content
    }

    public void Update(TimeSpan elapsedTime)
    {
        _timer = _timer.Add(elapsedTime);
        Console.WriteLine(_timer);
        if (!(_timer > TickRate)) { return; }

        _timer   =  TimeSpan.Zero;
        _runtime.Add(TickRate);

        for (int _Iteration = 0; _Iteration < Iterations; _Iteration++) 
        { 
            _physics.Tick(_repository, Bodies);
            _repository.Commit();
            _repository.Clear();
        }

        foreach (RigidBody body in Bodies) // Integrate particle physics
        {
            body.Integrate(TickRate);
            body.Update();
            Collision.CheckAllObjects(Bodies.Cast<object>().ToList());
        }

        foreach (RigidBody body in RemoveQueue) { Bodies.Remove(body); } // Remove aged particles

        RemoveQueue.Clear();
    }

    public void RomoveBody(RigidBody body) { RemoveQueue.Add(body); }
}