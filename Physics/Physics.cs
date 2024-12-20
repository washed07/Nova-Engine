using System;
using System.Linq;
using Nova.Bodies;
using Nova.Geometry;
using Nova.Numerics;
using System.Collections.Generic;
using Nova.Physics;
using Nova.Physics.Generators;

namespace Nova.physics;

public class NovaPhysics(NovaEngine novaEngine)
{
    private readonly NovaEngine _novaEngine = novaEngine;

    public Force Gravity { get; set; } = new Gravity((0, 980));

    public void Tick(Repository repository, List<RigidBody> bodies)
    {
        //Force.Register(repository, Gravity, bodies);

        foreach (RigidBody body in _novaEngine.Bodies.Where(body => !body.IsMassInf()))
        {
        }
    }

}