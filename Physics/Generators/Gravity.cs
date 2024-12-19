using Nova.Numerics;
using Nova.Physics;
using Nova.Geometry;
using Nova.Bodies;
using System.Runtime.CompilerServices;
using System.Collections.Generic;

namespace Nova.Physics.Generators;

public class Gravity(Vect gravity) : Force
{
    private readonly Vect _gravity = gravity;

    public override void Calculate(List<RigidBody> bodies)
    {
        foreach (RigidBody body in bodies)
        {
            if (body.IsMassInf()) { continue; }
            
            Sum = _gravity;
        }
    }
}