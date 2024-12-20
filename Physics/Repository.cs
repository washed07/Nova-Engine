// In Development
using Nova.Numerics;
using Nova.Bodies;
using System.Collections.Generic;
using System.Linq;
using System;

namespace Nova.Physics;

public class Repository()
{
    private readonly List<Registry> _registries = [];
    private readonly List<RigidBody> _registeredBodies = [];
    public void Register(Registry registry)
    {
        foreach (RigidBody body in registry.Bodies)
        {
            if (!_registeredBodies.Contains(body)) { _registeredBodies.Add(body); }
        }
        _registries.Add(registry);
    }

    public void Commit()
    {
        foreach (RigidBody body in _registeredBodies)
        {
            Vect netForce = Vect.Zero;
            foreach (Registry registry in _registries)
            {
                if (registry.Bodies.Contains(body)) 
                { 
                    registry.Force.Calculate(registry.Bodies);
                    netForce += registry.Force.Sum;
                }
            }
            if (netForce != Vect.Zero) { body.ApplyForce(netForce); };
        }
    }

    public void Clear() { _registries.Clear(); }
        
}

public struct Registry(List<RigidBody> bodies, Force force)
{
    public readonly List<RigidBody> Bodies      = bodies;
    public readonly Force           Force       = force;
    //public readonly Torque          Torque      = torque;
}