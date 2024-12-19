using Nova.Numerics;
using System.Collections.Generic;
using Nova.Bodies;

namespace Nova.Physics;

public class Force
{
    public Vect Sum { get; set;}
    
    public static implicit operator Force(Vect force) 
    { 
        Force newForce = new Force();
        newForce.Sum = force;
        return newForce;
    }
    public static implicit operator Vect(Force force) { return force.Sum; }

    public Vect Direction()     { return Sum.Normalize(); } 
    public Vect Perpendicular() { return Direction().Perpendicular(); }
    
    public void Direct(Vect direction) { Sum *= direction; }
    public Force Direct(Force force, Vect direction) { return new Force().Sum = force.Sum * direction; }

    public virtual void Calculate(List<RigidBody> bodies) {}
    
    public static void Register(Repository repository, Force force, List<RigidBody> bodies) { repository.Register( new Registry(bodies, force));}
}

public class LinearForce(Num magnitude, Vect direction)
{
    public  Vect Sum        => Dir * Magnitude;
    public  Num  Magnitude  { get; set; } = magnitude;
    private Vect Dir { get; set; } = direction;
    
    public Vect Direction { get => Dir; set { if (value.Magnitude() > 1 || value.Magnitude() < -1) { Dir = value.Normalize();} else { Dir = value;} } }

    public static implicit operator LinearForce(Force force) { return new LinearForce(force.Sum.Magnitude(), force.Direction()); }
    public static implicit operator Force(LinearForce force) { return new Force().Sum = force.Sum * force.Direction; }
    public static implicit operator Num(LinearForce force) { return force.Magnitude; }

    public virtual void Calculate(List<RigidBody> bodies) {}
    
    public static void Register(Repository repository, LinearForce force, List<RigidBody> bodies) { repository.Register( new Registry(bodies, force:force));}
}

// public class Torque(Num sum)
// {
//     public Num Sum { get; set; } = sum;
//     
//     public static implicit operator Num(Torque torque) { return torque.Sum; }
//     public static implicit operator Torque(Num sum) { return new Torque(sum); }
//
//     public void Calculate(List<RigidBody> bodies) {}
//     
//     public void Register(Repository repository, Torque force, List<RigidBody> bodies) { repository.Register( new Registry(bodies, force:force));}
// }