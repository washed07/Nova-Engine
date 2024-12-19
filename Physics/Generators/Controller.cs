using System;
using System.Collections.Generic;
using Microsoft.Xna.Framework.Input;
using Nova.Bodies;
using Nova.Geometry;
using Nova.Numerics;

namespace Nova.Physics.Generators;


public class Controller : LinearForce
{
    private readonly RigidBody _controlledBody;
    public Controller(RigidBody body, Num Speed) : base(Speed, Vect.Zero)
    {
        _controlledBody = body;
    }

    public void Move()
        {
            KeyboardState state    = Keyboard.GetState();

            if (state.IsKeyDown(Keys.W))
            {
                Direction = Vect.Up;
            }
            if (state.IsKeyDown(Keys.A))
            {
                Direction = Vect.Left;
            }
            if (state.IsKeyDown(Keys.S))
            {
                Direction = Vect.Down;
            }
            if (state.IsKeyDown(Keys.D))
            {
                Direction = Vect.Right;
            }

            _controlledBody.Impose(Sum);
        }
}