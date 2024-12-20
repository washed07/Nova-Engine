using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Nova.Numerics;

[StructLayout(LayoutKind.Sequential)]
[method: MethodImpl(MethodImplOptions.AggressiveInlining)]
public struct Vect(Num x, Num y, Num z = default(Num), Num w = default(Num)) : IEquatable<Vect>, IComparable<Vect>
{
    public Num x = x, y = y, z = z, w = w;
    private readonly Num? cachedMagnitude = null;

    public static readonly Vect Zero = (0, 0);

    public static readonly Vect Up = (0, -1);

    public static readonly Vect Down = (0, 1);

    public static readonly Vect Right = (1, 0);

    public static readonly Vect Left = (-1, 0);

    // Add more useful vector constants
    public static readonly Vect One = (1, 1);
    public static readonly Vect Forward = (0, -1);
    public static readonly Vect Back = (0, 1);
    public static readonly Vect PositiveInfinity = (float.PositiveInfinity, float.PositiveInfinity);
    public static readonly Vect NegativeInfinity = (float.NegativeInfinity, float.NegativeInfinity);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vect WithMagnitude(Num magnitude)
    {
        Num currentMag = Magnitude();
        if (currentMag == 0)
        { return this; }

        Num scale = magnitude / currentMag;
        return this * scale;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Num Magnitude()
    {
        if (cachedMagnitude.HasValue)
        { return cachedMagnitude.Value; }

        return Math.Sqrt(SqrMagnitude());
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Num SqrMagnitude() { return x * x + y * y + z * z + w * w; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vect Normalize(Vect vect)
    {
        Num mag = vect.Magnitude();
        if (mag == 0)
        { return vect; }

        return vect / mag;
    }

    public Vect[] Select(Func<Vect, Vect> selector) => [selector(this)];

    public Vect Normalize() => Normalize(this);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Vect Perpendicular() => new(-y, x, z, w);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vect operator -(Vect a) => new(-a.x, -a.y, -a.z, -a.w);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vect Lerp(Vect a, Vect b, Num t) { return a + (b - a) * t; }

    public static Vect SmoothStep(Vect a, Vect b, Num t)
    {
        t = Num.Clamp((t - 0) / (1 - 0), 0, 1);
        t = t * t * (3 - 2 * t);
        return Lerp(a, b, t);
    }

    public static Vect CatmullRom(Vect p0, Vect p1, Vect p2, Vect p3, Num t)
    {
        Num t2 = t * t;
        Num t3 = t2 * t;

        return 0.5f * (
            (2 * p1) +
            (-p0 + p2) * t +
            (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 +
            (-p0 + 3 * p1 - 3 * p2 + p3) * t3
        );
    }

    private static Vect Min(Vect a, Vect b)
    {
        return new Vect(Num.Min(a.x, b.x), Num.Min(a.y, b.y), Num.Min(a.z, b.z), Num.Min(a.w, b.w));
    }

    private static Vect Max(Vect a, Vect b)
    {
        return new Vect(Num.Max(a.x, b.x), Num.Max(a.y, b.y), Num.Max(a.z, b.z), Num.Max(a.w, b.w));
    }

    public static Vect Clamp(Vect value, Vect min, Vect max) { return Min(Max(value, min), max); }

    public static Vect Abs(Vect value)
    {
        return new Vect(Num.Abs(value.x), Num.Abs(value.y), Num.Abs(value.z), Num.Abs(value.w));
    }

    // Optimized operators using SIMD when available
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vect operator +(Vect a, Vect b) { return new Vect(a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w); }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vect operator -(Vect a, Vect b) { return new Vect(a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w); }

    public static Vect Rotate(Vect v, Num angle)
    {
        Num cos = Num.Cos(angle);
        Num sin = Num.Sin(angle);
        return new Vect(v.x * cos - v.y * sin, v.x * sin + v.y * cos, v.z, v.w);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vect operator *(Vect a, Num b) { return new Vect(a.x * b, a.y * b, a.z * b, a.w * b); }

    public static Vect operator +(Vect a, Num b) { return new Vect(a.x + b, a.y + b, a.z + b, a.w + b); }

    public static Vect operator +(Num a, Vect b) { return new Vect(a + b.x, a + b.y, a + b.z, a + b.w); }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vect operator *(Vect a, Vect b) { return new Vect(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w); }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vect operator /(Vect a, Num b)
    {
        Num invB = 1 / b;
        return new Vect(a.x * invB, a.y * invB, a.z * invB, a.w * invB);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vect operator /(Vect a, Vect b) { return new Vect(a.x / b.x, a.y / b.y, a.z / b.z, a.w / b.w); }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vect operator *(Num a, Vect b) { return b * a; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vect operator /(Num a, Vect b) { return new Vect(a / b.x, a / b.y, a / b.z, a / b.w); }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Num Dot(Vect a, Vect b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vect Cross(Vect a, Vect b)
    {
        return new Vect(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Num Distance(Vect a, Vect b) { return (a - b).Magnitude(); }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Num DistanceSqr(Vect a, Vect b)
    {
        Num dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z, dw = a.w - b.w;
        return dx * dx + dy * dy + dz * dz + dw * dw;
    }

    // Efficient comparison operators using squared magnitude
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator >(Vect a, Vect b) { return a.SqrMagnitude() > b.SqrMagnitude(); }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vect operator -(Vect a, Num b) { return new Vect(a.x - b, a.y - b, a.z - b, a.w - b); }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator <(Vect a, Vect b) { return a.SqrMagnitude() < b.SqrMagnitude(); }

    public static implicit operator Vect((Num x, Num y, Num z) tuple) { return new Vect(tuple.x, tuple.y, tuple.z); }

    public static implicit operator Vect((Num x, Num y) tuple) { return new Vect(tuple.x, tuple.y); }

    public static implicit operator Vect(Vector2 v) { return new Vect(v.X, v.Y); }

    public static implicit operator Vect(Microsoft.Xna.Framework.Vector2 v) { return new Vect(v.X, v.Y); }

    public static implicit operator Vect(Vector3 v) { return new Vect(v.X, v.Y, v.Z); }

    public static implicit operator Vect(Microsoft.Xna.Framework.Vector3 v) { return new Vect(v.X, v.Y, v.Z); }

    public static implicit operator Vector2(Vect v) { return new Vector2(v.x, v.y); }

    public static implicit operator Microsoft.Xna.Framework.Vector2(Vect v)
    {
        return new Microsoft.Xna.Framework.Vector2(v.x, v.y);
    }

    public static implicit operator Vector3(Vect v) { return new Vector3(v.x, v.y, v.z); }

    public static implicit operator Microsoft.Xna.Framework.Vector3(Vect v)
    {
        return new Microsoft.Xna.Framework.Vector3(v.x, v.y, v.z);
    }

    public override bool Equals(object obj)
    {
        if (obj is Vect vect)
        { return vect == this; }

        return false;
    }

    public static bool operator ==(Vect a, Vect b) { return a.x == b.x && a.y == b.y && a.z == b.z && a.w == b.w; }

    public static bool operator !=(Vect a, Vect b) { return !(a == b); }

    public override int GetHashCode()
    {
        unchecked
        {
            int hash = 17;
            hash = hash * 31 + x.GetHashCode();
            hash = hash * 31 + y.GetHashCode();
            hash = hash * 31 + z.GetHashCode();
            return hash * 31 + w.GetHashCode();
        }
    }

    private static readonly string Format = "({0:F6}, {1:F6}, {2:F6}, {3:F6})";
    public override string ToString() { return string.Format(Format, x, y, z, w); }

    public bool Equals(Vect other)
    {
        return x.Equals(other.x) && y.Equals(other.y) && z.Equals(other.z) && w.Equals(other.w) && Nullable.Equals
            (cachedMagnitude, other.cachedMagnitude);
    }

    public int CompareTo(Vect other) { return Magnitude().CompareTo(other.Magnitude()); }

    // Add angle calculation methods
    public Num AngleDeg()
    {
        return Num.Atan2(y, x) * 180 / Num.PI;
    }

    public Num AngleRad()
    {
        return Num.Atan2(y, x);
    }

    public static Num Angle(Vect from, Vect to)
    {
        Num denominator = Num.Sqrt(from.SqrMagnitude() * to.SqrMagnitude());
        if (denominator < 1e-15f)
            return 0;

        Num dot = Num.Clamp(Dot(from, to) / denominator, -1, 1);
        return Num.Acos(dot);
    }

    public static Num SignedAngle(Vect from, Vect to)
    {
        Num unsigned = Angle(from, to);
        Num sign = Num.Sign(from.x * to.y - from.y * to.x);
        return unsigned * sign;
    }

    // Add vector projection and reflection
    public static Vect Project(Vect vector, Vect onNormal)
    {
        Num sqrMag = onNormal.SqrMagnitude();
        if (sqrMag < 1e-15f)
            return Zero;

        Num dot = Dot(vector, onNormal);
        return onNormal * dot / sqrMag;
    }

    public static Vect Reflect(Vect inDirection, Vect inNormal)
    {
        Num factor = -2 * Dot(inNormal, inDirection);
        return inDirection + factor * inNormal;
    }

    // Add component-wise operations
    public static Vect Scale(Vect a, Vect b)
    {
        return new Vect(a.x * b.x, a.y * b.y, a.z * b.z, a.w * b.w);
    }

    public void Scale(Vect scale)
    {
        x *= scale.x;
        y *= scale.y;
        z *= scale.z;
        w *= scale.w;
    }

    public static Vect InverseLerp(Vect a, Vect b, Vect value)
    {
        if (a != b)
        {
            return new Vect(
                (value.x - a.x) / (b.x - a.x),
                (value.y - a.y) / (b.y - a.y),
                (value.z - a.z) / (b.z - a.z),
                (value.w - a.w) / (b.w - a.w)
            );
        }
        return Zero;
    }

    // Add utility methods
    public bool IsNormalized()
    {
        return Math.Abs(SqrMagnitude() - 1) < 1e-15f;
    }

    public static Vect ClampMagnitude(Vect vector, Num maxLength)
    {
        Num sqrMagnitude = vector.SqrMagnitude();
        if (sqrMagnitude > maxLength * maxLength)
        {
            Num mag = Num.Sqrt(sqrMagnitude);
            Num normalizedX = vector.x / mag;
            Num normalizedY = vector.y / mag;
            Num normalizedZ = vector.z / mag;
            Num normalizedW = vector.w / mag;
            return new Vect(
                normalizedX * maxLength,
                normalizedY * maxLength,
                normalizedZ * maxLength,
                normalizedW * maxLength);
        }
        return vector;
    }

    public Vect SetMagnitude(Num newMagnitude)
    {
        return Normalize() * newMagnitude;
    }

    // Add vector construction helpers
    public static Vect FromAngle(Num angle)
    {
        return new Vect(Num.Cos(angle), Num.Sin(angle));
    }

    public static Vect FromAngleDegrees(Num angleDegrees)
    {
        Num angleRadians = angleDegrees * Num.PI / 180;
        return FromAngle(angleRadians);
    }

    // Add swizzling operations
    public Vect yx => new(y, x);
    public Vect zx => new(z, x);
    public Vect zy => new(z, y);
    public Vect xy => new(x, y);
    public Vect xz => new(x, z);
    public Vect yz => new(y, z);

    // Add component min/max operations
    public Num MaxComponent() => Num.Max(Num.Max(Num.Max(x, y), z), w);
    public Num MinComponent() => Num.Min(Num.Min(Num.Min(x, y), z), w);
}
