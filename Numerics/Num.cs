#nullable enable
using System;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.Numerics;

namespace Nova.Numerics;

public readonly struct Num : // All in one integral data type, with a few extra features and all Math operations and functions.
    IEquatable<Num>, 
    IConvertible,
    INumber<Num>,
    IAdditionOperators<Num, Num, Num>,
    ISubtractionOperators<Num, Num, Num>,
    IMultiplyOperators<Num, Num, Num>,
    IDivisionOperators<Num, Num, Num>,
    IModulusOperators<Num, Num, Num>,
    IComparable,
    IComparable<Num>,
    ISpanFormattable,
    IFormattable,
    ISpanParsable<Num>
{
    private readonly       float _value;
    public static readonly Num   E         = 2.718281828459045235360287471352662497757247093699959574966967d;
    public static readonly Num   PI        = 3.141592653589793238462643383279502884197169399375105820974944d;
    public static readonly double  PId      = 3.141592653589793d;
    public static readonly float   PIf       = 3.1415926f;
    public static readonly Num   SQRT2     = 1.414213562373095048801688724209698078569671875376948073176679d;
    public static readonly Num   PHI       = 1.618033988749894848204586834365638117720309179805762862135448d;
    public static readonly Num   TAU       = 6.283185307179586476925286766559005768394338798750211641949889d;
    public static readonly Num   LN2       = 0.693147180559945309417232121458176568075500134360255254120680009;
    public static readonly Num   LN10      = 2.302585092994045684017991454684364207601101488628772976033327900;
    public static readonly Num   LOG2E     = 1.442695040888963407359924681001892137426645954152985934135449407;
    public static readonly Num   LOG10E    = 0.434294481903251827651128918916605082294397005803666566114453783;
    private const          double Tolerance = 1e-15d; // Tolerance for floating point comparisons || Performance heavy at lower values

    public readonly static Num MaxF = float.MaxValue;
    public readonly static Num MaxD = double.MaxValue;
    public readonly static Num MinF = float.MinValue;
    public readonly static Num MinD = double.MinValue;

    private Num(float value) { _value = value; }

    public static Num Abs(Num value)
    {
        // Manual implementation without using float operations
        return value < 0 ? -value : value;
    }

    public static Num Min(Num a, Num b) { if (a < b) { return a; } else { return b; } } // Nothing too special here

    public static Num Max(Num a, Num b) { if (a > b) { return a; } else { return b; } } // Same goes for this

    public static Num Sign(Num value) { return value < 0 ? -1 : value > 0 ? 1 : 0; } // Sign function

    public static Num Cos(Num value)
    {
        // Taylor series for cosine: cos(x) = 1 - x²/2! + x⁴/4! - x⁶/6! + ...
        Num x = value % (2 * PI); // Normalize to [0, 2π]
        Num x2 = x * x;
        Num result = 1;
        
        // First 6 terms of Taylor series
        result -= x2 / 2;
        result += (x2 * x2) / 24;
        result -= (x2 * x2 * x2) / 720;
        result += (x2 * x2 * x2 * x2) / 40320;
        
        return result;
    }

    public static Num Acos(Num value)
    {
        // Taylor series for acos(x) = π/2 - x - x³/6 - 3x⁵/40 - 5x⁷/112 - 35x⁹/1152 - ...
        if (value > 1 || value < -1)
            return float.NaN;
            
        Num x = value;
        Num x2 = x * x;
        Num result = PI / 2;
        
        // First 6 terms of Taylor series
        result -= x;
        result -= x2 * x / 6;
        result -= (3 * x2 * x2 * x) / 40;
        result -= (5 * x2 * x2 * x2 * x) / 112;
        result -= (35 * x2 * x2 * x2 * x2 * x) / 1152;
        
        return result;
    }

    public static Num Sin(Num value)
    {
        // Taylor series for sine: sin(x) = x - x³/3! + x⁵/5! - x⁷/7! + ...
        Num x = value % (2 * PI); // Normalize to [0, 2π]
        Num x2 = x * x;
        Num x3 = x2 * x;
        Num result = x;
        
        // First 6 terms of Taylor series
        result -= x3 / 6;
        result += (x3 * x2) / 120;
        result -= (x3 * x2 * x2) / 5040;
        result += (x3 * x2 * x2 * x2) / 362880;
        
        return result;
    }

    public static Num Tan(Num value)
    {
        Num s = Sin(value);
        Num c = Cos(value);
        return s / (c == 0 ? Tolerance : c);
    }

    public static Num Atan(Num value)
    {
        // Taylor series for atan(x) = x - x³/3 + x⁵/5 - x⁷/7 + ...
        if (value > 1 || value < -1)
            return PI/2 - Atan(1/value);
            
        Num x = value;
        Num x2 = x * x;
        Num result = x;
        
        for (int i = 3; i <= 11; i += 2)
        {
            x *= -x2;
            result += x / i;
        }
        
        return result;
    }

    public static Num Atan2(Num y, Num x)
    {
        if (x > 0)
            return Atan(y/x);
        else if (x < 0)
            return y >= 0 ? Atan(y/x) + PI : Atan(y/x) - PI;
        else if (y > 0)
            return PI/2;
        else if (y < 0)
            return -PI/2;
        return 0; // undefined, but return 0
    }

    public static Num Sqrt(Num value)
    {
        if (value < 0) return new Num(float.NaN);
        if (value == 0) return Zero;
        
        // Newton's method: x[n+1] = (x[n] + value/x[n])/2
        Num x = value / 2;
        Num lastX;
        
        do
        {
            lastX = x;
            x = (x + value / x) / 2;
        } while (Abs(x - lastX) > Tolerance);
        
        return x;
    }

    public static Num Cbrt(Num value)
    {
        if (value == 0) return Zero;
        
        Num x = value / 3;  // Initial guess
        
        // Newton's method for cube root
        for (int i = 0; i < 8; i++)
        {
            x = (2 * x + value / (x * x)) / 3;
        }
        
        return x;
    }

    public static Num Exp(Num value)
    {
        // Taylor series for e^x
        if (value == 0) return One;
        
        Num result = One;
        Num term = One;
        
        for (int i = 1; i <= 10; i++)
        {
            term *= value / i;
            result += term;
        }
        
        return result;
    }

    public static Num Log(Num value)
    {
        // Use the identity log(x) = 2 * atanh((x-1)/(x+1))
        if (value <= 0) return new Num(float.NaN);
        if (value == 1) return Zero;
        
        Num y = (value - 1) / (value + 1);
        Num y2 = y * y;
        Num result = y;
        
        for (int i = 3; i <= 11; i += 2)
        {
            y *= y2;
            result += y / i;
        }
        
        return 2 * result;
    }

    public static Num Log2(Num value)
    {
        return Log(value) / LN2;
    }

    public static Num Log10(Num value)
    {
        return Log(value) / LN10;
    }

    public static Num Sinh(Num value)
    {
        return (Exp(value) - Exp(-value)) / 2;
    }

    public static Num Cosh(Num value)
    {
        return (Exp(value) + Exp(-value)) / 2;
    }

    public static Num Tanh(Num value)
    {
        if (value > 50) return One;  // Avoid overflow
        if (value < -50) return -One;
        
        Num exp2x = Exp(2 * value);
        return (exp2x - 1) / (exp2x + 1);
    }

    public static Num PowInt(Num value, int n)
    {
        if (n == 0) return One;
        if (n < 0)
        {
            value = One / value;
            n = -n;
        }
        
        Num result = One;
        while (n > 0)
        {
            if ((n & 1) == 1)
                result *= value;
            value *= value;
            n >>= 1;
        }
        return result;
    }

    public static Num Pow(Num x, Num y)
    {
        if (y == 0) return One;
        if (x == 0) return Zero;
        if (y == 1) return x;
        
        // Handle integer powers efficiently
        if (IsInteger(y))
            return PowInt(x, (int)y);
            
        // For non-integer powers, use exp(y*ln(x))
        return Exp(y * Log(x));
    }

    public static Num Round(Num value)
    {
        Num integerPart = new Num((int)value._value);
        Num fraction = value - integerPart;
        
        if (fraction < 0.5)
            return integerPart;
        return integerPart + One;
    }

    public static Num Floor(Num value)
    {
        return new Num((int)value._value);
    }

    public static Num Ceiling(Num value)
    {
        int intPart = (int)value._value;
        return value == intPart ? value : new Num(intPart + 1);
    }

    public static Num Clamp(Num value, Num min, Num max) { return Min(Max(value, min), max); }

    // Generic conversion method for numeric types
    private static Num FromT<T>(T value) where T : IConvertible { return new Num(Convert.ToSingle(value)); }

    public Num ToRadians() { return new Num(_value * PI / 180); }

    // Add null check for FromT
    private static Num? FromT<T>(T? value) where T : struct, IConvertible
    {
        return value == null ? null : new Num(Convert.ToSingle(value));
    }

    public static Num N(Num value) { return new Num(value); }

    public Num Radians() { return new Num(_value * PI / 180); }

    public Num Degrees() { return new Num(_value * 180 / PI); }

    // Implicit conversions
    public static implicit operator Num(byte value) { return FromT(value); }

    public static implicit operator Num(int value) { return FromT(value); }

    public static implicit operator Num(float value) { return FromT(value); }

    public static implicit operator Num(double value) { return FromT(value); }

    public static implicit operator Num(decimal value) { return FromT(value); }

    // Null-safe implicit conversions
    public static implicit operator Num?(float? value) { return value.HasValue ? new Num(value.Value) : null; }

    public static implicit operator Num?(int? value) { return value.HasValue ? new Num(value.Value) : null; }

    // Conversions to other types
    public static implicit operator float(Num value) { return value._value; }

    public static implicit operator double(Num value) { return value._value; }

    public static implicit operator int(Num value) { return (int)value._value; }

    public static implicit operator byte(Num value) { return (byte)value._value; }

    public static implicit operator decimal(Num value) { return (decimal)value._value; }

    // Basic arithmetic operators
    public static Num operator +(Num a, Num b) { return new Num(a._value + b._value); }

    public static Num operator -(Num a, Num b) { return new Num(a._value - b._value); }

    public static Num operator *(Num a, Num b) { return new Num(a._value * b._value); }

    public static Num operator /(Num a, Num b) { return new Num(a._value / b._value); }

    public static Num operator %(Num a, Num b) { return new Num(a._value % b._value); }

    public static Num operator -(Num a) { return new Num(-a._value); }

    public static Num operator +(Num a) { return a; }

    public static Num operator ++(Num a) { return new Num(a._value + 1); }

    public static Num operator --(Num a) { return new Num(a._value - 1); }

    // Null-safe arithmetic operators
    public static Num? operator +(Num? a, Num? b)
    {
        return a.HasValue && b.HasValue ? new Num(a.Value._value + b.Value._value) : null;
    }

    public static Num? operator -(Num? a, Num? b)
    {
        return a.HasValue && b.HasValue ? new Num(a.Value._value - b.Value._value) : null;
    }

    public static Num? operator *(Num? a, Num? b)
    {
        return a.HasValue && b.HasValue ? new Num(a.Value._value * b.Value._value) : null;
    }

    public static Num? operator /(Num? a, Num? b)
    {
        return a.HasValue && b.HasValue ? new Num(a.Value._value / b.Value._value) : null;
    }

    // Comparison operators
    public static bool operator ==(Num a, Num b) { return Abs(a._value - b._value) < Tolerance; }


    public static bool operator !=(Num a, Num b) { return Abs(a._value - b._value) > Tolerance; }

    public static bool operator >(Num a, Num b) { return a._value > b._value; }

    public static bool operator <(Num a, Num b) { return a._value < b._value; }

    public static bool operator >=(Num a, Num b) { return a._value >= b._value; }

    public static bool operator <=(Num a, Num b) { return a._value <= b._value; }

    // Null-safe comparison operators
    public static bool operator ==(Num? a, Num? b)
    {
        return (!a.HasValue && !b.HasValue) ||
               (a.HasValue  && b.HasValue && Abs(a.Value._value - b.Value._value) < Tolerance);
    }

    public static bool operator !=(Num? a, Num? b) { return !(a == b); }

    public static bool operator >(Num? a, Num? b)
    {
        return a.HasValue && b.HasValue && a.Value._value > b.Value._value;
    }

    public static bool operator <(Num? a, Num? b)
    {
        return a.HasValue && b.HasValue && a.Value._value < b.Value._value;
    }

    // Logical operators
    public static bool operator true(Num a) { return a._value != 0; }

    public static bool operator false(Num a) { return a._value == 0; }

    public static Num operator !(Num a) { return new Num(a._value == 0 ? 1 : 0); }

    // Bitwise operators
    public static Num operator &(Num a, Num b) { return new Num((int)a._value & (int)b._value); }

    public static Num operator |(Num a, Num b) { return new Num((int)a._value | (int)b._value); }

    public static Num operator ^(Num a, Num b) { return new Num((int)a._value ^ (int)b._value); }

    public static Num operator ~(Num a) { return new Num(~(int)a._value); }

    public static Num operator <<(Num a, int b) { return new Num((int)a._value << b); }

    public static Num operator >> (Num a, int b) { return new Num((int)a._value >> b); }

    // Equality implementation
    public bool Equals(Num other) { return Abs(_value - other._value) < Tolerance; }

    public override bool Equals(object? obj) { return obj is Num other && Equals(other); }

    public override int GetHashCode() { return _value.GetHashCode(); }

    public override string ToString() { return _value.ToString(CultureInfo.CurrentCulture); }

    public int CompareTo(Num other)
    {
        if (Abs(this._value - other._value) < Tolerance)
        {
            return 0; // Equal within tolerance
        }

        return this._value > other._value ? 1 : -1; // Greater than or less than
    }

    // Add IConvertible implementation
    public TypeCode GetTypeCode() => TypeCode.Single;
    public sbyte ToSByte(IFormatProvider? provider) => Convert.ToSByte(_value);
    public float ToSingle(IFormatProvider? provider) => _value;
    public bool ToBoolean(IFormatProvider? provider) => _value != 0;
    public byte ToByte(IFormatProvider? provider) => Convert.ToByte(_value);
    public char ToChar(IFormatProvider? provider) => Convert.ToChar(_value);
    public DateTime ToDateTime(IFormatProvider? provider) => throw new InvalidCastException();
    public decimal ToDecimal(IFormatProvider? provider) => Convert.ToDecimal(_value);
    public double ToDouble(IFormatProvider? provider) => Convert.ToDouble(_value);
    public short ToInt16(IFormatProvider? provider) => Convert.ToInt16(_value);
    public int ToInt32(IFormatProvider? provider) => Convert.ToInt32(_value);
    public long ToInt64(IFormatProvider? provider) => Convert.ToInt64(_value);
    string IConvertible.ToString(IFormatProvider? provider) => _value.ToString(provider);
    object IConvertible.ToType(Type conversionType, IFormatProvider? provider) => Convert.ChangeType(_value, conversionType, provider);
    ushort IConvertible.ToUInt16(IFormatProvider? provider) => Convert.ToUInt16(_value);
    uint IConvertible.ToUInt32(IFormatProvider? provider) => Convert.ToUInt32(_value);
    ulong IConvertible.ToUInt64(IFormatProvider? provider) => Convert.ToUInt64(_value);

    // Add required static properties for INumber<Num>
    public static Num One => new Num(1);
    public static Num Zero => new Num(0);
    public static Num AdditiveIdentity => Zero;
    public static Num MultiplicativeIdentity => One;

    // Add required methods for INumber<Num>
    public static Num Parse(string s, IFormatProvider? provider) 
        => new Num(float.Parse(s, provider));
    
    public static bool TryParse([NotNullWhen(true)] string? s, IFormatProvider? provider, out Num result)
    {
        if (float.TryParse(s, NumberStyles.Float, provider, out float value))
        {
            result = new Num(value);
            return true;
        }
        result = Zero;
        return false;
    }

    public int CompareTo(object? obj)
    {
        if (obj is Num other)
            return CompareTo(other);
        throw new ArgumentException("Object must be of type Num");
    }

    // Add absolute value method required by INumber<T>
    static Num INumberBase<Num>.Abs(Num value) => Abs(value);
    
    // Add required methods for sign checks
    public static bool IsNegative(Num value) => value._value < 0;
    public static bool IsPositive(Num value) => value._value > 0;
    public static bool IsZero(Num value) => Abs(value._value) < Tolerance;
    public static bool IsInteger(Num value) => value._value % 1 == 0;

    // Add INumberBase<Num> required implementations
    public static bool IsCanonical(Num value) => true;
    public static bool IsComplexNumber(Num value) => false;
    public static bool IsEvenInteger(Num value) => value._value % 2 == 0 && IsInteger(value);
    public static bool IsFinite(Num value) => float.IsFinite(value._value);
    public static bool IsImaginaryNumber(Num value) => false;
    public static bool IsInfinity(Num value) => float.IsInfinity(value._value);
    public static bool IsNaN(Num value) => float.IsNaN(value._value);
    public static bool IsNegativeInfinity(Num value) => float.IsNegativeInfinity(value._value);
    public static bool IsNormal(Num value) => float.IsNormal(value._value);
    public static bool IsOddInteger(Num value) => value._value % 2 != 0 && IsInteger(value);
    public static bool IsPositiveInfinity(Num value) => float.IsPositiveInfinity(value._value);
    public static bool IsRealNumber(Num value) => !float.IsNaN(value._value);
    public static bool IsSubnormal(Num value) => float.IsSubnormal(value._value);
    
    public static Num MaxMagnitude(Num x, Num y)
    {
        // Manual implementation
        Num absX = Abs(x);
        Num absY = Abs(y);
        return absX > absY ? x : absY > absX ? y : x >= y ? x : y;
    }
    public static Num MaxMagnitudeNumber(Num x, Num y) => new Num(float.MaxMagnitudeNumber(x._value, y._value));
    public static Num MinMagnitude(Num x, Num y)
    {
        // Manual implementation
        Num absX = Abs(x);
        Num absY = Abs(y);
        return absX < absY ? x : absY < absX ? y : x <= y ? x : y;
    }
    public static Num MinMagnitudeNumber(Num x, Num y) => new Num(float.MinMagnitudeNumber(x._value, y._value));

    // Add parsing methods
    public static Num Parse(ReadOnlySpan<char> s, NumberStyles style, IFormatProvider? provider)
        => new Num(float.Parse(s, style, provider));
    
    public static Num Parse(string s, NumberStyles style, IFormatProvider? provider)
        => new Num(float.Parse(s, style, provider));

    public static bool TryParse(ReadOnlySpan<char> s, NumberStyles style, IFormatProvider? provider, out Num result)
    {
        if (float.TryParse(s, style, provider, out float value))
        {
            result = new Num(value);
            return true;
        }
        result = Zero;
        return false;
    }

    public static bool TryParse(string? s, NumberStyles style, IFormatProvider? provider, out Num result)
    {
        if (float.TryParse(s, style, provider, out float value))
        {
            result = new Num(value);
            return true;
        }
        result = Zero;
        return false;
    }

    // Add conversion methods
    public static bool TryConvertFromChecked<TOther>(TOther value, out Num result) where TOther : INumberBase<TOther>
    {
        try
        {
            result = new Num(Convert.ToSingle(value));
            return true;
        }
        catch
        {
            result = Zero;
            return false;
        }
    }

    public static bool TryConvertFromSaturating<TOther>(TOther value, out Num result) where TOther : INumberBase<TOther>
    {
        try
        {
            float val = Convert.ToSingle(value);
            result = new Num(float.IsNaN(val) ? 0 : val);
            return true;
        }
        catch
        {
            result = Zero;
            return false;
        }
    }

    public static bool TryConvertFromTruncating<TOther>(TOther value, out Num result) where TOther : INumberBase<TOther>
    {
        try
        {
            result = new Num((float)Math.Truncate(Convert.ToDouble(value)));
            return true;
        }
        catch
        {
            result = Zero;
            return false;
        }
    }

    public static bool TryConvertToChecked<TOther>(Num value, out TOther result) where TOther : INumberBase<TOther>
    {
        try
        {
            result = TOther.CreateChecked(value._value);
            return true;
        }
        catch
        {
            result = TOther.Zero;
            return false;
        }
    }

    public static bool TryConvertToSaturating<TOther>(Num value, out TOther result) where TOther : INumberBase<TOther>
    {
        try
        {
            result = TOther.CreateSaturating(value._value);
            return true;
        }
        catch
        {
            result = TOther.Zero;
            return false;
        }
    }

    public static bool TryConvertToTruncating<TOther>(Num value, out TOther result) where TOther : INumberBase<TOther>
    {
        try
        {
            result = TOther.CreateTruncating(value._value);
            return true;
        }
        catch
        {
            result = TOther.Zero;
            return false;
        }
    }

    // Add INumberBase<Num> static property
    public static int Radix => 2;

    // Add ISpanFormattable implementation
    public bool TryFormat(Span<char> destination, out int charsWritten, ReadOnlySpan<char> format, IFormatProvider? provider)
        => _value.TryFormat(destination, out charsWritten, format, provider);

    // Add IFormattable implementation
    public string ToString(string? format, IFormatProvider? formatProvider)
        => _value.ToString(format, formatProvider);

    // Add ISpanParsable<Num> implementation
    public static Num Parse(ReadOnlySpan<char> s, IFormatProvider? provider)
        => new Num(float.Parse(s, provider));

    public static bool TryParse(ReadOnlySpan<char> s, IFormatProvider? provider, out Num result)
    {
        if (float.TryParse(s, provider, out float value))
        {
            result = new Num(value);
            return true;
        }
        result = Zero;
        return false;
    }

    // Add a custom power function for integer exponents
    public static Num Pow(Num value, int exponent)
    {
        if (exponent == 0) return One;
        if (exponent < 0) return One / Pow(value, -exponent);
        
        Num result = One;
        Num base_value = value;
        
        while (exponent > 0)
        {
            if ((exponent & 1) == 1)
                result *= base_value;
            base_value *= base_value;
            exponent >>= 1;
        }
        
        return result;
    }

    // Add factorial calculation for Taylor series
    private static Num Factorial(int n)
    {
        Num result = One;
        for (int i = 2; i <= n; i++)
            result *= i;
        return result;
    }
}