// #! csharp
using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

using Rhino.Geometry;

/*
Inputs (GH C# Script ports / script variables):
  Muzzle (Point3d)
  AzimuthDeg (double)
  ElevationsDeg (List<double> or IEnumerable)
  V0 (double)
  Cd (double)
  MassKg (double)
  DiameterM (double)
  Rho (double)
  Wind (Vector3d)
  Dt (double)
  TMax (double)
  GroundZ (double)
  ScatterN (int)
  SigmaV0 (double)
  SigmaCd (double)
  Seed (int)

Outputs:
  TrajCurves (DataTree<Curve>)
  TrajPts    (DataTree<Point3d>)
  ImpactPts  (DataTree<Point3d>)
  Summary    (DataTree<string>)

Tip:
  为了减少转换问题，建议给这些端口设置 Type Hint（如 Muzzle=Point3d、Wind=Vector3d、ElevationsDeg=List<double>、其余数值=double/int）。
*/

// -----------------------------
// 0) Robust input conversion
// -----------------------------
Point3d muzzle = ToPoint3d(Muzzle, Point3d.Origin);
double azimuthDeg = ToDouble(AzimuthDeg, 0.0);
List<double> elevationsDeg = ToDoubleList(ElevationsDeg);
double v0Nominal = ToDouble(V0, 370.0);
double cdNominal = ToDouble(Cd, 0.3);
double massKg = ToDouble(MassKg, 0.024);
double diameterM = ToDouble(DiameterM, 0.002);
double rho = ToDouble(Rho, 1.225);
Vector3d wind = ToVector3d(Wind, Vector3d.Zero);
double dt = ToDouble(Dt, 0.01);
double tMax = ToDouble(TMax, 12.0);
double groundZ = ToDouble(GroundZ, 0.0);
int scatterN = ToInt(ScatterN, 0);
double sigmaV0 = ToDouble(SigmaV0, 0.0);
double sigmaCd = ToDouble(SigmaCd, 0.0);
int seed = ToInt(Seed, 0);

// -----------------------------
// 1) Parameter defense (same intent as original RunScript)
// -----------------------------
if (elevationsDeg == null || elevationsDeg.Count == 0)
  elevationsDeg = new List<double>() { 0.0 };

if (dt <= 0) dt = 0.01;
if (tMax <= 0) tMax = 12.0;
if (massKg <= 0) massKg = 0.024;       // 24g 默认
if (diameterM <= 0) diameterM = 0.002; // 2mm 粗略默认
if (rho <= 0) rho = 1.225;

double area = Math.PI * 0.25 * diameterM * diameterM; // 圆截面积
int nSamples = (scatterN <= 0) ? 1 : (scatterN + 1);  // sample 0 = 名义；后面是扰动
var rng = new Random(seed);

var outCurves = new DataTree<Curve>();
var outPts = new DataTree<Point3d>();
var outImpact = new DataTree<Point3d>();
var outSummary = new DataTree<string>();

// -----------------------------
// 2) Main loop: elevation & scatter
// -----------------------------
for (int eIdx = 0; eIdx < elevationsDeg.Count; eIdx++)
{
  double elevDegNominal = elevationsDeg[eIdx];

  for (int sIdx = 0; sIdx < nSamples; sIdx++)
  {
    double v0 = v0Nominal;
    double cd = cdNominal;

    if (sIdx > 0)
    {
      // 正态扰动：Box-Muller
      v0 = v0Nominal + sigmaV0 * NextGaussian(rng);
      cd = cdNominal + sigmaCd * NextGaussian(rng);
      if (v0 < 1) v0 = 1;
      if (cd < 0.01) cd = 0.01;
    }

    Vector3d vInit = VelocityFromAngles(v0, azimuthDeg, elevDegNominal);

    List<TrajPoint> traj = IntegrateRK4(
      muzzle, vInit, dt, tMax, rho, cd, area, massKg, wind, groundZ
    );

    var path = new GH_Path(eIdx, sIdx);

    var pts = new List<Point3d>(traj.Count);
    foreach (var tp in traj) pts.Add(tp.P);

    // polyline curve
    if (pts.Count >= 2)
    {
      var pl = new Polyline(pts);
      Curve c = pl.ToNurbsCurve();
      outCurves.Add(c, path);
    }

    foreach (var p in pts) outPts.Add(p, path);

    // impact: last point (touched ground or reached tMax)
    Point3d impact = pts[pts.Count - 1];
    outImpact.Add(impact, path);

    // summary
    double flightT = traj[traj.Count - 1].T;
    double maxZ = double.NegativeInfinity;
    double maxRange = 0.0;
    for (int i = 0; i < traj.Count; i++)
    {
      maxZ = Math.Max(maxZ, traj[i].P.Z);
      double range = HorizontalRange(muzzle, traj[i].P);
      if (range > maxRange) maxRange = range;
    }

    string line =
      $"elevDeg={elevDegNominal:0.###}, sample={sIdx}, V0={v0:0.###} m/s, Cd={cd:0.###}, " +
      $"T={flightT:0.###} s, maxZ={maxZ:0.###} m, maxRange={maxRange:0.###} m";
    outSummary.Add(line, path);
  }
}

// -----------------------------
// 3) Assign outputs (must match GH output port names)
// -----------------------------
TrajCurves = outCurves;
TrajPts = outPts;
ImpactPts = outImpact;
Summary = outSummary;

// ====================================================================
// Helpers (ported from original Script_Instance)
// ====================================================================
struct TrajPoint
{
  public Point3d P;
  public Vector3d V;
  public double T;
  public TrajPoint(Point3d p, Vector3d v, double t) { P = p; V = v; T = t; }
}

static double NextGaussian(Random rng)
{
  // Box-Muller
  double u1 = 1.0 - rng.NextDouble();
  double u2 = 1.0 - rng.NextDouble();
  return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
}

static Vector3d VelocityFromAngles(double v0, double azDeg, double elDeg)
{
  double az = DegToRad(azDeg);
  double el = DegToRad(elDeg);

  double cosEl = Math.Cos(el);
  double vx = v0 * cosEl * Math.Cos(az);
  double vy = v0 * cosEl * Math.Sin(az);
  double vz = v0 * Math.Sin(el);
  return new Vector3d(vx, vy, vz);
}

static double DegToRad(double d) { return d * Math.PI / 180.0; }

static double HorizontalRange(Point3d a, Point3d b)
{
  double dx = b.X - a.X;
  double dy = b.Y - a.Y;
  return Math.Sqrt(dx * dx + dy * dy);
}

static Vector3d Accel(Vector3d v, Vector3d wind, double rho, double cd, double area, double mass)
{
  Vector3d g = new Vector3d(0, 0, -9.80665);

  // 相对风速（空气相对弹丸）
  Vector3d vRel = v - wind;
  double speed = vRel.Length;
  if (speed < 1e-9) return g;

  // a_drag = - (0.5*rho*Cd*A/m) * |v| * v
  double k = 0.5 * rho * cd * area / mass;
  Vector3d aDrag = -k * speed * vRel;
  return g + aDrag;
}

static List<TrajPoint> IntegrateRK4(
  Point3d p0, Vector3d v0, double dt, double tMax,
  double rho, double cd, double area, double mass,
  Vector3d wind, double groundZ
)
{
  var res = new List<TrajPoint>(4096);

  Point3d p = p0;
  Vector3d v = v0;
  double t = 0.0;

  res.Add(new TrajPoint(p, v, t));

  int guard = 0;
  while (t < tMax && guard < 2_000_000)
  {
    guard++;

    // 若已触地，停止（保留最后点）
    if (p.Z <= groundZ && t > 0) break;

    // RK4 for (p,v)
    // dp/dt = v
    // dv/dt = a(v)
    Vector3d a1 = Accel(v, wind, rho, cd, area, mass);
    Vector3d k1v = a1 * dt;
    Vector3d k1p = v * dt;

    Vector3d v2 = v + 0.5 * k1v;
    Vector3d a2 = Accel(v2, wind, rho, cd, area, mass);
    Vector3d k2v = a2 * dt;
    Vector3d k2p = (v + 0.5 * k1v) * dt;

    Vector3d v3 = v + 0.5 * k2v;
    Vector3d a3 = Accel(v3, wind, rho, cd, area, mass);
    Vector3d k3v = a3 * dt;
    Vector3d k3p = (v + 0.5 * k2v) * dt;

    Vector3d v4 = v + k3v;
    Vector3d a4 = Accel(v4, wind, rho, cd, area, mass);
    Vector3d k4v = a4 * dt;
    Vector3d k4p = (v + k3v) * dt;

    Vector3d dv = (k1v + 2 * k2v + 2 * k3v + k4v) / 6.0;
    Vector3d dp = (k1p + 2 * k2p + 2 * k3p + k4p) / 6.0;

    v = v + dv;
    p = p + dp;
    t += dt;

    res.Add(new TrajPoint(p, v, t));

    // 防止异常发散
    if (!p.IsValid || !v.IsValid) break;
    if (Math.Abs(p.X) > 1e6 || Math.Abs(p.Y) > 1e6 || Math.Abs(p.Z) > 1e6) break;
    if (v.Length > 5000) break;
  }

  return res;
}

// -----------------------------
// Converters (GH friendly)
// -----------------------------
static double ToDouble(object v, double defVal)
{
  if (v == null) return defVal;
  if (v is double d) return d;
  if (v is float f) return f;
  if (v is int i) return i;
  if (v is long l) return l;
  if (v is GH_Number ghn) return ghn.Value;
  if (v is IConvertible c)
  {
    try { return Convert.ToDouble(c); } catch { }
  }
  return defVal;
}

static int ToInt(object v, int defVal)
{
  if (v == null) return defVal;
  if (v is int i) return i;
  if (v is long l) return (int)l;
  if (v is GH_Integer ghi) return ghi.Value;
  if (v is GH_Number ghn) return (int)Math.Round(ghn.Value);
  if (v is IConvertible c)
  {
    try { return Convert.ToInt32(c); } catch { }
  }
  return defVal;
}

static Point3d ToPoint3d(object v, Point3d defVal)
{
  if (v == null) return defVal;
  if (v is Point3d p) return p;
  if (v is GH_Point ghp) return ghp.Value;
  return defVal;
}

static Vector3d ToVector3d(object v, Vector3d defVal)
{
  if (v == null) return defVal;
  if (v is Vector3d vv) return vv;
  if (v is GH_Vector ghv) return ghv.Value;
  return defVal;
}

static List<double> ToDoubleList(object v)
{
  if (v == null) return null;

  if (v is List<double> ld) return ld;
  if (v is double d) return new List<double> { d };
  if (v is GH_Number ghn) return new List<double> { ghn.Value };

  if (v is IEnumerable<double> ed) return ed.ToList();

  if (v is IEnumerable en)
  {
    var res = new List<double>();
    foreach (var item in en)
      res.Add(ToDouble(item, 0.0));
    return res;
  }

  return null;
}
