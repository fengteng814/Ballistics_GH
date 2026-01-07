// 02_TrapScene_WithLabels_pure.csx
// Step 1b — Trap 场景增强（脚点/枪口/竖线/TextDot/边界靶轨/AimLines）
// 说明：本脚本为“纯 .csx”，不包含 Script_Instance/RunScript。GH 端请把端口命名为下列变量名。
// Inputs:
//   TrapPlane        : Plane        — 坐标系（默认 WorldXY）
//   stationSpacing   : double       — 站位间距（m，默认 3.0）
//   muzzleHeight     : double       — 枪口高度（m，默认 1.5）
//   trapHouseOrigin  : Point3d      — 靶房出靶点（默认 TrapPlane.Origin + Y*10）
//   alphaL/alphaR    : double       — 水平边界角（deg，0 指向 +Y，正向朝 +X）
//   betaLow/betaHigh : double       — 仰角边界（deg）
//   v0               : double       — 靶速（m/s，默认 25）
//   tMin/tMax        : double       — 时间范围（s，默认 [0,4]）
//   samples          : int          — 每条靶轨采样点数（默认 81）
//   buildAimLines    : bool         — 是否绘制瞄准线
//   aimStride        : int          — 瞄准线抽样步长（默认 10）
//   axisLen          : double       — 坐标轴显示长度（默认 10）
//
// Outputs:
//   Axes, StationsFoot, StationsMuzzle, StationUpLines, StationLabels, TargetBoundaryCurves, AimLines, Debug

using System;
using System.Collections;
using System.Collections.Generic;

using Rhino;
using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

// ---------------- robust cast helpers (避免必须设置 Type Hint) ----------------
static double ToDouble(object x, double defVal)
{
  if (x == null) return defVal;
  if (x is double) return (double)x;
  if (x is float) return (double)(float)x;
  if (x is int) return (double)(int)x;
  if (x is long) return (double)(long)x;
  if (x is GH_Number gn) return gn.Value;
  try { return Convert.ToDouble(x); } catch { return defVal; }
}
static int ToInt(object x, int defVal)
{
  if (x == null) return defVal;
  if (x is int) return (int)x;
  if (x is long) return (int)(long)x;
  if (x is double) return (int)Math.Round((double)x);
  if (x is GH_Integer gi) return gi.Value;
  try { return Convert.ToInt32(x); } catch { return defVal; }
}
static bool ToBool(object x, bool defVal)
{
  if (x == null) return defVal;
  if (x is bool) return (bool)x;
  if (x is GH_Boolean gb) return gb.Value;
  try { return Convert.ToBoolean(x); } catch { return defVal; }
}
static Plane ToPlane(object x, Plane defVal)
{
  if (x == null) return defVal;
  if (x is Plane) return (Plane)x;
  if (x is GH_Plane gp) return gp.Value;
  if (x is GH_ObjectWrapper ow && ow.Value is Plane) return (Plane)ow.Value;
  if (x is IGH_Goo goo)
  {
    try
    {
      object v = goo.ScriptVariable();
      if (v is Plane) return (Plane)v;
    }
    catch { }
  }
  return defVal;
}
static Point3d ToPoint3d(object x, Point3d defVal)
{
  if (x == null) return defVal;
  if (x is Point3d) return (Point3d)x;
  if (x is GH_Point gp) return gp.Value;
  if (x is GH_ObjectWrapper ow && ow.Value is Point3d) return (Point3d)ow.Value;
  if (x is IGH_Goo goo)
  {
    try
    {
      object v = goo.ScriptVariable();
      if (v is Point3d) return (Point3d)v;
    }
    catch { }
  }
  return defVal;
}

// ---------------- read inputs (GH 会把端口变量当成 object 提供) ----------------
Plane _TrapPlane = ToPlane(TrapPlane, Plane.WorldXY);
double _stationSpacing = ToDouble(stationSpacing, 3.0);
double _muzzleHeight = ToDouble(muzzleHeight, 1.5);

double _alphaL = ToDouble(alphaL, -45.0);
double _alphaR = ToDouble(alphaR,  45.0);
double _betaLow  = ToDouble(betaLow,  2.0);
double _betaHigh = ToDouble(betaHigh, 18.0);

double _v0 = ToDouble(v0, 25.0);
double _tMin = ToDouble(tMin, 0.0);
double _tMax = ToDouble(tMax, _tMin + 4.0);
int _samples = ToInt(samples, 81);
bool _buildAimLines = ToBool(buildAimLines, true);
int _aimStride = ToInt(aimStride, 10);
double _axisLen = ToDouble(axisLen, 10.0);

if (_tMin < 0) _tMin = 0.0;
if (_tMax <= _tMin) _tMax = _tMin + 4.0;
if (_samples < 2) _samples = 81;
if (_aimStride < 1) _aimStride = 1;
if (_stationSpacing <= 0) _stationSpacing = 3.0;
if (_muzzleHeight <= 0) _muzzleHeight = 1.5;
if (_v0 <= 0) _v0 = 25.0;
if (_axisLen <= 0) _axisLen = 10.0;

// trapHouseOrigin fallback
Point3d _trapHouseOrigin = ToPoint3d(trapHouseOrigin, _TrapPlane.Origin + _TrapPlane.YAxis * 10.0);

// Ensure axes are unitized
Vector3d X = _TrapPlane.XAxis; X.Unitize();
Vector3d Y = _TrapPlane.YAxis; Y.Unitize();
Vector3d Z = _TrapPlane.ZAxis; Z.Unitize();
Point3d O = _TrapPlane.Origin;

// ---------------- 1) Axes geometry (3D) ----------------
var axes = new List<Curve>();
axes.Add(new Line(O, O + X * _axisLen).ToNurbsCurve()); // +X
axes.Add(new Line(O, O + Y * _axisLen).ToNurbsCurve()); // +Y
axes.Add(new Line(O, O + Z * _axisLen).ToNurbsCurve()); // +Z

// ---------------- 2) Stations (Trap 1..5) ----------------
var footPts = new List<Point3d>(5);
var muzzlePts = new List<Point3d>(5);
var upLines = new List<Curve>(5);
var labels = new List<TextDot>(5);

for (int stationId = 1; stationId <= 5; stationId++)
{
  double xOffset = (stationId - 3) * _stationSpacing;
  Point3d foot = O + X * xOffset;
  Point3d muzzle = foot + Z * _muzzleHeight;

  footPts.Add(foot);
  muzzlePts.Add(muzzle);

  upLines.Add(new Line(foot, muzzle).ToNurbsCurve());
  labels.Add(new TextDot($"S{stationId}", foot));
}

// ---------------- 3) Target boundary curves (alpha/beta edges) ----------------
double[] alphas = new double[] { _alphaL, _alphaR };
double[] betas  = new double[] { _betaLow, _betaHigh };

var targetCurves = new GH_Structure<GH_Curve>();
int targetCurvesBuilt = 0;

double dt = (_tMax - _tMin) / (_samples - 1);

for (int ai = 0; ai < 2; ai++)
{
  for (int bi = 0; bi < 2; bi++)
  {
    double aRad = DegToRad(alphas[ai]);
    double bRad = DegToRad(betas[bi]);

    Vector3d dir = DirFromAlphaBeta(X, Y, Z, aRad, bRad);

    var pts = new List<Point3d>(_samples);
    for (int k = 0; k < _samples; k++)
    {
      double t = _tMin + dt * k;
      Point3d p = _trapHouseOrigin + dir * (_v0 * t);
      pts.Add(p);
    }

    if (pts.Count >= 2)
    {
      Curve c = new Polyline(pts).ToNurbsCurve();
      var path = new GH_Path(ai, bi); // {alphaIndex; betaIndex}
      targetCurves.Append(new GH_Curve(c), path);
      targetCurvesBuilt++;
    }
  }
}

// ---------------- 4) Optional: Aim lines (muzzle -> target point samples) ----------------
var aimTree = new GH_Structure<GH_Curve>();
int aimLinesBuilt = 0;

if (_buildAimLines)
{
  for (int sIdx = 0; sIdx < 5; sIdx++)
  {
    Point3d muzzle = muzzlePts[sIdx];
    int stationId = sIdx + 1;

    for (int ai = 0; ai < 2; ai++)
    {
      for (int bi = 0; bi < 2; bi++)
      {
        double aRad = DegToRad(alphas[ai]);
        double bRad = DegToRad(betas[bi]);
        Vector3d dir = DirFromAlphaBeta(X, Y, Z, aRad, bRad);

        var path = new GH_Path(stationId, ai, bi);

        for (int k = 0; k < _samples; k += _aimStride)
        {
          double t = _tMin + dt * k;
          Point3d target = _trapHouseOrigin + dir * (_v0 * t);

          Curve ray = new Line(muzzle, target).ToNurbsCurve();
          aimTree.Append(new GH_Curve(ray), path);
          aimLinesBuilt++;
        }
      }
    }
  }
}

// ---------------- outputs ----------------
Axes = axes;
StationsFoot = footPts;
StationsMuzzle = muzzlePts;
StationUpLines = upLines;
StationLabels = labels;
TargetBoundaryCurves = targetCurves;
AimLines = aimTree;

Debug =
  "Trap 3D unified model (robust-cast)\n" +
  $"TrapPlane Origin={FmtPt(O)}  X={FmtVec(X)}  Y={FmtVec(Y)}  Z={FmtVec(Z)}\n" +
  $"stationSpacing={_stationSpacing}, muzzleHeight={_muzzleHeight}\n" +
  $"trapHouseOrigin={FmtPt(_trapHouseOrigin)}\n" +
  $"alphaL={_alphaL}°, alphaR={_alphaR}°, betaLow={_betaLow}°, betaHigh={_betaHigh}°\n" +
  $"v0={_v0} m/s, tMin={_tMin}s, tMax={_tMax}s, samples={_samples}\n" +
  $"targetCurvesBuilt={targetCurvesBuilt}\n" +
  $"buildAimLines={_buildAimLines}, aimStride={_aimStride}, aimLinesBuilt={aimLinesBuilt}\n" +
  "TargetBoundaryCurves tree paths:\n" +
  "  {0;0}=alphaL+betaLow, {0;1}=alphaL+betaHigh, {1;0}=alphaR+betaLow, {1;1}=alphaR+betaHigh\n" +
  "AimLines tree paths (if enabled): {stationId; alphaIndex; betaIndex}";

// ---------------- helpers ----------------
static double DegToRad(double deg) { return deg * Math.PI / 180.0; }

static Vector3d DirFromAlphaBeta(Vector3d X, Vector3d Y, Vector3d Z, double aRad, double bRad)
{
  double cb = Math.Cos(bRad);
  double sb = Math.Sin(bRad);
  double sa = Math.Sin(aRad);
  double ca = Math.Cos(aRad);

  Vector3d dir =
    X * (sa * cb) +
    Y * (ca * cb) +
    Z * (sb);

  return dir;
}

static string FmtPt(Point3d p) { return $"({p.X:0.###},{p.Y:0.###},{p.Z:0.###})"; }
static string FmtVec(Vector3d v) { return $"({v.X:0.###},{v.Y:0.###},{v.Z:0.###})"; }
