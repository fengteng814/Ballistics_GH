// #! csharp
// TrapSkeet_Modules.csx
// Unified modules for coordinate system, rule tables, target tracks, aim directions,
// coupling analysis, visualization helpers, and data export.
//
// Inputs (GH C# Script ports / script variables):
//   Mode              : string   ("Trap" or "Skeet")
//   Origin            : Point3d  (world origin)
//   XDir              : Vector3d (X axis direction)
//   YDir              : Vector3d (Y axis direction)
//   GroundZ           : double
//   MuzzleHeight      : double
//   StationRadius     : double
//   StationSpacing    : double   (Trap only, used when StationRadius <= 0)
//   StationCountTrap  : int
//   StationCountSkeet : int
//   TrapTableId       : int
//   SkeetTableId      : int
//   TrackV0           : double   (target speed)
//   TrackTMin         : double
//   TrackTMax         : double
//   TrackSamples      : int
//   AimStride         : int
//   BuildAimDirs      : bool
//
// Outputs:
//   StationFootPts       (List<Point3d>)
//   StationMuzzlePts     (List<Point3d>)
//   RuleTable            (DataTree<string>)
//   TrackCurves          (DataTree<Curve>)
//   TrackPoints          (DataTree<Point3d>)
//   TrackTimes           (DataTree<double>)
//   AimDirs              (DataTree<Vector3d>)
//   AimRays              (DataTree<Curve>)
//   HitPoints            (DataTree<Point3d>)
//   HitDistances         (DataTree<double>)
//   HitParams            (DataTree<string>)
//   AimAzEl              (DataTree<string>)
//   UserText             (DataTree<string>)
//   ExportJson           (string)
//   ExportCsv            (string)
//   Debug                (string)

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

using Rhino.Geometry;

using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

// -------------------------------
// Robust cast helpers
// -------------------------------
static double ToDouble(object x, double defVal)
{
  if (x == null) return defVal;
  if (x is double d) return d;
  if (x is float f) return (double)f;
  if (x is int i) return i;
  if (x is long l) return l;
  if (x is GH_Number gn) return gn.Value;
  try { return Convert.ToDouble(x); } catch { return defVal; }
}

static int ToInt(object x, int defVal)
{
  if (x == null) return defVal;
  if (x is int i) return i;
  if (x is long l) return (int)l;
  if (x is double d) return (int)Math.Round(d);
  if (x is GH_Integer gi) return gi.Value;
  try { return Convert.ToInt32(x); } catch { return defVal; }
}

static bool ToBool(object x, bool defVal)
{
  if (x == null) return defVal;
  if (x is bool b) return b;
  if (x is GH_Boolean gb) return gb.Value;
  try { return Convert.ToBoolean(x); } catch { return defVal; }
}

static string ToStringSafe(object x, string defVal)
{
  if (x == null) return defVal;
  if (x is string s) return s;
  try { return x.ToString(); } catch { return defVal; }
}

static Point3d ToPoint3d(object x, Point3d defVal)
{
  if (x == null) return defVal;
  if (x is Point3d p) return p;
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

static Vector3d ToVector3d(object x, Vector3d defVal)
{
  if (x == null) return defVal;
  if (x is Vector3d v) return v;
  if (x is GH_Vector gv) return gv.Value;
  if (x is GH_ObjectWrapper ow && ow.Value is Vector3d) return (Vector3d)ow.Value;
  if (x is IGH_Goo goo)
  {
    try
    {
      object v = goo.ScriptVariable();
      if (v is Vector3d) return (Vector3d)v;
    }
    catch { }
  }
  return defVal;
}

static Vector3d SafeUnit(Vector3d v, Vector3d fallback)
{
  if (!v.IsValid) return fallback;
  if (v.IsZero) return fallback;
  v.Unitize();
  return v;
}

// -------------------------------
// Rule table models
// -------------------------------
class RuleEntry
{
  public string Mode;
  public int TableId;
  public int MachineId;
  public double AzimuthDeg;
  public double ElevationDeg;
  public double Speed;
  public RuleEntry(string mode, int tableId, int machineId, double azDeg, double elDeg, double speed)
  {
    Mode = mode;
    TableId = tableId;
    MachineId = machineId;
    AzimuthDeg = azDeg;
    ElevationDeg = elDeg;
    Speed = speed;
  }
}

static List<RuleEntry> BuildTrapRules(int tableId, double v0)
{
  // Simplified default tables: 3 machines
  // TableId 1: centered, TableId 2: left-bias, TableId 3: right-bias
  if (tableId <= 0) tableId = 1;

  var rules = new List<RuleEntry>();
  double[] az;
  switch (tableId)
  {
    case 2:
      az = new[] { -25.0, -10.0, 5.0 };
      break;
    case 3:
      az = new[] { -5.0, 10.0, 25.0 };
      break;
    default:
      az = new[] { -15.0, 0.0, 15.0 };
      break;
  }

  double[] el = new[] { 10.0, 12.0, 14.0 };

  for (int i = 0; i < 3; i++)
    rules.Add(new RuleEntry("Trap", tableId, i + 1, az[i], el[i], v0));

  return rules;
}

static List<RuleEntry> BuildSkeetRules(int tableId, double v0)
{
  // Simplified skeet tables: high house = 1, low house = 2
  if (tableId <= 0) tableId = 1;

  var rules = new List<RuleEntry>();
  if (tableId == 1)
  {
    rules.Add(new RuleEntry("Skeet", tableId, 1, 45.0, 4.0, v0));
    rules.Add(new RuleEntry("Skeet", tableId, 2, -45.0, 4.0, v0));
  }
  else
  {
    rules.Add(new RuleEntry("Skeet", tableId, 1, 55.0, 5.0, v0));
    rules.Add(new RuleEntry("Skeet", tableId, 2, -55.0, 5.0, v0));
  }

  return rules;
}

// -------------------------------
// Geometry helpers
// -------------------------------
static double DegToRad(double deg) { return deg * Math.PI / 180.0; }

static Vector3d DirFromAngles(Vector3d xAxis, Vector3d yAxis, Vector3d zAxis, double azDeg, double elDeg)
{
  double az = DegToRad(azDeg);
  double el = DegToRad(elDeg);

  double cosEl = Math.Cos(el);
  double sinEl = Math.Sin(el);
  double cosAz = Math.Cos(az);
  double sinAz = Math.Sin(az);

  return xAxis * (cosEl * sinAz) + yAxis * (cosEl * cosAz) + zAxis * sinEl;
}

static double HorizontalRange(Point3d a, Point3d b)
{
  double dx = b.X - a.X;
  double dy = b.Y - a.Y;
  return Math.Sqrt(dx * dx + dy * dy);
}

static string FmtVec(Vector3d v) => $"({v.X:0.###},{v.Y:0.###},{v.Z:0.###})";
static string FmtPt(Point3d p) => $"({p.X:0.###},{p.Y:0.###},{p.Z:0.###})";

// -------------------------------
// Read inputs
// -------------------------------
string mode = ToStringSafe(Mode, "Trap");
Point3d origin = ToPoint3d(Origin, Point3d.Origin);
Vector3d xAxis = SafeUnit(ToVector3d(XDir, Vector3d.XAxis), Vector3d.XAxis);
Vector3d yAxis = SafeUnit(ToVector3d(YDir, Vector3d.YAxis), Vector3d.YAxis);
Vector3d zAxis = Vector3d.CrossProduct(xAxis, yAxis);
if (zAxis.IsZero) zAxis = Vector3d.ZAxis;
else zAxis.Unitize();

double groundZ = ToDouble(GroundZ, origin.Z);
double muzzleHeight = ToDouble(MuzzleHeight, 1.5);
double stationRadius = ToDouble(StationRadius, 0.0);
double stationSpacing = ToDouble(StationSpacing, 3.0);
int stationCountTrap = ToInt(StationCountTrap, 5);
int stationCountSkeet = ToInt(StationCountSkeet, 8);
int trapTableId = ToInt(TrapTableId, 1);
int skeetTableId = ToInt(SkeetTableId, 1);

double trackV0 = ToDouble(TrackV0, 25.0);
double tMin = ToDouble(TrackTMin, 0.0);
double tMax = ToDouble(TrackTMax, 4.0);
int trackSamples = ToInt(TrackSamples, 81);
int aimStride = ToInt(AimStride, 5);
bool buildAimDirs = ToBool(BuildAimDirs, true);

if (muzzleHeight <= 0) muzzleHeight = 1.5;
if (stationSpacing <= 0) stationSpacing = 3.0;
if (trackSamples < 2) trackSamples = 81;
if (aimStride < 1) aimStride = 1;
if (tMax <= tMin) tMax = tMin + 4.0;

bool isTrap = mode.Equals("Trap", StringComparison.OrdinalIgnoreCase);

// -------------------------------
// Station layout
// -------------------------------
var stationFootPts = new List<Point3d>();
var stationMuzzlePts = new List<Point3d>();

int stationCount = isTrap ? stationCountTrap : stationCountSkeet;
if (stationCount < 1) stationCount = isTrap ? 5 : 8;

if (stationRadius > 0 && !isTrap)
{
  // Skeet: stations around a semicircle centered at origin (X/Y plane)
  double step = Math.PI / (stationCount - 1);
  for (int i = 0; i < stationCount; i++)
  {
    double angle = -Math.PI / 2.0 + step * i;
    Vector3d dir = xAxis * Math.Cos(angle) + yAxis * Math.Sin(angle);
    Point3d foot = origin + dir * stationRadius;
    Point3d muzzle = foot + zAxis * muzzleHeight;
    stationFootPts.Add(foot);
    stationMuzzlePts.Add(muzzle);
  }
}
else
{
  // Trap: linear stations along X axis
  for (int i = 0; i < stationCount; i++)
  {
    double offset = (i - (stationCount - 1) / 2.0) * stationSpacing;
    Point3d foot = origin + xAxis * offset;
    Point3d muzzle = foot + zAxis * muzzleHeight;
    stationFootPts.Add(foot);
    stationMuzzlePts.Add(muzzle);
  }
}

// -------------------------------
// Rule table generation
// -------------------------------
List<RuleEntry> rules = isTrap ? BuildTrapRules(trapTableId, trackV0) : BuildSkeetRules(skeetTableId, trackV0);

var ruleTableTree = new DataTree<string>();
for (int i = 0; i < rules.Count; i++)
{
  RuleEntry r = rules[i];
  var path = new GH_Path(r.TableId, r.MachineId);
  ruleTableTree.Add($"mode={r.Mode}", path);
  ruleTableTree.Add($"table={r.TableId}", path);
  ruleTableTree.Add($"machine={r.MachineId}", path);
  ruleTableTree.Add($"azimuthDeg={r.AzimuthDeg:0.###}", path);
  ruleTableTree.Add($"elevationDeg={r.ElevationDeg:0.###}", path);
  ruleTableTree.Add($"speed={r.Speed:0.###}", path);
}

// Trap filtering: only 3 opposite machines per station (centered around facing Y axis)
Func<int, List<RuleEntry>> trapFilter = stationId =>
{
  if (!isTrap) return rules;
  // Return 3 machines (id 1..3) always, but allow order by station
  var selected = new List<RuleEntry>();
  for (int i = 0; i < rules.Count; i++)
  {
    if (rules[i].MachineId == 1 || rules[i].MachineId == 2 || rules[i].MachineId == 3)
      selected.Add(rules[i]);
  }
  return selected;
};

// -------------------------------
// Track generation
// -------------------------------
var trackCurves = new DataTree<Curve>();
var trackPoints = new DataTree<Point3d>();
var trackTimes = new DataTree<double>();

int samples = trackSamples;
double dt = (tMax - tMin) / (samples - 1);

for (int i = 0; i < rules.Count; i++)
{
  RuleEntry r = rules[i];
  Vector3d dir = DirFromAngles(xAxis, yAxis, zAxis, r.AzimuthDeg, r.ElevationDeg);

  var pts = new List<Point3d>(samples);
  var path = new GH_Path(r.TableId, r.MachineId);

  for (int k = 0; k < samples; k++)
  {
    double t = tMin + dt * k;
    Point3d p = origin + dir * (r.Speed * t);
    pts.Add(p);
    trackPoints.Add(p, path);
    trackTimes.Add(t, path);
  }

  if (pts.Count >= 2)
    trackCurves.Add(new Polyline(pts).ToNurbsCurve(), path);
}

// -------------------------------
// AimDirs generation
// -------------------------------
var aimDirsTree = new DataTree<Vector3d>();
var aimRayTree = new DataTree<Curve>();

if (buildAimDirs)
{
  for (int sIdx = 0; sIdx < stationCount; sIdx++)
  {
    int stationId = sIdx + 1;
    Point3d muzzle = stationMuzzlePts[sIdx];
    List<RuleEntry> stationRules = isTrap ? trapFilter(stationId) : rules;

    foreach (RuleEntry r in stationRules)
    {
      Vector3d dir = DirFromAngles(xAxis, yAxis, zAxis, r.AzimuthDeg, r.ElevationDeg);
      var pathBase = new GH_Path(stationId, r.TableId, r.MachineId);

      for (int k = 0; k < samples; k += aimStride)
      {
        double t = tMin + dt * k;
        Point3d target = origin + dir * (r.Speed * t);
        Vector3d aim = target - muzzle;
        if (aim.IsZero) continue;
        aim.Unitize();

      var indices = new int[pathBase.Indices.Length + 1];
      Array.Copy(pathBase.Indices, indices, pathBase.Indices.Length);
      indices[indices.Length - 1] = k;
      var path = new GH_Path(indices);
        aimDirsTree.Add(aim, path);
        aimRayTree.Add(new Line(muzzle, target).ToNurbsCurve(), path);
      }
    }
  }
}

// -------------------------------
// Coupling analysis (ray/track proximity)
// -------------------------------
var hitPoints = new DataTree<Point3d>();
var hitDistances = new DataTree<double>();
var hitParams = new DataTree<string>();
var aimAzEl = new DataTree<string>();

foreach (GH_Path path in aimDirsTree.Paths)
{
  IList<Vector3d> aims = aimDirsTree.get_Branch(path) as IList<Vector3d>;
  if (aims == null || aims.Count == 0) continue;

  int stationId = path.Indices.Length > 0 ? path.Indices[0] : 1;
  int tableId = path.Indices.Length > 1 ? path.Indices[1] : 1;
  int machineId = path.Indices.Length > 2 ? path.Indices[2] : 1;

  Point3d muzzle = stationMuzzlePts[Math.Max(0, Math.Min(stationId - 1, stationMuzzlePts.Count - 1))];

  GH_Path trackPath = new GH_Path(tableId, machineId);
  IList<Point3d> trackPts = trackPoints.get_Branch(trackPath) as IList<Point3d>;
  if (trackPts == null || trackPts.Count == 0) continue;

  foreach (Vector3d aim in aims)
  {
    double s = 0.0;
    Point3d hit = Point3d.Unset;
    double t = 0.0;

    // Closest point on sampled track to the ray
    double bestDist = double.MaxValue;
    int bestIdx = 0;
    Point3d bestPt = Point3d.Unset;

    for (int i = 0; i < trackPts.Count; i++)
    {
      Point3d trackPt = trackPts[i];
      Vector3d diff = trackPt - muzzle;
      double sProj = diff * aim;
      Point3d rayPt = muzzle + aim * sProj;
      double dist = rayPt.DistanceTo(trackPt);
      if (dist < bestDist)
      {
        bestDist = dist;
        bestIdx = i;
        bestPt = trackPt;
        s = sProj;
      }
    }

    hit = bestPt;
    t = tMin + dt * bestIdx;
    double distance = bestDist;

    hitPoints.Add(hit, path);
    hitDistances.Add(distance, path);
    hitParams.Add($"track_t={t:0.###}, ray_s={s:0.###}", path);

    double az = Math.Atan2(aim * xAxis, aim * yAxis) * 180.0 / Math.PI;
    double el = Math.Asin(aim * zAxis) * 180.0 / Math.PI;
    aimAzEl.Add($"az={az:0.###}, el={el:0.###}", path);
  }
}

// -------------------------------
// Visualization metadata (UserText)
// -------------------------------
var userTextTree = new DataTree<string>();
for (int sIdx = 0; sIdx < stationCount; sIdx++)
{
  int stationId = sIdx + 1;
  var path = new GH_Path(stationId);
  userTextTree.Add($"station={stationId}", path);
  userTextTree.Add($"muzzle_height={muzzleHeight:0.###}", path);
  userTextTree.Add($"ground_z={groundZ:0.###}", path);
}

// -------------------------------
// Export module (JSON + CSV summary)
// -------------------------------
var summaryRows = new List<string>();
summaryRows.Add("station_id,rule_machine,max_range");

double globalMaxRange = 0.0;
int globalStation = 0;
int globalMachine = 0;

for (int sIdx = 0; sIdx < stationCount; sIdx++)
{
  int stationId = sIdx + 1;
  Point3d muzzle = stationMuzzlePts[sIdx];

  foreach (RuleEntry r in rules)
  {
    GH_Path trackPath = new GH_Path(r.TableId, r.MachineId);
    IList<Point3d> pts = trackPoints.get_Branch(trackPath) as IList<Point3d>;
    if (pts == null || pts.Count == 0) continue;

    double maxRange = 0.0;
    foreach (Point3d p in pts)
      maxRange = Math.Max(maxRange, HorizontalRange(muzzle, p));

    summaryRows.Add($"{stationId},{r.MachineId},{maxRange:0.###}");

    if (maxRange > globalMaxRange)
    {
      globalMaxRange = maxRange;
      globalStation = stationId;
      globalMachine = r.MachineId;
    }
  }
}

string json = "{" +
  $"\"mode\":\"{mode}\"," +
  $"\"origin\":\"{FmtPt(origin)}\"," +
  $"\"axis_x\":\"{FmtVec(xAxis)}\"," +
  $"\"axis_y\":\"{FmtVec(yAxis)}\"," +
  $"\"ground_z\":{groundZ:0.###}," +
  $"\"muzzle_height\":{muzzleHeight:0.###}," +
  $"\"track_v0\":{trackV0:0.###}," +
  $"\"track_tmin\":{tMin:0.###}," +
  $"\"track_tmax\":{tMax:0.###}," +
  $"\"table_id\":{(isTrap ? trapTableId : skeetTableId)}," +
  $"\"max_range_station\":{globalStation}," +
  $"\"max_range_machine\":{globalMachine}," +
  $"\"max_range\":{globalMaxRange:0.###}," +
  $"\"version\":\"v1\"" +
  "}";

string csv = string.Join("\n", summaryRows);

// -------------------------------
// Outputs
// -------------------------------
StationFootPts = stationFootPts;
StationMuzzlePts = stationMuzzlePts;
RuleTable = ruleTableTree;
TrackCurves = trackCurves;
TrackPoints = trackPoints;
TrackTimes = trackTimes;
AimDirs = aimDirsTree;
AimRays = aimRayTree;
HitPoints = hitPoints;
HitDistances = hitDistances;
HitParams = hitParams;
AimAzEl = aimAzEl;
UserText = userTextTree;
ExportJson = json;
ExportCsv = csv;

Debug =
  "Trap/Skeet modules (unified)\n" +
  $"mode={mode}, origin={FmtPt(origin)}\n" +
  $"axis_x={FmtVec(xAxis)}, axis_y={FmtVec(yAxis)}, axis_z={FmtVec(zAxis)}\n" +
  $"stations={stationCount}, stationRadius={stationRadius:0.###}, stationSpacing={stationSpacing:0.###}\n" +
  $"track_t=[{tMin:0.###},{tMax:0.###}], samples={samples}, aimStride={aimStride}\n" +
  $"rules={rules.Count}, trapTableId={trapTableId}, skeetTableId={skeetTableId}\n" +
  $"maxRange={globalMaxRange:0.###} (station {globalStation}, machine {globalMachine})";
