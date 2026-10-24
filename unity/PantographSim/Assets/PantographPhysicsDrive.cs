// PantographPhysicsDrive.cs - PATH B (full physics) drive for the five-bar.
//
// Drives the KINEMATIC cranks to the ROS 2 position commands
//   /left_motor/cmd_pos , /right_motor/cmd_pos   (Float64, rad - q=0 at home),
// with MoveRotation in FixedUpdate. The DYNAMIC arms follow through the joints,
// and the closing ConfigurableJoint (PantographSetup) keeps them JOINED.
//
// Prerequisite: "Tools > Pantograph > Setup Five-Bar (Physics + Loop Closure)" has been run.
// PLACEMENT: on the root 'five_bar_mechanism'. (Remove the old PantographController
// - it makes everything kinematic, which cancels the physics of Path B.)
using System.IO;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class PantographPhysicsDrive : MonoBehaviour
{
    [Header("ROS Topics")]
    public string leftTopic  = "/left_motor/cmd_pos";
    public string rightTopic = "/right_motor/cmd_pos";

    [Header("Angle convention - adjust if an arm turns the wrong way")]
    public float leftSign  = 1f;
    public float rightSign = 1f;
    public float leftOffsetDeg  = 0f;
    public float rightOffsetDeg = 0f;

    [Header("Self-test (without ROS): sweep to check that it does NOT break")]
    public float selfTestSweepDeg = 0f;   // e.g. 8
    public float selfTestPeriod   = 4f;

    [Header("Physics tuning (applied in Awake)")]
    public float fixedDt   = 0.002f;      // 500 Hz - closed-loop stability (Gazebo: 1ms)
    public int   solverIters = 60;
    public int   velIters    = 40;

    [Header("CSV logging (gap_mm = how far the closure opens, should be ~ 0)")]
    public bool   logCsv = true;
    public string csvFileName = "unity_physics_log.csv";
    public float  logRateHz = 50f;

    Rigidbody leftCrankRb, rightCrankRb;
    Transform baseLink, leftCrank, rightCrank, rightArm, ee;
    Quaternion leftHome, rightHome;
    Vector3 planeN, tipLocalRight;

    float qLeft, qRight;
    bool haveRosData;

    StreamWriter csv;
    float tStart, lastLog = -999f;

    void Awake()
    {
        Time.fixedDeltaTime = Mathf.Max(0.0005f, fixedDt);
        Physics.defaultSolverIterations = Mathf.Max(6, solverIters);
        Physics.defaultSolverVelocityIterations = Mathf.Max(1, velIters);
    }

    void Start()
    {
        baseLink   = FindChild("base_link");
        leftCrank  = FindChild("left_crank");
        rightCrank = FindChild("right_crank");
        rightArm   = FindChild("right_arm");
        ee         = FindChild("end_effector_link");
        var leftArm = FindChild("left_arm");
        if (!leftCrank || !rightCrank || !rightArm || !ee || !leftArm)
        { Debug.LogError("[PantographDrive] Links not found. Did you run Setup? Is the script on 'five_bar_mechanism'?"); enabled = false; return; }

        leftCrankRb  = leftCrank.GetComponent<Rigidbody>();
        rightCrankRb = rightCrank.GetComponent<Rigidbody>();
        if (!leftCrankRb || !rightCrankRb)
        { Debug.LogError("[PantographDrive] The cranks have no Rigidbody - run Setup first (Path B)."); enabled = false; return; }
        leftCrankRb.isKinematic = true;
        rightCrankRb.isKinematic = true;

        // Motor rotation axis = axis of the elbow hinge (plane normal)
        var elbow = leftArm.GetComponent<HingeJoint>();
        planeN = elbow != null ? leftArm.TransformDirection(elbow.axis).normalized : Vector3.up;

        leftHome  = leftCrankRb.rotation;
        rightHome = rightCrankRb.rotation;

        // closure point on the right arm (coincides with the EE at home) -> for measuring the gap
        tipLocalRight = rightArm.InverseTransformPoint(ee.position);

        var ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float64Msg>(leftTopic,  m => { qLeft  = (float)m.data; haveRosData = true; });
        ros.Subscribe<Float64Msg>(rightTopic, m => { qRight = (float)m.data; haveRosData = true; });

        if (logCsv) OpenCsv();
        tStart = Time.time;
        Debug.Log($"[PantographDrive] Ready (Path B). dt={Time.fixedDeltaTime*1000f:F1}ms, " +
                  $"solver={Physics.defaultSolverIterations}/{Physics.defaultSolverVelocityIterations}. " +
                  (selfTestSweepDeg > 0f ? "Self-test ACTIVE." : "Waiting for ROS."));
    }

    void FixedUpdate()
    {
        if (selfTestSweepDeg > 0f && !haveRosData)
        {
            float a = selfTestSweepDeg * Mathf.Deg2Rad *
                      Mathf.Sin(2f * Mathf.PI * (Time.time - tStart) / Mathf.Max(0.1f, selfTestPeriod));
            qLeft = a; qRight = -a;
        }
        float qL = qLeft  * leftSign  + leftOffsetDeg  * Mathf.Deg2Rad;
        float qR = qRight * rightSign + rightOffsetDeg * Mathf.Deg2Rad;

        // Drive the kinematic cranks; the dynamic arms + the loop closure follow.
        leftCrankRb.MoveRotation(Quaternion.AngleAxis(qL * Mathf.Rad2Deg, planeN) * leftHome);
        rightCrankRb.MoveRotation(Quaternion.AngleAxis(qR * Mathf.Rad2Deg, planeN) * rightHome);
    }

    void Update() { LogRow(); }

    void LogRow()
    {
        if (!logCsv || csv == null || Time.time - lastLog < 1f / Mathf.Max(1f, logRateHz)) return;
        lastLog = Time.time;
        Vector3 e = baseLink.InverseTransformPoint(ee.position);
        float gapMm = Vector3.Distance(rightArm.TransformPoint(tipLocalRight), ee.position) * 1000f;
        csv.WriteLine($"{Time.time - tStart:F4},{qLeft:F6},{qRight:F6},{e.x:F6},{e.y:F6},{e.z:F6},{gapMm:F3}");
    }

    Transform FindChild(string n)
    {
        foreach (var t in GetComponentsInChildren<Transform>(true))
            if (t.name == n) return t;
        return null;
    }

    void OpenCsv()
    {
        string path = Path.Combine(Application.dataPath, "..", csvFileName);
        csv = new StreamWriter(path, false) { AutoFlush = true };
        csv.WriteLine("t,q_left,q_right,ee_x,ee_y,ee_z,gap_mm");
        Debug.Log("[PantographDrive] CSV -> " + Path.GetFullPath(path));
    }

    void OnDisable()
    {
        if (csv != null) { csv.Flush(); csv.Close(); csv = null; }
    }
}
