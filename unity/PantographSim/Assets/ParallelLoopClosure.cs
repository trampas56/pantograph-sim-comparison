// ParallelLoopClosure.cs - Runtime controller for the five-bar in Unity.
// Drives the two motors from the ROS 2 position commands
//   /left_motor/cmd_pos , /right_motor/cmd_pos   (Float64, rad - q=0 at home),
// like the Gazebo / MuJoCo / Webots sims, and logs a CSV for plots.
//
// LOOP CLOSURE - ANALYTICAL (arms guaranteed to stay joined):
//   PhysX is unreliable with closed loops AND the URDF has a wrong home pose
//   (the right arm does not reach the EE). So we ignore the URDF home and compute
//   the CORRECT closed configuration: with the actual arm lengths (0.12) and the
//   elbow positions (from the cranks), we find the EE as the intersection of two circles and turn the two
//   arms to point there -> their tips COINCIDE. All bodies become kinematic.
//
// PLACEMENT: on the root GameObject 'five_bar_mechanism'.
using System.IO;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class PantographController : MonoBehaviour
{
    [Header("ROS Topics")]
    public string leftTopic  = "/left_motor/cmd_pos";
    public string rightTopic = "/right_motor/cmd_pos";

    [Header("Angle convention - adjust if an arm turns the wrong way")]
    public float leftSign  = 1f;
    public float rightSign = 1f;
    public float leftOffsetDeg  = 0f;
    public float rightOffsetDeg = 0f;

    [Header("Self-test (without ROS): sweep to see the motion")]
    public float selfTestSweepDeg = 0f;   // e.g. 20
    public float selfTestPeriod   = 4f;

    [Header("CSV logging")]
    public bool   logCsv = true;
    public string csvFileName = "unity_trajectory_log.csv";
    public float  logRateHz = 50f;

    Transform baseLink, leftCrank, rightCrank, leftArm, rightArm, ee;
    Quaternion leftHome, rightHome, leftArmHome, rightArmHome;
    Vector3 leftAxisW, rightAxisW, planeN, refCenter;

    Vector3 tipLocal, armDirLHome, armDirRHome;   // for the analytical solver
    float armLen;

    float qLeft, qRight;
    bool haveRosData;

    StreamWriter csv;
    float tStart, lastLog = -999f;

    void Start()
    {
        baseLink   = FindChild("base_link");
        leftCrank  = FindChild("left_crank");
        rightCrank = FindChild("right_crank");
        leftArm    = FindChild("left_arm");
        rightArm   = FindChild("right_arm");
        ee         = FindChild("end_effector_link");
        if (!baseLink || !leftCrank || !rightCrank || !leftArm || !rightArm || !ee)
        { Debug.LogError("[Pantograph] Links not found. Put the script on 'five_bar_mechanism'."); enabled = false; return; }

        foreach (var t in new[] { baseLink, leftCrank, rightCrank, leftArm, rightArm, ee })
        {
            var rb = t.GetComponent<Rigidbody>();
            if (rb != null) { rb.isKinematic = true; rb.interpolation = RigidbodyInterpolation.None; }
        }

        leftHome     = leftCrank.rotation;
        rightHome    = rightCrank.rotation;
        leftArmHome  = leftArm.rotation;
        rightArmHome = rightArm.rotation;
        leftAxisW  = JointAxisWorld(leftCrank);
        rightAxisW = JointAxisWorld(rightCrank);
        planeN = leftAxisW;

        // ACTUAL arm length + local tip of the rod (the EE sits rigidly at the tip of the left arm)
        tipLocal = leftArm.InverseTransformPoint(ee.position);   // |tipLocal| = 0.12
        armLen   = tipLocal.magnitude;
        // actual rod directions at home (not towards the broken EE)
        armDirLHome = (leftArm.TransformPoint(tipLocal)  - leftArm.position).normalized;
        armDirRHome = (rightArm.TransformPoint(tipLocal) - rightArm.position).normalized;
        // reference centre = midpoint of the two motors (to pick the correct circle intersection)
        refCenter = (leftCrank.position + rightCrank.position) * 0.5f;

        var ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float64Msg>(leftTopic,  m => { qLeft  = (float)m.data; haveRosData = true; });
        ros.Subscribe<Float64Msg>(rightTopic, m => { qRight = (float)m.data; haveRosData = true; });

        if (logCsv) OpenCsv();
        tStart = Time.time;
        Debug.Log($"[Pantograph] Ready. armLen={armLen:F4} (should be ~0.12). " +
                  (selfTestSweepDeg > 0f ? "Self-test ACTIVE." : "Waiting for ROS."));
    }

    void Update()
    {
        if (selfTestSweepDeg > 0f && !haveRosData)
        {
            float a = selfTestSweepDeg * Mathf.Deg2Rad *
                      Mathf.Sin(2f * Mathf.PI * (Time.time - tStart) / Mathf.Max(0.1f, selfTestPeriod));
            qLeft = a; qRight = -a;
        }
        float qL = qLeft  * leftSign  + leftOffsetDeg  * Mathf.Deg2Rad;
        float qR = qRight * rightSign + rightOffsetDeg * Mathf.Deg2Rad;

        // 1) turn the cranks to the commands
        leftCrank.rotation  = Quaternion.AngleAxis(qL * Mathf.Rad2Deg, leftAxisW)  * leftHome;
        rightCrank.rotation = Quaternion.AngleAxis(qR * Mathf.Rad2Deg, rightAxisW) * rightHome;

        // 2) elbow positions (from the cranks)
        Vector3 pa = leftArm.position, pb = rightArm.position;

        // 3) intersection of two circles of the SAME radius armLen -> the CORRECT closed EE position
        Vector3 dv = pb - pa; float d = Mathf.Max(dv.magnitude, 1e-6f);
        float aDist = d * 0.5f;                          // equal radii -> midpoint
        float h2 = armLen * armLen - aDist * aDist;
        float h = h2 > 0f ? Mathf.Sqrt(h2) : 0f;
        Vector3 m = pa + (aDist / d) * dv;
        Vector3 perp = Vector3.Cross(planeN, dv).normalized;
        Vector3 cand1 = m + h * perp, cand2 = m - h * perp;
        // pick the intersection that is FARTHER from the centre of the motors (the working side)
        Vector3 eePos = (cand1 - refCenter).sqrMagnitude >= (cand2 - refCenter).sqrMagnitude ? cand1 : cand2;

        // 4) turn the arms to point at the EE -> their tips coincide there
        leftArm.rotation  = Quaternion.FromToRotation(armDirLHome, (eePos - pa).normalized) * leftArmHome;
        rightArm.rotation = Quaternion.FromToRotation(armDirRHome, (eePos - pb).normalized) * rightArmHome;
        ee.position = eePos;
        ee.rotation = leftArm.rotation;

        LogRow();
    }

    void LogRow()
    {
        if (!logCsv || csv == null || Time.time - lastLog < 1f / Mathf.Max(1f, logRateHz)) return;
        lastLog = Time.time;
        Vector3 e = baseLink.InverseTransformPoint(ee.position);
        // gap = distance of the right arm tip from the EE (should be ~ 0)
        float gapMm = Vector3.Distance(rightArm.TransformPoint(tipLocal), ee.position) * 1000f;
        csv.WriteLine($"{Time.time - tStart:F4},{qLeft:F6},{qRight:F6},{e.x:F6},{e.y:F6},{e.z:F6},{gapMm:F3}");
    }

    Transform FindChild(string n)
    {
        foreach (var t in GetComponentsInChildren<Transform>(true))
            if (t.name == n) return t;
        return null;
    }

    Vector3 JointAxisWorld(Transform crank)
    {
        var hj = crank.GetComponent<HingeJoint>();
        return hj != null ? crank.TransformDirection(hj.axis).normalized : Vector3.up;
    }

    void OpenCsv()
    {
        string path = Path.Combine(Application.dataPath, "..", csvFileName);
        csv = new StreamWriter(path, false) { AutoFlush = true };
        csv.WriteLine("t,q_left,q_right,ee_x,ee_y,ee_z,gap_mm");
        Debug.Log("[Pantograph] CSV -> " + Path.GetFullPath(path));
    }

    void OnDisable()
    {
        if (csv != null) { csv.Flush(); csv.Close(); csv = null; }
    }
}
