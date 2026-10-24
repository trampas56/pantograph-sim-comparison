// PantographPhysicsDrive.cs - ΔΡΟΜΟΣ B (πλήρης φυσική) drive για τον five-bar.
//
// Οδηγεί τις ΚΙΝΗΜΑΤΙΚΕΣ μανιβέλες (cranks) στις εντολές θέσης του ROS 2
//   /left_motor/cmd_pos , /right_motor/cmd_pos   (Float64, rad - q=0 στο home),
// με MoveRotation στο FixedUpdate. Τα ΔΥΝΑΜΙΚΑ χέρια ακολουθούν μέσω των αρθρώσεων,
// και το ConfigurableJoint κλεισίματος (PantographSetup) τα κρατά ΕΝΩΜΕΝΑ.
//
// Προϋπόθεση: έχει τρέξει "Tools > Pantograph > Setup Five-Bar (Physics + Loop Closure)".
// ΤΟΠΟΘΕΤΗΣΗ: στο root 'five_bar_mechanism'. (Αφαίρεσε τον παλιό PantographController
// - αυτός κάνει τα πάντα kinematic, που ακυρώνει τη φυσική του Δρόμου B.)
using System.IO;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class PantographPhysicsDrive : MonoBehaviour
{
    [Header("ROS Topics")]
    public string leftTopic  = "/left_motor/cmd_pos";
    public string rightTopic = "/right_motor/cmd_pos";

    [Header("Σύμβαση γωνιών - ρύθμισε αν κάποιο χέρι στρίβει ανάποδα")]
    public float leftSign  = 1f;
    public float rightSign = 1f;
    public float leftOffsetDeg  = 0f;
    public float rightOffsetDeg = 0f;

    [Header("Self-test (χωρίς ROS): sweep για να δεις ότι ΔΕΝ σπάει")]
    public float selfTestSweepDeg = 0f;   // π.χ. 8
    public float selfTestPeriod   = 4f;

    [Header("Physics tuning (εφαρμόζεται στο Awake)")]
    public float fixedDt   = 0.002f;      // 500 Hz - σταθερότητα κλειστού βρόχου (Gazebo: 1ms)
    public int   solverIters = 60;
    public int   velIters    = 40;

    [Header("Καταγραφή CSV (gap_mm = πόσο ανοίγει το κλείσιμο, πρέπει ~ 0)")]
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
        { Debug.LogError("[PantographDrive] Δεν βρέθηκαν τα links. Έτρεξες το Setup; Είναι το script στο 'five_bar_mechanism';"); enabled = false; return; }

        leftCrankRb  = leftCrank.GetComponent<Rigidbody>();
        rightCrankRb = rightCrank.GetComponent<Rigidbody>();
        if (!leftCrankRb || !rightCrankRb)
        { Debug.LogError("[PantographDrive] Οι cranks δεν έχουν Rigidbody - τρέξε πρώτα το Setup (Δρόμος B)."); enabled = false; return; }
        leftCrankRb.isKinematic = true;
        rightCrankRb.isKinematic = true;

        // Άξονας περιστροφής κινητήρα = άξονας του elbow hinge (κάθετη του επιπέδου)
        var elbow = leftArm.GetComponent<HingeJoint>();
        planeN = elbow != null ? leftArm.TransformDirection(elbow.axis).normalized : Vector3.up;

        leftHome  = leftCrankRb.rotation;
        rightHome = rightCrankRb.rotation;

        // σημείο κλεισίματος στο δεξί χέρι (στο home συμπίπτει με το EE) -> για μέτρηση κενού
        tipLocalRight = rightArm.InverseTransformPoint(ee.position);

        var ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float64Msg>(leftTopic,  m => { qLeft  = (float)m.data; haveRosData = true; });
        ros.Subscribe<Float64Msg>(rightTopic, m => { qRight = (float)m.data; haveRosData = true; });

        if (logCsv) OpenCsv();
        tStart = Time.time;
        Debug.Log($"[PantographDrive] Έτοιμο (Δρόμος B). dt={Time.fixedDeltaTime*1000f:F1}ms, " +
                  $"solver={Physics.defaultSolverIterations}/{Physics.defaultSolverVelocityIterations}. " +
                  (selfTestSweepDeg > 0f ? "Self-test ΕΝΕΡΓΟ." : "Περιμένω ROS."));
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

        // Οδήγησε τις κινηματικές μανιβέλες, τα δυναμικά χέρια + το loop closure ακολουθούν.
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
