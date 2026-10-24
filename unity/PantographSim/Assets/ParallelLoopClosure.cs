// ParallelLoopClosure.cs - Runtime controller για τον five-bar στο Unity.
// Οδηγεί τους δύο κινητήρες από τις εντολές θέσης του ROS 2
//   /left_motor/cmd_pos , /right_motor/cmd_pos   (Float64, rad - q=0 στο home),
// όπως οι sims Gazebo / MuJoCo / Webots, και καταγράφει CSV για γραφικές.
//
// ΚΛΕΙΣΙΜΟ ΒΡΟΧΟΥ - ΑΝΑΛΥΤΙΚΑ (εγγυημένα ενωμένα χέρια):
//   Το PhysX είναι αναξιόπιστο σε κλειστούς βρόχους ΚΑΙ το URDF έχει λάθος home pose
//   (το δεξί χέρι δεν φτάνει το EE). Γι' αυτό αγνοούμε το home του URDF και υπολογίζουμε
//   τη ΣΩΣΤΗ κλειστή διάταξη: με τα πραγματικά μήκη χεριών (0.12) και τις θέσεις των
//   αγκώνων (από τους cranks), βρίσκουμε το EE ως τομή δύο κύκλων και στρέφουμε τα δύο
//   χέρια να δείχνουν εκεί -> οι άκρες τους ΣΥΜΠΙΠΤΟΥΝ. Όλα τα bodies γίνονται kinematic.
//
// ΤΟΠΟΘΕΤΗΣΗ: στο root GameObject 'five_bar_mechanism'.
using System.IO;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class PantographController : MonoBehaviour
{
    [Header("ROS Topics")]
    public string leftTopic  = "/left_motor/cmd_pos";
    public string rightTopic = "/right_motor/cmd_pos";

    [Header("Σύμβαση γωνιών - ρύθμισε αν κάποιο χέρι στρίβει ανάποδα")]
    public float leftSign  = 1f;
    public float rightSign = 1f;
    public float leftOffsetDeg  = 0f;
    public float rightOffsetDeg = 0f;

    [Header("Self-test (χωρίς ROS): sweep για να δεις την κίνηση)")]
    public float selfTestSweepDeg = 0f;   // π.χ. 20
    public float selfTestPeriod   = 4f;

    [Header("Καταγραφή CSV")]
    public bool   logCsv = true;
    public string csvFileName = "unity_trajectory_log.csv";
    public float  logRateHz = 50f;

    Transform baseLink, leftCrank, rightCrank, leftArm, rightArm, ee;
    Quaternion leftHome, rightHome, leftArmHome, rightArmHome;
    Vector3 leftAxisW, rightAxisW, planeN, refCenter;

    Vector3 tipLocal, armDirLHome, armDirRHome;   // για τον αναλυτικό solver
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
        { Debug.LogError("[Pantograph] Δεν βρέθηκαν τα links. Βάλε το script στο 'five_bar_mechanism'."); enabled = false; return; }

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

        // ΠΡΑΓΜΑΤΙΚΟ μήκος χεριού + τοπική άκρη της βέργας (το EE είναι άκαμπτα στην άκρη του αριστερού χεριού)
        tipLocal = leftArm.InverseTransformPoint(ee.position);   // |tipLocal| = 0.12
        armLen   = tipLocal.magnitude;
        // πραγματικές διευθύνσεις βέργας στο home (όχι προς το χαλασμένο EE)
        armDirLHome = (leftArm.TransformPoint(tipLocal)  - leftArm.position).normalized;
        armDirRHome = (rightArm.TransformPoint(tipLocal) - rightArm.position).normalized;
        // κέντρο αναφοράς = μέσο των δύο κινητήρων (για επιλογή της σωστής τομής κύκλων)
        refCenter = (leftCrank.position + rightCrank.position) * 0.5f;

        var ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<Float64Msg>(leftTopic,  m => { qLeft  = (float)m.data; haveRosData = true; });
        ros.Subscribe<Float64Msg>(rightTopic, m => { qRight = (float)m.data; haveRosData = true; });

        if (logCsv) OpenCsv();
        tStart = Time.time;
        Debug.Log($"[Pantograph] Έτοιμο. armLen={armLen:F4} (πρέπει ~0.12). " +
                  (selfTestSweepDeg > 0f ? "Self-test ΕΝΕΡΓΟ." : "Περιμένω ROS."));
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

        // 1) γύρνα τους cranks στις εντολές
        leftCrank.rotation  = Quaternion.AngleAxis(qL * Mathf.Rad2Deg, leftAxisW)  * leftHome;
        rightCrank.rotation = Quaternion.AngleAxis(qR * Mathf.Rad2Deg, rightAxisW) * rightHome;

        // 2) θέσεις αγκώνων (από τους cranks)
        Vector3 pa = leftArm.position, pb = rightArm.position;

        // 3) τομή δύο κύκλων ΙΔΙΑΣ ακτίνας armLen -> η ΣΩΣΤΗ κλειστή θέση του EE
        Vector3 dv = pb - pa; float d = Mathf.Max(dv.magnitude, 1e-6f);
        float aDist = d * 0.5f;                          // ίσες ακτίνες -> μέσο
        float h2 = armLen * armLen - aDist * aDist;
        float h = h2 > 0f ? Mathf.Sqrt(h2) : 0f;
        Vector3 m = pa + (aDist / d) * dv;
        Vector3 perp = Vector3.Cross(planeN, dv).normalized;
        Vector3 cand1 = m + h * perp, cand2 = m - h * perp;
        // διάλεξε την τομή που είναι ΜΑΚΡΥΤΕΡΑ από το κέντρο των κινητήρων (η πλευρά εργασίας)
        Vector3 eePos = (cand1 - refCenter).sqrMagnitude >= (cand2 - refCenter).sqrMagnitude ? cand1 : cand2;

        // 4) στρίψε τα χέρια να δείχνουν στο EE -> οι άκρες συμπίπτουν εκεί
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
        // κενό = απόσταση άκρης δεξιού χεριού από το EE (πρέπει ~ 0)
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
