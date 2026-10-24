// PantographForceControl.cs - ΔΡΟΜΟΣ B, Phase 2β: έλεγχος ΔΥΝΑΜΗΣ με Ιακωβιανό.
//
// Ιδέα (ρομποτική): θέλουμε δύναμη F στο τελικό σημείο (EE) προς τον τοίχο.
//   F_cmd = F_target * d_hat            (d_hat = κατεύθυνση προς τοίχο, ομαλό ξεκίνημα με ramp)
//   τ     = J^T * F_cmd               (ροπές κινητήρων, J = Ιακωβιανός ζωντανά)
// Το EE σπρώχνει, λόγω φυσικής δεν περνά τον τοίχο -> ισορροπεί πιέζοντας με F_target.
//
// Σιγά-σιγά = ΑΠΟΣΒΕΣΗ μέσω angularDamping των κινητήρων (σταθερή στο PhysX),
// ΟΧΙ explicit -b*v (που ήταν ασταθές). Στην ισορροπία ω=0, αλλά το PhysX πολλαπλασιάζει
// την ταχύτητα με (1-c*Δt) ΑΦΟΥ προσθέσει τη ροπή -> ο αισθητήρας δίνει F*(1-c*Δt)
// (c=40, Δt=2 ms -> x0.92, π.χ. 1.475 -> 1.356 N, με Δt=1 ms -> x0.96).
//
// Κάνει τους κινητήρες ΔΥΝΑΜΙΚΟΥΣ + HingeJoint στη βάση, και αυξάνει αδράνεια/μάζα
// για ΣΤΑΘΕΡΗ δυναμική (δεν αλλάζει τη στατική ισορροπία 1 N). Loop closure + projection
// ΜΕΝΟΥΝ -> δεν σπάει.
//
// ΤΟΠΟΘΕΤΗΣΗ: στο root 'five_bar_mechanism'. Πρέπει να έχει μπει ο τοίχος.
using System.Collections;
using System.IO;
using UnityEngine;

public class PantographForceControl : MonoBehaviour
{
    [Header("Εντολή δύναμης στο EE")]
    public float targetForceN = 1.0f;     // δύναμη προς τον τοίχο
    public float forceRampSec = 3.0f;     // 0 -> target (ομαλό ξεκίνημα)
    public float startDelaySec = 3.0f;    // αναμονή μετά το Play πριν ξεκινήσει ο έλεγχος (0 = αμέσως)

    [Header("Σιγά-σιγά: απόσβεση κινητήρων (σταθερή)")]
    public float motorAngularDamping = 40f;   // μεγαλύτερο = πιο αργή/σταθερή προσέγγιση

    [Header("Σταθερότητα δυναμικής (δεν αλλάζει την ισορροπία)")]
    public float bodyMass    = 0.10f;     // μάζα κινούμενων links
    public float bodyInertia = 0.005f;    // inertiaTensor (αντικαθιστά το 1e-4 του setup)
    public float maxTorque   = 0.1f;      // όριο ροπής κινητήρα (Nm) - ΙΔΙΟ με Gazebo MAX_TORQUE -> κορεσμός ~1.4N

    [Header("Physics tuning (Awake)")]
    public float fixedDt = 0.002f;        // 2 ms (500 Hz), όπως οι μετρήσεις της διπλωματικής (1.356 N στα 2 N)
    public int   solverIters = 100, velIters = 50;

    [Header("CSV + logging")]
    public bool   logCsv = true;
    public string csvFileName = "unity_force_log.csv";
    public float  logRateHz = 100f;
    public float  consoleEverySec = 0.5f;

    Transform baseLink, leftCrank, rightCrank, leftArm, rightArm, ee;
    Rigidbody  baseRb, leftCrankRb, rightCrankRb, eeRb;
    ContactForceProbe eeProbe;

    Vector3 planeN, uHat, vHat, refCenter;
    Vector2 OL, OR;
    float crankLenL, crankLenR, armLen, aL0, aR0;
    Vector2 dHatPlane;
    Vector3 tipLocalRight;

    float tStart, lastLog = -999f, lastConsole = -999f;
    bool running;                         // false όσο περιμένει το startDelaySec
    StreamWriter csv;

    void Awake()
    {
        Time.fixedDeltaTime = Mathf.Max(0.0005f, fixedDt);
        Physics.defaultSolverIterations = Mathf.Max(6, solverIters);
        Physics.defaultSolverVelocityIterations = Mathf.Max(1, velIters);
        var pd = GetComponent<PantographPhysicsDrive>();
        if (pd) pd.enabled = false;
    }

    IEnumerator Start()
    {
        // Αναμονή: οι κινητήρες μένουν kinematic (όπως τους αφήνει το Setup) -> ο μηχανισμός
        // μένει ακίνητος στο home. Το CSV/t=0 ξεκινούν μετά την αναμονή (ίδια γραφική με πριν).
        if (startDelaySec>0f)
        {
            Debug.Log($"[ForceControl] Αναμονή {startDelaySec:F1}s πριν την εκκίνηση...");
            yield return new WaitForSeconds(startDelaySec);
        }

        baseLink=Find("base_link"); leftCrank=Find("left_crank"); rightCrank=Find("right_crank");
        leftArm=Find("left_arm"); rightArm=Find("right_arm"); ee=Find("end_effector_link");
        if (!baseLink||!leftCrank||!rightCrank||!leftArm||!rightArm||!ee)
        { Debug.LogError("[ForceControl] Λείπουν links - έτρεξες το Setup;"); enabled=false; yield break; }
        baseRb=baseLink.GetComponent<Rigidbody>();
        leftCrankRb=leftCrank.GetComponent<Rigidbody>(); rightCrankRb=rightCrank.GetComponent<Rigidbody>();
        eeRb=ee.GetComponent<Rigidbody>();
        if (!baseRb||!leftCrankRb||!rightCrankRb||!eeRb)
        { Debug.LogError("[ForceControl] Λείπουν Rigidbody - τρέξε πρώτα το Setup."); enabled=false; yield break; }

        // άξονες επιπέδου (ίδιοι με Setup/Wall)
        planeN = Vector3.Cross(rightCrank.position-leftCrank.position, leftArm.position-leftCrank.position).normalized;
        if (planeN.sqrMagnitude<1e-9f) planeN=Vector3.up;
        refCenter = (leftCrank.position+rightCrank.position)*0.5f;
        uHat = Vector3.ProjectOnPlane(ee.position-refCenter, planeN).normalized;
        vHat = Vector3.Cross(planeN, uHat).normalized;

        // 2D γεωμετρία στο home
        OL=P2(leftCrank.position); OR=P2(rightCrank.position);
        Vector2 eLh=P2(leftArm.position), eRh=P2(rightArm.position), Eh=P2(ee.position);
        crankLenL=(eLh-OL).magnitude; crankLenR=(eRh-OR).magnitude;
        armLen=(Eh-eLh).magnitude;
        aL0=Mathf.Atan2(eLh.y-OL.y, eLh.x-OL.x);
        aR0=Mathf.Atan2(eRh.y-OR.y, eRh.x-OR.x);

        // κατεύθυνση προς τον τοίχο
        var wall=GameObject.Find("virtual_wall");
        if (wall!=null) dHatPlane=(P2(wall.transform.position)-Eh).normalized;
        else { dHatPlane=new Vector2(1,0); Debug.LogWarning("[ForceControl] Δεν βρέθηκε 'virtual_wall' - σπρώχνω +reach."); }
        if (dHatPlane.sqrMagnitude<1e-9f) dHatPlane=new Vector2(1,0);

        // ΣΤΑΘΕΡΟΤΗΤΑ: αύξησε αδράνεια/μάζα όλων των κινούμενων links
        foreach (var rb in new[]{leftCrankRb,rightCrankRb,leftArm.GetComponent<Rigidbody>(),
                                 rightArm.GetComponent<Rigidbody>(),eeRb})
        { rb.mass=bodyMass; rb.inertiaTensor=Vector3.one*bodyInertia; rb.centerOfMass=Vector3.zero; }

        // κινητήρες -> torque mode (δυναμικοί + hinge στη βάση + απόσβεση)
        SetupMotor(leftCrankRb, leftCrank);
        SetupMotor(rightCrankRb, rightCrank);

        eeProbe = ee.GetComponent<ContactForceProbe>();
        if (eeProbe==null) eeProbe = ee.gameObject.AddComponent<ContactForceProbe>();
        tipLocalRight = rightArm.InverseTransformPoint(ee.position);

        // ΔΙΑΓΝΩΣΤΙΚΟ κατεύθυνσης: για δύναμη +d_hat, αναμενόμενη κίνηση EE ~ J*J^T*d_hat -> πρέπει ~ +d_hat
        const float h=1e-4f;
        Vector2 a=FK(h,0)-FK(-h,0), b=FK(0,h)-FK(0,-h);
        float j00=a.x/(2*h), j10=a.y/(2*h), j01=b.x/(2*h), j11=b.y/(2*h);
        float tL=j00*dHatPlane.x+j10*dHatPlane.y, tR=j01*dHatPlane.x+j11*dHatPlane.y;
        Vector2 aE=new Vector2(j00*tL+j01*tR, j10*tL+j11*tR);
        float align = aE.sqrMagnitude>1e-12f ? Vector2.Dot(aE.normalized,dHatPlane) : 0f;

        if (logCsv) OpenCsv();
        tStart=Time.time;
        running=true;
        Debug.Log($"[ForceControl] Έτοιμο. target={targetForceN}N ramp={forceRampSec}s damping={motorAngularDamping} dt={Time.fixedDeltaTime*1000:F1}ms.\n" +
                  $"  J@home=[{j00:F4} {j01:F4}; {j10:F4} {j11:F4}]  dHat=({dHatPlane.x:F2},{dHatPlane.y:F2})  " +
                  $"ALIGNMENT={align:F2} (πρέπει κοντά στο +1, αν είναι αρνητικό/μηδέν -> λάθος J).");
    }

    void SetupMotor(Rigidbody rb, Transform crank)
    {
        rb.isKinematic=false; rb.useGravity=false;
        rb.angularDamping=motorAngularDamping; rb.linearDamping=0.1f;
        bool has=false;
        foreach (var h in crank.GetComponents<HingeJoint>()) if (h.connectedBody==baseRb) has=true;
        if (!has)
        {
            var hj=crank.gameObject.AddComponent<HingeJoint>();
            hj.connectedBody=baseRb;
            hj.anchor=Vector3.zero;
            hj.axis=crank.InverseTransformDirection(planeN).normalized;
            hj.autoConfigureConnectedAnchor=true;
            hj.enableCollision=false; hj.enablePreprocessing=false;
        }
    }

    void FixedUpdate()
    {
        if (!running) return;             // ακόμα στην αναμονή (ή απέτυχε το setup)
        float qL=DeltaAngle(P2(leftArm.position),  OL, aL0);
        float qR=DeltaAngle(P2(rightArm.position), OR, aR0);

        const float h=1e-4f;
        Vector2 a=FK(qL+h,qR)-FK(qL-h,qR), b=FK(qL,qR+h)-FK(qL,qR-h);
        float j00=a.x/(2*h), j10=a.y/(2*h), j01=b.x/(2*h), j11=b.y/(2*h);

        float framp=Mathf.Min(1f,(Time.time-tStart)/Mathf.Max(0.01f,forceRampSec))*targetForceN;
        Vector2 F=framp*dHatPlane;

        float tauL=Mathf.Clamp(j00*F.x+j10*F.y, -maxTorque, maxTorque);
        float tauR=Mathf.Clamp(j01*F.x+j11*F.y, -maxTorque, maxTorque);
        leftCrankRb.AddTorque(planeN*tauL, ForceMode.Force);
        rightCrankRb.AddTorque(planeN*tauR, ForceMode.Force);

        // F_est = (J^T)^-1 * τ_clamped -> δύναμη που αντιστοιχεί στην ΕΦΑΡΜΟΣΜΕΝΗ (κομμένη) ροπή.
        // Δείχνει τον κορεσμό όταν χτυπάει το maxTorque (όπως το F_est του Gazebo).
        float det=j00*j11-j01*j10, fEst=0f;
        if (Mathf.Abs(det)>1e-9f)
        {
            float fx=( j11*tauL-j10*tauR)/det, fy=(-j01*tauL+j00*tauR)/det;
            fEst=fx*dHatPlane.x+fy*dHatPlane.y;   // συνιστώσα προς τον τοίχο
        }

        Log(framp, fEst, tauL, tauR);
    }

    Vector2 FK(float qL, float qR)
    {
        Vector2 eL=OL+crankLenL*new Vector2(Mathf.Cos(aL0+qL), Mathf.Sin(aL0+qL));
        Vector2 eR=OR+crankLenR*new Vector2(Mathf.Cos(aR0+qR), Mathf.Sin(aR0+qR));
        Vector2 d=eR-eL; float dl=Mathf.Max(d.magnitude,1e-6f);
        float aa=dl*0.5f; float hh=armLen*armLen-aa*aa; float hgt=hh>0?Mathf.Sqrt(hh):0f;
        Vector2 m=eL+(aa/dl)*d;
        Vector2 perp=new Vector2(-d.y,d.x)/dl;
        Vector2 c1=m+hgt*perp, c2=m-hgt*perp;
        return c1.sqrMagnitude>=c2.sqrMagnitude ? c1 : c2;
    }

    float DeltaAngle(Vector2 elbow, Vector2 O, float a0)
    {
        float d=Mathf.Atan2(elbow.y-O.y, elbow.x-O.x)-a0;
        while (d> Mathf.PI) d-=2*Mathf.PI;
        while (d<-Mathf.PI) d+=2*Mathf.PI;
        return d;
    }

    Vector2 P2(Vector3 p){ Vector3 r=p-refCenter; return new Vector2(Vector3.Dot(r,uHat), Vector3.Dot(r,vHat)); }
    Transform Find(string n){ foreach (var t in GetComponentsInChildren<Transform>(true)) if (t.name==n) return t; return null; }

    void OpenCsv()
    {
        string path=Path.Combine(Application.dataPath,"..",csvFileName);
        csv=new StreamWriter(path,false){AutoFlush=true};
        csv.WriteLine("t,F_cmd_N,F_est_N,ee_reach,ee_lat,v_reach,contactF_N,gap_mm,tauL,tauR");
        Debug.Log("[ForceControl] CSV -> "+Path.GetFullPath(path));
    }
    void Log(float framp, float fEst, float tauL, float tauR)
    {
        Vector2 E=P2(ee.position);
        float reach=E.x, lat=E.y;
        float vReach=Vector3.Dot(eeRb.linearVelocity,uHat);
        float contactF = eeProbe!=null ? Mathf.Abs(Vector3.Dot(eeProbe.Force,uHat)) : 0f;
        float gapMm = Vector3.Distance(rightArm.TransformPoint(tipLocalRight), ee.position)*1000f;

        if (logCsv && csv!=null && Time.time-lastLog>=1f/Mathf.Max(1f,logRateHz))
        {
            lastLog=Time.time;
            csv.WriteLine($"{Time.time-tStart:F4},{framp:F4},{fEst:F4},{reach:F5},{lat:F5},{vReach:F4},{contactF:F4},{gapMm:F3},{tauL:F4},{tauR:F4}");
        }
        if (Time.time-lastConsole>=consoleEverySec)
        {
            lastConsole=Time.time;
            Debug.Log($"[FC] t={Time.time-tStart:F1}s Fcmd={framp:F2}N Fest={fEst:F2}N reach={reach*1000:F1}mm " +
                      $"contact={contactF:F2}N gap={gapMm:F2}mm");
        }
    }
    void OnDisable(){ if (csv!=null){ csv.Flush(); csv.Close(); csv=null; } }
}
