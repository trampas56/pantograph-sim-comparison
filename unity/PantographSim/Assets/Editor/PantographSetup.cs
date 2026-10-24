// PantographSetup.cs - Editor-only setup για τον five-bar (ΔΡΟΜΟΣ B: πλήρης φυσική).
// Μετατρέπει τον εισαγόμενο ArticulationBody-tree σε Rigidbody + αρθρώσεις και
// ΚΛΕΙΝΕΙ τον παράλληλο βρόχο ΧΩΡΙΣ να σπάει:
//
//   1) base + δύο cranks -> KINEMATIC (η βάση καρφωμένη, οι μανιβέλες οδηγούνται).
//   2) left_arm, right_arm, end_effector -> DYNAMIC (ελαφριά, χωρίς βαρύτητα).
//   3) SNAP στη ΣΩΣΤΗ κλειστή θέση (τομή 2 κύκλων) ΠΡΙΝ μπουν οι αρθρώσεις ->
//      το constraint ξεκινά με ~0 σφάλμα (όχι 196mm yank που εκρήγνυται).
//   4) Δέντρο: HingeJoint στους αγκώνες (left_elbow, right_elbow) + ee_left.
//   5) ΚΛΕΙΣΙΜΟ ΒΡΟΧΟΥ: ConfigurableJoint right_arm <-> end_effector με
//      linear LOCKED + free spin γύρω από το επίπεδο + PROJECTION -> δεν ξεκολλάει.
//
// ΣΗΜΑΝΤΙΚΟ: τρέξε σε EDIT mode (όχι Play). Αν αποτύχει η αφαίρεση των
// ArticulationBody, ΞΑΝΑΦΟΡΤΩΣΕ τη σκηνή χωρίς save και ξανατρέξε.
// Menu: Tools > Pantograph > Setup Five-Bar (Physics + Loop Closure)
#if UNITY_EDITOR
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public static class PantographSetup
{
    const string RootName = "five_bar_mechanism";
    static readonly string[] LinkNames =
        { "base_link", "left_crank", "left_arm", "end_effector_link", "right_crank", "right_arm" };
    // ΚΙΝΗΜΑΤΙΚΑ (οδηγούμενα/καρφωμένα) - τα υπόλοιπα γίνονται δυναμικά
    static readonly string[] Kinematic =
        { "base_link", "left_crank", "right_crank" };
    // Αρθρώσεις δέντρου: child -> (parent στο hierarchy). ΟΧΙ στους cranks (οδηγούνται απευθείας).
    // Το end_effector ΔΕΝ είναι εδώ - μπαίνει ως rigid weld στο left_arm (όχι revolute).
    static readonly string[] TreeHinges =
        { "left_arm", "right_arm" };

    // κατάσταση που περνά ανάμεσα στις δύο φάσεις
    static GameObject s_root;
    static Dictionary<string, Transform> s_links;
    static Dictionary<string, Vector3> s_anchorPos;
    static Dictionary<string, Quaternion> s_anchorRot;
    static Dictionary<string, float> s_mass;
    static int s_retries;

    [MenuItem("Tools/Pantograph/Setup Five-Bar (Physics + Loop Closure)")]
    public static void Setup()
    {
        if (EditorApplication.isPlaying)
        {
            Debug.LogError("[PantographSetup] Βγες από το Play mode (πάτα το Play) και ξανατρέξε σε EDIT mode.");
            return;
        }

        s_root = GameObject.Find(RootName);
        if (s_root == null) { Debug.LogError($"[PantographSetup] Δεν βρέθηκε root '{RootName}'."); return; }

        s_links = new Dictionary<string, Transform>();
        foreach (var t in s_root.GetComponentsInChildren<Transform>(true))
            if (System.Array.IndexOf(LinkNames, t.name) >= 0 && !s_links.ContainsKey(t.name))
                s_links[t.name] = t;
        foreach (var n in LinkNames)
            if (!s_links.ContainsKey(n)) { Debug.LogError($"[PantographSetup] Λείπει το link '{n}'."); return; }

        // --- ΦΑΣΗ 1: κατέγραψε δεδομένα άρθρωσης & ΣΒΗΣΕ ΟΛΑ τα ArticulationBody ---
        s_anchorPos = new Dictionary<string, Vector3>();
        s_anchorRot = new Dictionary<string, Quaternion>();
        s_mass = new Dictionary<string, float>();
        foreach (var n in LinkNames)
        {
            s_anchorPos[n] = Vector3.zero;
            s_anchorRot[n] = Quaternion.identity;
            s_mass[n] = 0.02f;
            var ab = s_links[n].GetComponent<ArticulationBody>();
            if (ab != null)
            {
                s_mass[n]      = Mathf.Max(ab.mass, 0.001f);
                s_anchorPos[n] = ab.anchorPosition;   // θέση άρθρωσης στο local frame του link
                s_anchorRot[n] = ab.anchorRotation;   // ο revolute άξονας = X αυτού του frame
            }
        }
        // Σημ.: ΔΕΝ εξαρτιόμαστε πλέον από δεδομένα ArticulationBody για τους άξονες -
        // ο άξονας υπολογίζεται γεωμετρικά (SnapToConsistentHome), ώστε το Setup να
        // δουλεύει και σε σκηνή που έχει ΗΔΗ μετατραπεί σε Rigidbody (καμία AB).

        // ΠΡΩΤΑ αφαίρεσε τα URDF helper scripts (UrdfInertial/UrdfJointRevolute/UrdfLink...) που
        // "κλειδώνουν" το ArticulationBody μέσω [RequireComponent], αλλιώς ΔΕΝ αφαιρείται.
        foreach (var n in LinkNames)
        {
            var go = s_links[n].gameObject;
            for (int pass = 0; pass < 5; pass++)
            {
                bool removed = false;
                foreach (var c in go.GetComponents<MonoBehaviour>())
                {
                    if (c == null) continue;
                    var ns = c.GetType().Namespace;
                    if (ns != null && ns.StartsWith("Unity.Robotics.UrdfImporter"))
                    { Object.DestroyImmediate(c); removed = true; }
                }
                if (!removed) break;
            }
        }
        // ΤΩΡΑ αφαιρείται το ArticulationBody
        foreach (var ab in s_root.GetComponentsInChildren<ArticulationBody>(true))
            Object.DestroyImmediate(ab);

        s_retries = 0;
        EditorApplication.delayCall += FinishSetup;   // ΦΑΣΗ 2 στο επόμενο editor-tick
        Debug.Log("[PantographSetup] Αφαιρέθηκαν τα ArticulationBody. Ολοκλήρωση στο επόμενο tick...");
    }

    static void FinishSetup()
    {
        EditorApplication.delayCall -= FinishSetup;
        if (s_root == null || s_links == null) return;

        // Δοκίμασε Rigidbody στο base_link, αν το Unity δεν έχει "καθαρίσει" ακόμα τα
        // ArticulationBody, το AddComponent γυρίζει null -> ξαναδοκίμασε στο επόμενο tick.
        var baseGo = s_links["base_link"].gameObject;
        var rbTest = baseGo.GetComponent<Rigidbody>();
        if (rbTest == null) rbTest = baseGo.AddComponent<Rigidbody>();
        if (rbTest == null)
        {
            if (s_retries++ < 8) { EditorApplication.delayCall += FinishSetup; return; }
            Debug.LogError("[PantographSetup] Το Unity δεν επιτρέπει ακόμα Rigidbody. " +
                           "Ξαναφόρτωσε τη σκηνή χωρίς save και ξανατρέξε.");
            return;
        }

        // --- ΦΑΣΗ 2α: Rigidbody σε όλα τα links ---
        foreach (var n in LinkNames)
        {
            var go = s_links[n].gameObject;
            var rb = go.GetComponent<Rigidbody>();
            if (rb == null) rb = go.AddComponent<Rigidbody>();
            rb.mass = s_mass[n];
            rb.useGravity = false;                  // οριζόντιος μηχανισμός - χωρίς βαρύτητα
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
            rb.linearDamping = 0.05f;
            rb.angularDamping = 0.05f;
            rb.solverIterations = 80;               // σταθερότητα κλειστού βρόχου (per-body)
            rb.solverVelocityIterations = 40;
            rb.centerOfMass = Vector3.zero;
            rb.inertiaTensor = Vector3.one * 1e-4f; // μικρά/ελαφριά links (χωρίς colliders)
            rb.isKinematic = false;
        }
        foreach (var n in Kinematic)
            s_links[n].GetComponent<Rigidbody>().isKinematic = true;

        // Καθάρισε ΟΛΕΣ τις παλιές αρθρώσεις (από προηγούμενο setup A/B) -> idempotent,
        // δουλεύει πάνω σε ό,τι σκηνή κι αν έχεις ήδη.
        foreach (var n in LinkNames)
            foreach (var j in s_links[n].GetComponents<Joint>())
                Object.DestroyImmediate(j);

        // --- ΦΑΣΗ 2β: SNAP στη ΣΩΣΤΗ κλειστή θέση (τομή 2 κύκλων) ---
        // Στρέφουμε τα δύο χέρια ώστε οι άκρες τους να ΣΥΜΠΙΠΤΟΥΝ στο EE, ΠΡΙΝ μπει
        // το constraint. Έτσι το loop closure ξεκινά με ~0 σφάλμα -> δεν εκρήγνυται.
        Vector3 planeN, eePos;
        SnapToConsistentHome(out planeN, out eePos);

        // --- ΦΑΣΗ 2γ: HingeJoint στις αρθρώσεις του δέντρου (αγκώνες + ee) ---
        foreach (var n in TreeHinges)
        {
            var child = s_links[n];
            var parentRb = child.parent ? child.parent.GetComponent<Rigidbody>() : null;
            if (parentRb == null) { Debug.LogWarning($"[PantographSetup] '{n}': χωρίς parent Rigidbody."); continue; }

            foreach (var old in child.GetComponents<HingeJoint>()) Object.DestroyImmediate(old);             // idempotent
            foreach (var old in child.GetComponents<ConfigurableJoint>()) Object.DestroyImmediate(old);

            var hj = child.gameObject.AddComponent<HingeJoint>();
            hj.connectedBody = parentRb;
            hj.anchor = s_anchorPos[n];
            hj.axis = child.InverseTransformDirection(planeN).normalized;   // revolute γύρω από το επίπεδο
            hj.autoConfigureConnectedAnchor = true;                         // rest = τρέχουσα (κλειστή) θέση
            hj.enableCollision = false;
            hj.enablePreprocessing = false;
        }

        // --- EE: rigid weld (ConfigurableJoint all-locked + projection) στο left_arm ---
        // Το end_effector είναι η άκρη του αριστερού χεριού, το κρατάμε ΑΚΑΜΠΤΑ πάνω του
        // (όχι revolute) ώστε να μην υπάρχει ελεύθερος βαθμός περιστροφής που τρέμει.
        {
            var eeT = s_links["end_effector_link"];
            var leftArmRb = s_links["left_arm"].GetComponent<Rigidbody>();
            foreach (var old in eeT.GetComponents<HingeJoint>()) Object.DestroyImmediate(old);
            foreach (var old in eeT.GetComponents<ConfigurableJoint>()) Object.DestroyImmediate(old);
            var weld = eeT.gameObject.AddComponent<ConfigurableJoint>();
            weld.connectedBody = leftArmRb;
            weld.autoConfigureConnectedAnchor = true;
            weld.xMotion = weld.yMotion = weld.zMotion = ConfigurableJointMotion.Locked;
            weld.angularXMotion = weld.angularYMotion = weld.angularZMotion = ConfigurableJointMotion.Locked;
            weld.projectionMode = JointProjectionMode.PositionAndRotation;
            weld.projectionDistance = 0.0005f;
            weld.projectionAngle = 1f;
            weld.enableCollision = false;
            weld.enablePreprocessing = false;
            weld.breakForce = Mathf.Infinity;
            weld.breakTorque = Mathf.Infinity;
        }

        // --- ΦΑΣΗ 2δ: ΚΛΕΙΣΙΜΟ ΒΡΟΧΟΥ - ConfigurableJoint right_arm <-> end_effector ---
        // = MuJoCo <connect> / Gazebo DetachableJoint. Point-coincidence + free spin
        // γύρω από το επίπεδο, με PROJECTION ώστε να ΜΗΝ ξεκολλάει ποτέ οπτικά.
        var rightArm = s_links["right_arm"];
        var ee = s_links["end_effector_link"];
        foreach (var old in rightArm.GetComponents<ConfigurableJoint>()) Object.DestroyImmediate(old);       // idempotent

        var loop = rightArm.gameObject.AddComponent<ConfigurableJoint>();
        loop.connectedBody = ee.GetComponent<Rigidbody>();
        loop.anchor = rightArm.InverseTransformPoint(eePos);               // άκρη δεξιού χεριού (=EE) στο home
        loop.autoConfigureConnectedAnchor = true;                         // connectedAnchor ~ κέντρο EE
        loop.axis = rightArm.InverseTransformDirection(planeN).normalized; // πρωτεύων = το επίπεδο normal
        loop.secondaryAxis = rightArm.InverseTransformDirection((eePos - rightArm.position)).normalized;

        loop.xMotion = loop.yMotion = loop.zMotion = ConfigurableJointMotion.Locked;   // ίδιο σημείο (συγκόλληση θέσης)
        loop.angularXMotion = ConfigurableJointMotion.Free;               // ελεύθερη περιστροφή γύρω από το επίπεδο
        loop.angularYMotion = ConfigurableJointMotion.Locked;             // μένουν συνεπίπεδα
        loop.angularZMotion = ConfigurableJointMotion.Locked;

        loop.projectionMode = JointProjectionMode.PositionAndRotation;    // ΚΛΕΙΔΙ: hard-snap -> δεν "σπάει"
        loop.projectionDistance = 0.0005f;                                // 0.5mm ανοχή
        loop.projectionAngle = 1f;
        loop.enableCollision = false;
        loop.enablePreprocessing = false;                                 // καλύτερη σταθερότητα σε stiff loop
        loop.breakForce = Mathf.Infinity;                                 // ποτέ δεν σπάει
        loop.breakTorque = Mathf.Infinity;

        EditorUtility.SetDirty(s_root);
        UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(s_root.scene);

        float gapMm = Vector3.Distance(rightArm.TransformPoint(loop.anchor), ee.position) * 1000f;
        Debug.Log($"[PantographSetup] Έτοιμο (ΔΡΟΜΟΣ B): 6x Rigidbody, 3 hinges + 1 ConfigurableJoint " +
                  $"(projection ON). base & cranks = kinematic. Κενό κλεισίματος στο home = {gapMm:F3} mm. " +
                  "Βάλε τον PantographPhysicsDrive στο root, Self Test Sweep Deg=8, Play, μετά ΑΠΟΘΗΚΕΥΣΕ (Ctrl+S).");
    }

    // Τομή 2 κύκλων ίδιας ακτίνας (= μήκος χεριού): βρίσκει τη σωστή κλειστή θέση του EE
    // και στρέφει τα δύο χέρια να δείχνουν εκεί ώστε οι άκρες να συμπίπτουν.
    static void SnapToConsistentHome(out Vector3 planeN, out Vector3 eePos)
    {
        Transform leftCrank = s_links["left_crank"], rightCrank = s_links["right_crank"];
        Transform leftArm = s_links["left_arm"], rightArm = s_links["right_arm"], ee = s_links["end_effector_link"];

        // Άξονας περιστροφής κινητήρα = κάθετη στο επίπεδο του μηχανισμού.
        // ΓΕΩΜΕΤΡΙΚΟΣ υπολογισμός (δεν χρειάζεται ArticulationBody): κάθετη στο επίπεδο
        // που ορίζουν οι δύο κινητήρες και ο αριστερός αγκώνας.
        planeN = Vector3.Cross(rightCrank.position - leftCrank.position,
                               leftArm.position - leftCrank.position).normalized;
        if (planeN.sqrMagnitude < 1e-9f) planeN = Vector3.up;

        // Πραγματικό μήκος χεριού (rigid local offset - ανεξάρτητο από το σπασμένο home)
        Vector3 tipLocal = leftArm.InverseTransformPoint(ee.position);
        float armLen = tipLocal.magnitude;

        Vector3 pa = leftArm.position, pb = rightArm.position;        // θέσεις αγκώνων
        Vector3 dv = pb - pa; float d = Mathf.Max(dv.magnitude, 1e-6f);
        float aDist = d * 0.5f;                                        // ίσες ακτίνες -> μέσο
        float h2 = armLen * armLen - aDist * aDist;
        float h = h2 > 0f ? Mathf.Sqrt(h2) : 0f;
        Vector3 m = pa + (aDist / d) * dv;
        Vector3 perp = Vector3.Cross(planeN, dv).normalized;
        Vector3 refCenter = (leftCrank.position + rightCrank.position) * 0.5f;
        Vector3 c1 = m + h * perp, c2 = m - h * perp;
        // διάλεξε την τομή στην πλευρά εργασίας (μακρύτερα από το κέντρο των κινητήρων)
        eePos = (c1 - refCenter).sqrMagnitude >= (c2 - refCenter).sqrMagnitude ? c1 : c2;

        Vector3 armDirL = (leftArm.TransformPoint(tipLocal) - leftArm.position).normalized;
        Vector3 armDirR = (rightArm.TransformPoint(tipLocal) - rightArm.position).normalized;
        leftArm.rotation  = Quaternion.FromToRotation(armDirL, (eePos - pa).normalized) * leftArm.rotation;
        rightArm.rotation = Quaternion.FromToRotation(armDirR, (eePos - pb).normalized) * rightArm.rotation;
        ee.position = eePos;
        ee.rotation = leftArm.rotation;
    }
}
#endif
