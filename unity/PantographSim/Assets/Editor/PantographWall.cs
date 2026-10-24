// PantographWall.cs - Editor-only: προσθέτει τον τοίχο επαφής (από world.sdf virtual_wall)
// ΣΩΣΤΑ τοποθετημένο, υπολογίζοντας τη θέση ΓΕΩΜΕΤΡΙΚΑ από τον μηχανισμό στο Unity
// (όχι με literal SDF συντεταγμένες - το frame του Unity είναι διαφορετικό).
//
//   SDF: box 0.05x0.20x0.08, επιφάνεια επαφής στο reach=0.120, EE home στο 0.0898.
//   -> ο τοίχος μπαίνει ~30 mm μπροστά από το EE home, κάθετα στον άξονα προσέγγισης.
//
// Προσθέτει επίσης SphereCollider (r=0.015, από model_torque.sdf ee_tip) στο
// end_effector ώστε να ΜΠΟΡΕΙ να ακουμπήσει τον τοίχο.
//
// Τρέξε σε EDIT mode (στο home), ΑΦΟΥ έχει γίνει το Setup.
// Menu: Tools > Pantograph > Add Contact Wall (+ EE collider)
#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;

public static class PantographWall
{
    const string RootName = "five_bar_mechanism";
    const string WallName = "virtual_wall";

    // Από το SDF (m)
    const float EeHomeReach      = 0.0898f;  // reach του EE στο home
    const float WallSurfaceReach = 0.140f;   // επιφάνεια επαφής (50mm μπροστά, φτάνει άνετα, μακριά από singularity ~0.196)
    const float WallThickness    = 0.05f;    // SDF x
    const float WallWidth        = 0.20f;    // SDF y
    const float WallHeight       = 0.08f;    // SDF z
    const float EeColliderRadius = 0.015f;   // SDF ee_tip sphere

    [MenuItem("Tools/Pantograph/Add Contact Wall (+ EE collider)")]
    public static void AddWall()
    {
        if (EditorApplication.isPlaying)
        { Debug.LogError("[PantographWall] Βγες από Play mode και ξανατρέξε σε EDIT mode (στο home)."); return; }

        var root = GameObject.Find(RootName);
        if (root == null) { Debug.LogError($"[PantographWall] Δεν βρέθηκε root '{RootName}'."); return; }

        Transform Find(string n)
        {
            foreach (var t in root.GetComponentsInChildren<Transform>(true))
                if (t.name == n) return t;
            return null;
        }
        var leftCrank = Find("left_crank"); var rightCrank = Find("right_crank");
        var leftArm   = Find("left_arm");   var ee        = Find("end_effector_link");
        if (!leftCrank || !rightCrank || !leftArm || !ee)
        { Debug.LogError("[PantographWall] Λείπουν links - έτρεξες πρώτα το Setup;"); return; }

        // --- Frame-agnostic άξονες (ίδιοι με το Setup) ---
        Vector3 n = Vector3.Cross(rightCrank.position - leftCrank.position,
                                  leftArm.position - leftCrank.position).normalized;   // κάθετη επιπέδου
        if (n.sqrMagnitude < 1e-9f) n = Vector3.up;
        Vector3 refCenter = (leftCrank.position + rightCrank.position) * 0.5f;          // κέντρο κινητήρων
        Vector3 E = ee.position;                                                        // EE home
        Vector3 u = Vector3.ProjectOnPlane(E - refCenter, n).normalized;               // άξονας προσέγγισης (reach)
        if (u.sqrMagnitude < 1e-9f) { Debug.LogError("[PantographWall] Αδύνατος υπολογισμός reach axis."); return; }

        // Ο τοίχος: επιφάνεια 30.2mm μπροστά από το EE home, κέντρο = +μισό πάχος
        float gap = WallSurfaceReach - EeHomeReach;                  // 0.0302 m
        Vector3 center = E + u * (gap + WallThickness * 0.5f);

        // --- Wall GameObject (idempotent) ---
        var existing = GameObject.Find(WallName);
        GameObject wall = existing != null ? existing : GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.name = WallName;
        wall.isStatic = true;
        var rbW = wall.GetComponent<Rigidbody>(); if (rbW) Object.DestroyImmediate(rbW);  // static -> χωρίς Rigidbody

        wall.transform.SetPositionAndRotation(center, Quaternion.LookRotation(u, n));
        // local: +x=πλάτος(v), +y=ύψος(n), +z=πάχος(u)
        wall.transform.localScale = new Vector3(WallWidth, WallHeight, WallThickness);

        // Beige υλικό (URP/Lit)
        var mr = wall.GetComponent<MeshRenderer>();
        if (mr == null) mr = wall.AddComponent<MeshRenderer>();
        var sh = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
        var mat = new Material(sh);
        Color beige = new Color(0.96f, 0.96f, 0.86f, 1f);
        mat.color = beige;
        if (mat.HasProperty("_BaseColor")) mat.SetColor("_BaseColor", beige);
        mr.sharedMaterial = mat;

        // PhysicsMaterial (Unity 6: PhysicsMaterial - όχι το παλιό PhysicMaterial)
        var pm = new PhysicsMaterial("wall_surface")
        {
            dynamicFriction = 0.4f, staticFriction = 0.4f, bounciness = 0f,
            frictionCombine = PhysicsMaterialCombine.Average,
            bounceCombine   = PhysicsMaterialCombine.Minimum,
        };
        var col = wall.GetComponent<BoxCollider>();
        if (col == null) col = wall.AddComponent<BoxCollider>();
        col.sharedMaterial = pm;

        // --- EE collider (sphere) ώστε να μπορεί να ακουμπήσει ---
        var eeCol = ee.GetComponent<SphereCollider>();
        if (eeCol == null) eeCol = ee.gameObject.AddComponent<SphereCollider>();
        eeCol.radius = EeColliderRadius;
        eeCol.center = Vector3.zero;
        eeCol.sharedMaterial = pm;

        EditorUtility.SetDirty(wall);
        UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(wall.scene);

        float clearanceMm = (gap - EeColliderRadius) * 1000f;
        Debug.Log($"[PantographWall] Τοίχος OK: επιφάνεια επαφής reach=0.120, πάχος 5cm, " +
                  $"κάθετος στον άξονα προσέγγισης. Διάκενο σφαίρας EE->τοίχου στο home ~ {clearanceMm:F1} mm. " +
                  "ΠΡΟΣΟΧΗ: με kinematic cranks (θέση), ο τοίχος μπλοκάρει μόνο τα ΔΥΝΑΜΙΚΑ χέρια - " +
                  "για πραγματική δύναμη επαφής χρειάζεται torque-driven έλεγχος (επόμενο βήμα: force control).");
    }
}
#endif
