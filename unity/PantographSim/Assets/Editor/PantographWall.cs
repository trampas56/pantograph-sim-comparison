// PantographWall.cs - Editor-only: adds the contact wall (from world.sdf virtual_wall)
// placed CORRECTLY, computing its position GEOMETRICALLY from the mechanism in Unity
// (not with literal SDF coordinates - the Unity frame is different).
//
//   SDF: box 0.05x0.20x0.08, contact surface at reach=0.120, EE home at 0.0898.
//   -> the wall goes ~30 mm in front of the EE home, perpendicular to the approach axis.
//
// Also adds a SphereCollider (r=0.015, from model_torque.sdf ee_tip) to the
// end_effector so that it CAN touch the wall.
//
// Run in EDIT mode (at home), AFTER Setup has been done.
// Menu: Tools > Pantograph > Add Contact Wall (+ EE collider)
#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;

public static class PantographWall
{
    const string RootName = "five_bar_mechanism";
    const string WallName = "virtual_wall";

    // From the SDF (m)
    const float EeHomeReach      = 0.0898f;  // EE reach at home
    const float WallSurfaceReach = 0.140f;   // contact surface (50mm ahead, easily reachable, far from the singularity ~0.196)
    const float WallThickness    = 0.05f;    // SDF x
    const float WallWidth        = 0.20f;    // SDF y
    const float WallHeight       = 0.08f;    // SDF z
    const float EeColliderRadius = 0.015f;   // SDF ee_tip sphere

    [MenuItem("Tools/Pantograph/Add Contact Wall (+ EE collider)")]
    public static void AddWall()
    {
        if (EditorApplication.isPlaying)
        { Debug.LogError("[PantographWall] Exit Play mode and run again in EDIT mode (at home)."); return; }

        var root = GameObject.Find(RootName);
        if (root == null) { Debug.LogError($"[PantographWall] Root '{RootName}' not found."); return; }

        Transform Find(string n)
        {
            foreach (var t in root.GetComponentsInChildren<Transform>(true))
                if (t.name == n) return t;
            return null;
        }
        var leftCrank = Find("left_crank"); var rightCrank = Find("right_crank");
        var leftArm   = Find("left_arm");   var ee        = Find("end_effector_link");
        if (!leftCrank || !rightCrank || !leftArm || !ee)
        { Debug.LogError("[PantographWall] Links missing - did you run Setup first?"); return; }

        // --- Frame-agnostic axes (same as Setup) ---
        Vector3 n = Vector3.Cross(rightCrank.position - leftCrank.position,
                                  leftArm.position - leftCrank.position).normalized;   // plane normal
        if (n.sqrMagnitude < 1e-9f) n = Vector3.up;
        Vector3 refCenter = (leftCrank.position + rightCrank.position) * 0.5f;          // motor centre
        Vector3 E = ee.position;                                                        // EE home
        Vector3 u = Vector3.ProjectOnPlane(E - refCenter, n).normalized;               // approach axis (reach)
        if (u.sqrMagnitude < 1e-9f) { Debug.LogError("[PantographWall] Could not compute the reach axis."); return; }

        // The wall: surface 30.2mm in front of the EE home, centre = +half the thickness
        float gap = WallSurfaceReach - EeHomeReach;                  // 0.0302 m
        Vector3 center = E + u * (gap + WallThickness * 0.5f);

        // --- Wall GameObject (idempotent) ---
        var existing = GameObject.Find(WallName);
        GameObject wall = existing != null ? existing : GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.name = WallName;
        wall.isStatic = true;
        var rbW = wall.GetComponent<Rigidbody>(); if (rbW) Object.DestroyImmediate(rbW);  // static -> no Rigidbody

        wall.transform.SetPositionAndRotation(center, Quaternion.LookRotation(u, n));
        // local: +x=width(v), +y=height(n), +z=thickness(u)
        wall.transform.localScale = new Vector3(WallWidth, WallHeight, WallThickness);

        // Beige material (URP/Lit)
        var mr = wall.GetComponent<MeshRenderer>();
        if (mr == null) mr = wall.AddComponent<MeshRenderer>();
        var sh = Shader.Find("Universal Render Pipeline/Lit") ?? Shader.Find("Standard");
        var mat = new Material(sh);
        Color beige = new Color(0.96f, 0.96f, 0.86f, 1f);
        mat.color = beige;
        if (mat.HasProperty("_BaseColor")) mat.SetColor("_BaseColor", beige);
        mr.sharedMaterial = mat;

        // PhysicsMaterial (Unity 6: PhysicsMaterial - not the old PhysicMaterial)
        var pm = new PhysicsMaterial("wall_surface")
        {
            dynamicFriction = 0.4f, staticFriction = 0.4f, bounciness = 0f,
            frictionCombine = PhysicsMaterialCombine.Average,
            bounceCombine   = PhysicsMaterialCombine.Minimum,
        };
        var col = wall.GetComponent<BoxCollider>();
        if (col == null) col = wall.AddComponent<BoxCollider>();
        col.sharedMaterial = pm;

        // --- EE collider (sphere) so that it can touch ---
        var eeCol = ee.GetComponent<SphereCollider>();
        if (eeCol == null) eeCol = ee.gameObject.AddComponent<SphereCollider>();
        eeCol.radius = EeColliderRadius;
        eeCol.center = Vector3.zero;
        eeCol.sharedMaterial = pm;

        EditorUtility.SetDirty(wall);
        UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(wall.scene);

        float clearanceMm = (gap - EeColliderRadius) * 1000f;
        Debug.Log($"[PantographWall] Wall OK: contact surface reach=0.120, thickness 5cm, " +
                  $"perpendicular to the approach axis. EE sphere -> wall clearance at home ~ {clearanceMm:F1} mm. " +
                  "NOTE: with kinematic cranks (position), the wall only blocks the DYNAMIC arms - " +
                  "a real contact force needs torque-driven control (next step: force control).");
    }
}
#endif
