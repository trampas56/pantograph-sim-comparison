// PantographSetup.cs - Editor-only setup for the five-bar (PATH B: full physics).
// Converts the imported ArticulationBody tree into Rigidbody + joints and
// CLOSES the parallel loop WITHOUT it breaking:
//
//   1) base + two cranks -> KINEMATIC (the base is pinned, the cranks are driven).
//   2) left_arm, right_arm, end_effector -> DYNAMIC (light, no gravity).
//   3) SNAP to the CORRECT closed pose (intersection of 2 circles) BEFORE the joints are added ->
//      the constraint starts with ~0 error (not a 196mm yank that explodes).
//   4) Tree: HingeJoint at the elbows (left_elbow, right_elbow) + ee_left.
//   5) LOOP CLOSURE: ConfigurableJoint right_arm <-> end_effector with
//      linear LOCKED + free spin about the plane normal + PROJECTION -> it does not come apart.
//
// IMPORTANT: run in EDIT mode (not Play). If removing the
// ArticulationBody fails, RELOAD the scene without saving and run again.
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
    // KINEMATIC (driven/pinned) - the rest become dynamic
    static readonly string[] Kinematic =
        { "base_link", "left_crank", "right_crank" };
    // Tree joints: child -> (parent in the hierarchy). NOT on the cranks (they are driven directly).
    // The end_effector is NOT here - it is added as a rigid weld to left_arm (not revolute).
    static readonly string[] TreeHinges =
        { "left_arm", "right_arm" };

    // state passed between the two phases
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
            Debug.LogError("[PantographSetup] Exit Play mode (press Play) and run again in EDIT mode.");
            return;
        }

        s_root = GameObject.Find(RootName);
        if (s_root == null) { Debug.LogError($"[PantographSetup] Root '{RootName}' not found."); return; }

        s_links = new Dictionary<string, Transform>();
        foreach (var t in s_root.GetComponentsInChildren<Transform>(true))
            if (System.Array.IndexOf(LinkNames, t.name) >= 0 && !s_links.ContainsKey(t.name))
                s_links[t.name] = t;
        foreach (var n in LinkNames)
            if (!s_links.ContainsKey(n)) { Debug.LogError($"[PantographSetup] Link '{n}' is missing."); return; }

        // --- PHASE 1: record the joint data & DELETE ALL ArticulationBody components ---
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
                s_anchorPos[n] = ab.anchorPosition;   // joint position in the link's local frame
                s_anchorRot[n] = ab.anchorRotation;   // the revolute axis = X of this frame
            }
        }
        // Note: we NO LONGER depend on ArticulationBody data for the axes -
        // the axis is computed geometrically (SnapToConsistentHome), so that Setup also
        // works on a scene that has ALREADY been converted to Rigidbody (no AB).

        // FIRST remove the URDF helper scripts (UrdfInertial/UrdfJointRevolute/UrdfLink...) that
        // "lock" the ArticulationBody through [RequireComponent], otherwise it is NOT removed.
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
        // NOW the ArticulationBody can be removed
        foreach (var ab in s_root.GetComponentsInChildren<ArticulationBody>(true))
            Object.DestroyImmediate(ab);

        s_retries = 0;
        EditorApplication.delayCall += FinishSetup;   // PHASE 2 on the next editor tick
        Debug.Log("[PantographSetup] ArticulationBody components removed. Finishing on the next tick...");
    }

    static void FinishSetup()
    {
        EditorApplication.delayCall -= FinishSetup;
        if (s_root == null || s_links == null) return;

        // Try a Rigidbody on base_link; if Unity has not "cleaned up" the
        // ArticulationBody yet, AddComponent returns null -> try again on the next tick.
        var baseGo = s_links["base_link"].gameObject;
        var rbTest = baseGo.GetComponent<Rigidbody>();
        if (rbTest == null) rbTest = baseGo.AddComponent<Rigidbody>();
        if (rbTest == null)
        {
            if (s_retries++ < 8) { EditorApplication.delayCall += FinishSetup; return; }
            Debug.LogError("[PantographSetup] Unity does not allow a Rigidbody yet. " +
                           "Reload the scene without saving and run again.");
            return;
        }

        // --- PHASE 2a: Rigidbody on all links ---
        foreach (var n in LinkNames)
        {
            var go = s_links[n].gameObject;
            var rb = go.GetComponent<Rigidbody>();
            if (rb == null) rb = go.AddComponent<Rigidbody>();
            rb.mass = s_mass[n];
            rb.useGravity = false;                  // horizontal mechanism - no gravity
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.collisionDetectionMode = CollisionDetectionMode.ContinuousSpeculative;
            rb.linearDamping = 0.05f;
            rb.angularDamping = 0.05f;
            rb.solverIterations = 80;               // closed-loop stability (per body)
            rb.solverVelocityIterations = 40;
            rb.centerOfMass = Vector3.zero;
            rb.inertiaTensor = Vector3.one * 1e-4f; // small/light links (no colliders)
            rb.isKinematic = false;
        }
        foreach (var n in Kinematic)
            s_links[n].GetComponent<Rigidbody>().isKinematic = true;

        // Remove ALL old joints (from a previous setup A/B) -> idempotent,
        // works on whatever scene you already have.
        foreach (var n in LinkNames)
            foreach (var j in s_links[n].GetComponents<Joint>())
                Object.DestroyImmediate(j);

        // --- PHASE 2b: SNAP to the CORRECT closed pose (intersection of 2 circles) ---
        // Rotate the two arms so that their tips COINCIDE at the EE, BEFORE the
        // constraint is added. This way the loop closure starts with ~0 error -> it does not explode.
        Vector3 planeN, eePos;
        SnapToConsistentHome(out planeN, out eePos);

        // --- PHASE 2c: HingeJoint at the tree joints (elbows + ee) ---
        foreach (var n in TreeHinges)
        {
            var child = s_links[n];
            var parentRb = child.parent ? child.parent.GetComponent<Rigidbody>() : null;
            if (parentRb == null) { Debug.LogWarning($"[PantographSetup] '{n}': no parent Rigidbody."); continue; }

            foreach (var old in child.GetComponents<HingeJoint>()) Object.DestroyImmediate(old);             // idempotent
            foreach (var old in child.GetComponents<ConfigurableJoint>()) Object.DestroyImmediate(old);

            var hj = child.gameObject.AddComponent<HingeJoint>();
            hj.connectedBody = parentRb;
            hj.anchor = s_anchorPos[n];
            hj.axis = child.InverseTransformDirection(planeN).normalized;   // revolute about the plane normal
            hj.autoConfigureConnectedAnchor = true;                         // rest = current (closed) pose
            hj.enableCollision = false;
            hj.enablePreprocessing = false;
        }

        // --- EE: rigid weld (ConfigurableJoint all-locked + projection) to left_arm ---
        // The end_effector is the tip of the left arm; we hold it RIGIDLY on it
        // (not revolute) so that there is no free rotational degree of freedom that jitters.
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

        // --- PHASE 2d: LOOP CLOSURE - ConfigurableJoint right_arm <-> end_effector ---
        // = MuJoCo <connect> / Gazebo DetachableJoint. Point-coincidence + free spin
        // about the plane normal, with PROJECTION so that it NEVER comes apart visually.
        var rightArm = s_links["right_arm"];
        var ee = s_links["end_effector_link"];
        foreach (var old in rightArm.GetComponents<ConfigurableJoint>()) Object.DestroyImmediate(old);       // idempotent

        var loop = rightArm.gameObject.AddComponent<ConfigurableJoint>();
        loop.connectedBody = ee.GetComponent<Rigidbody>();
        loop.anchor = rightArm.InverseTransformPoint(eePos);               // tip of the right arm (=EE) at home
        loop.autoConfigureConnectedAnchor = true;                         // connectedAnchor ~ EE centre
        loop.axis = rightArm.InverseTransformDirection(planeN).normalized; // primary = the plane normal
        loop.secondaryAxis = rightArm.InverseTransformDirection((eePos - rightArm.position)).normalized;

        loop.xMotion = loop.yMotion = loop.zMotion = ConfigurableJointMotion.Locked;   // same point (position weld)
        loop.angularXMotion = ConfigurableJointMotion.Free;               // free rotation about the plane normal
        loop.angularYMotion = ConfigurableJointMotion.Locked;             // they stay coplanar
        loop.angularZMotion = ConfigurableJointMotion.Locked;

        loop.projectionMode = JointProjectionMode.PositionAndRotation;    // KEY: hard-snap -> it does not "break"
        loop.projectionDistance = 0.0005f;                                // 0.5mm tolerance
        loop.projectionAngle = 1f;
        loop.enableCollision = false;
        loop.enablePreprocessing = false;                                 // better stability in a stiff loop
        loop.breakForce = Mathf.Infinity;                                 // never breaks
        loop.breakTorque = Mathf.Infinity;

        EditorUtility.SetDirty(s_root);
        UnityEditor.SceneManagement.EditorSceneManager.MarkSceneDirty(s_root.scene);

        float gapMm = Vector3.Distance(rightArm.TransformPoint(loop.anchor), ee.position) * 1000f;
        Debug.Log($"[PantographSetup] Ready (PATH B): 6x Rigidbody, 3 hinges + 1 ConfigurableJoint " +
                  $"(projection ON). base & cranks = kinematic. Closure gap at home = {gapMm:F3} mm. " +
                  "Put PantographPhysicsDrive on the root, Self Test Sweep Deg=8, Play, then SAVE (Ctrl+S).");
    }

    // Intersection of 2 circles of equal radius (= arm length): finds the correct closed EE pose
    // and rotates the two arms to point there so that their tips coincide.
    static void SnapToConsistentHome(out Vector3 planeN, out Vector3 eePos)
    {
        Transform leftCrank = s_links["left_crank"], rightCrank = s_links["right_crank"];
        Transform leftArm = s_links["left_arm"], rightArm = s_links["right_arm"], ee = s_links["end_effector_link"];

        // Motor rotation axis = normal to the mechanism plane.
        // GEOMETRIC computation (no ArticulationBody needed): normal to the plane
        // defined by the two motors and the left elbow.
        planeN = Vector3.Cross(rightCrank.position - leftCrank.position,
                               leftArm.position - leftCrank.position).normalized;
        if (planeN.sqrMagnitude < 1e-9f) planeN = Vector3.up;

        // Actual arm length (rigid local offset - independent of the broken home)
        Vector3 tipLocal = leftArm.InverseTransformPoint(ee.position);
        float armLen = tipLocal.magnitude;

        Vector3 pa = leftArm.position, pb = rightArm.position;        // elbow positions
        Vector3 dv = pb - pa; float d = Mathf.Max(dv.magnitude, 1e-6f);
        float aDist = d * 0.5f;                                        // equal radii -> midpoint
        float h2 = armLen * armLen - aDist * aDist;
        float h = h2 > 0f ? Mathf.Sqrt(h2) : 0f;
        Vector3 m = pa + (aDist / d) * dv;
        Vector3 perp = Vector3.Cross(planeN, dv).normalized;
        Vector3 refCenter = (leftCrank.position + rightCrank.position) * 0.5f;
        Vector3 c1 = m + h * perp, c2 = m - h * perp;
        // pick the intersection on the working side (farther from the centre of the motors)
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
