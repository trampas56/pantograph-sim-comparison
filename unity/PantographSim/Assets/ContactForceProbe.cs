// ContactForceProbe.cs - Reads the contact force on a dynamic body (= FT sensor).
// Goes on end_effector_link (PantographForceControl adds it automatically).
// Force = world-space force vector the body receives from the contacts in this step.
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class ContactForceProbe : MonoBehaviour
{
    public Vector3 Force { get; private set; }
    public bool InContact { get; private set; }

    void OnCollisionStay(Collision c)
    {
        // impulse = total impulse applied to resolve the contact (world space).
        // Force ~ impulse / dt.
        Force = c.impulse / Mathf.Max(Time.fixedDeltaTime, 1e-6f);
        InContact = true;
    }

    void OnCollisionExit(Collision c)
    {
        Force = Vector3.zero;
        InContact = false;
    }
}
