// ContactForceProbe.cs - Διαβάζει τη δύναμη επαφής σε ένα δυναμικό σώμα (= FT αισθητήρας).
// Μπαίνει στο end_effector_link (το βάζει αυτόματα ο PantographForceControl).
// Force = world-space διάνυσμα δύναμης που δέχεται το σώμα από τις επαφές αυτό το βήμα.
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class ContactForceProbe : MonoBehaviour
{
    public Vector3 Force { get; private set; }
    public bool InContact { get; private set; }

    void OnCollisionStay(Collision c)
    {
        // impulse = συνολική ώθηση που εφαρμόστηκε για να λυθεί η επαφή (world space).
        // Δύναμη ~ impulse / dt.
        Force = c.impulse / Mathf.Max(Time.fixedDeltaTime, 1e-6f);
        InContact = true;
    }

    void OnCollisionExit(Collision c)
    {
        Force = Vector3.zero;
        InContact = false;
    }
}
