// ImpactLogger.cs - EXPERIMENT (only in the scratch copy): logs at EVERY step
// all contact notifications of the EE (enter/stay/exit) with the force impulse/dt.
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

public class ImpactLogger : MonoBehaviour
{
    public string fileName = "impact_steps.csv";
    readonly List<string> rows = new List<string>();
    Rigidbody rb;

    void Awake() { rb = GetComponent<Rigidbody>(); }

    void Rec(string ev, Collision c)
    {
        Vector3 F = c.impulse / Time.fixedDeltaTime;
        rows.Add(string.Format(CultureInfo.InvariantCulture, "{0:F4},{1},{2},{3:F4},{4:F4},{5:F5}",
            Time.fixedTime, ev, c.gameObject.name, F.magnitude, F.z, rb.linearVelocity.z));
    }

    void OnCollisionEnter(Collision c) => Rec("enter", c);
    void OnCollisionStay(Collision c)  => Rec("stay", c);
    void OnCollisionExit(Collision c)  => Rec("exit", c);

    void OnDisable()
    {
        File.WriteAllText(Path.Combine(Application.dataPath, "..", fileName),
            "t,event,other,F_mag,F_z,vz\n" + string.Join("\n", rows) + "\n");
    }
}
