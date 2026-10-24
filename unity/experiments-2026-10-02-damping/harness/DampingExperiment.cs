// DampingExperiment.cs - ΠΕΙΡΑΜΑ (μόνο στο scratch αντίγραφο, όχι στο project).
// Για κάθε (c = motorAngularDamping, dt = fixedDeltaTime, F = targetForceN) τρέχει Play,
// και μετρά μέσες τιμές F_est και F_sensor (contactF) στο διάστημα [avgFrom, simSec].
// Εκτέλεση: Unity -batchmode -nographics -projectPath ... -executeMethod DampingExperiment.Run
// env: EXP_CFGS="40:0.002:2,40:0.001:2"  EXP_SIM_SEC=8  EXP_OUT=/path/results.csv
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

public static class DampingExperiment
{
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;
    static readonly List<(float c, float dt, float f)> cfgs = new List<(float, float, float)>();
    static int idx = -1;
    static float simSec = 8f, avgFrom = 5f, runDt;
    static string outPath, csvName;
    static bool stopping;

    public static void Run()
    {
        string spec = Environment.GetEnvironmentVariable("EXP_CFGS") ?? "40:0.002:2,40:0.001:2";
        if (spec == "asis") cfgs.Add((float.NaN, float.NaN, float.NaN));   // σκηνή όπως είναι, χωρίς αλλαγές
        else foreach (var s in spec.Split(','))
        {
            var p = s.Split(':');
            cfgs.Add((float.Parse(p[0], Inv), float.Parse(p[1], Inv), float.Parse(p[2], Inv)));
        }
        if (float.TryParse(Environment.GetEnvironmentVariable("EXP_SIM_SEC"), NumberStyles.Float, Inv, out var ss)) simSec = ss;
        outPath = Environment.GetEnvironmentVariable("EXP_OUT") ?? Path.GetFullPath("exp_results.csv");
        File.WriteAllText(outPath, "c,dt_ms,F_target,c_dt,dt_runtime_ms,F_est,F_sensor,F_sensor_std,ratio,pred_1_minus_c_dt,n\n");

        // χωρίς domain reload -> τα static (λίστα, δείκτης) επιβιώνουν από το Play
        EditorSettings.enterPlayModeOptionsEnabled = true;
        EditorSettings.enterPlayModeOptions = EnterPlayModeOptions.DisableDomainReload;
        EditorApplication.playModeStateChanged += OnState;
        EditorApplication.update += OnUpdate;
        Next();
    }

    static void Next()
    {
        idx++;
        if (idx >= cfgs.Count) { Debug.Log("[EXP] DONE"); EditorApplication.Exit(0); return; }
        var (c, dt, f) = cfgs[idx];
        EditorSceneManager.OpenScene("Assets/Scenes/SampleScene.unity", OpenSceneMode.Single);
        var root = GameObject.Find("five_bar_mechanism");
        var fc = root.GetComponent<PantographForceControl>();
        var pd = root.GetComponent<PantographPhysicsDrive>();
        csvName = $"exp_{idx}.csv";
        if (float.IsNaN(c)) { fc.csvFileName = csvName; stopping = false; Debug.Log($"[EXP] run {idx}: as-is"); EditorApplication.isPlaying = true; return; }
        fc.motorAngularDamping = c;
        fc.fixedDt = dt;
        fc.targetForceN = f;
        fc.startDelaySec = 0f;
        fc.csvFileName = csvName;
        if (pd) pd.fixedDt = dt;   // ίδιο dt και στα δύο Awake -> δεν παίζει ρόλο η σειρά τους
        if (Environment.GetEnvironmentVariable("EXP_IMPACT") == "1")
            GameObject.Find("end_effector_link").AddComponent<ImpactLogger>().fileName = $"impact_{idx}.csv";
        stopping = false;
        Debug.Log($"[EXP] run {idx}: c={c} dt={dt} F={f}");
        EditorApplication.isPlaying = true;
    }

    static void OnUpdate()
    {
        if (!EditorApplication.isPlaying || stopping) return;
        if (Time.time >= simSec)
        {
            runDt = Time.fixedDeltaTime;
            stopping = true;
            EditorApplication.isPlaying = false;
        }
    }

    static void OnState(PlayModeStateChange s)
    {
        if (s != PlayModeStateChange.EnteredEditMode) return;
        try { Collect(); } catch (Exception e) { Debug.LogError("[EXP] collect failed: " + e); }
        Next();
    }

    static void Collect()
    {
        var (c, dt, f) = cfgs[idx];
        string path = Path.Combine(Application.dataPath, "..", csvName);
        double sEst = 0, sF = 0, sF2 = 0; int n = 0;
        foreach (var line in File.ReadAllLines(path))
        {
            var p = line.Split(',');
            if (p.Length < 7 || !double.TryParse(p[0], NumberStyles.Float, Inv, out var t)) continue;
            if (t < avgFrom) continue;
            double est = double.Parse(p[2], Inv), fs = double.Parse(p[6], Inv);
            sEst += est; sF += fs; sF2 += fs * fs; n++;
        }
        if (n == 0) { Debug.LogError($"[EXP] run {idx}: no rows"); return; }
        double mEst = sEst / n, mF = sF / n, sd = Math.Sqrt(Math.Max(0, sF2 / n - mF * mF));
        string row = string.Format(Inv, "{0},{1},{2},{3:F4},{4:F3},{5:F4},{6:F4},{7:F4},{8:F4},{9:F4},{10}",
            c, dt * 1000f, f, c * dt, runDt * 1000f, mEst, mF, sd, mF / mEst, 1.0 - c * dt, n);
        File.AppendAllText(outPath, row + "\n");
        Debug.Log("[EXP] result " + row);
    }
}
