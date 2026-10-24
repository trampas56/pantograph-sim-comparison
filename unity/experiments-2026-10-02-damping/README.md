# Damping and time-step test in Unity

I ran this test on 2 October 2026 to find out why in Unity the sensor shows about 8% less force than the controller computes. In the thesis this is Section 10.4 and Equation 10.1:

F_sensor = (1 - cΔt) F_est

where c is the damping on the motor arms (motorAngularDamping, 40 in the project) and Δt is the physics time step (2 ms).

## How it was done

The script harness/DampingExperiment.cs opens the scene, changes only c, Δt and the target force, and runs the simulation for 8 s. The mean values are taken from 5 s onwards. harness/ImpactLogger.cs writes the contact force at every step, to show the impact on the wall.

The two scripts are not inside PantographSim. To run the test again, put them in a copy of the project, DampingExperiment.cs in Assets/Editor and ImpactLogger.cs in Assets, and then:

```
EXP_CFGS="40:0.002:2,80:0.001:2" EXP_OUT=$PWD/results.csv Unity -batchmode -nographics -projectPath <copy>/PantographSim -executeMethod DampingExperiment.Run
```

In EXP_CFGS each setting is written as c:Δt:F, with Δt in seconds. With EXP_IMPACT=1 the contact force at every step is written as well.

## Results

| c | Δt | F | 1 - cΔt | F_sensor / F_est |
|---|---|---|---|---|
| 40 | 2 ms | 2 N | 0.920 | 0.920 |
| 80 | 1 ms | 2 N | 0.920 | 0.920 |
| 40 | 2 ms | 1 N | 0.920 | 0.920 |
| 40 | 1 ms | 2 N | 0.960 | 0.960 |
| 40 | 1 ms | 1 N | 0.960 | 0.960 |
| 20 | 1 ms | 2 N | 0.980 | 0.980 |
| 40 | 0.5 ms | 2 N | 0.980 | 0.980 |
| 5 | 1 ms | 2 N | 0.995 | 0.996 |

The ratio F_sensor / F_est is almost equal to 1 - cΔt (the difference is below 0.001) and depends only on the product cΔt. This happens because at every step PhysX first adds the motor torque to the velocity and then multiplies the velocity by 1 - cΔt for the damping. So at every step a fraction cΔt of the torque is lost, even when the arm is at rest against the wall.

With the scene as it is saved (c = 40, Δt = 2 ms, 2 N) the sensor reads 1.36 N, the same as in the June measurement.

## Impact on the wall (2 N, c = 40)

| Δt | Peak force | Impulse in the first 30 ms |
|---|---|---|
| 2 ms | 8.6 N | 71 mNs |
| 1 ms | 17.4 N | 70 mNs |
| 0.5 ms | 35 N | 70 mNs |

The impulse stays the same, but the peak force doubles each time the step is halved. So the height of the peak depends on the step and is not a real measurement.

## Files

- damping_sweep_results.csv: the mean values for the 8 settings in the table
- asis_2N_dt2ms_result.csv: the scene as it is saved
- runs/sweep_*.csv: the full log for 5 of the 8 settings. The other 3 were overwritten by later runs and only their mean values remain.
- runs/impact_steps_*.csv: the contact force at every step, for Δt 0.5, 1 and 2 ms
- harness: the two scripts

The Unity measurements for Table 11.1 are in thesis/data/force (unity_0.5N.csv, unity_1N.csv, unity_2N.csv), all with Δt = 2 ms. The 0.5 N run was done without graphics and the other two from the editor.
