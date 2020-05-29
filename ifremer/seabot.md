
# Notes on seabot


## launch 

`base.launch` calls:

- `driver.launch`
- `filter.launch`
- `kalman.launch`

`regulation.launch` lance deux noeuds:
- `depth_controler` (uses `seabot_depth_controler`)
-  `waypoint`

what is a parameter remap?

`mission.launch` calls:

- `base.launch`
- `regulation.launch`
- `safety.launch`
- `iridium.launch`




## mission

```
roslaunch seabot mission.launch
```

Quel xml va être utilisé?


## regulation

`src/seabot_depth_controller/src/state_feedback.cpp`

## parameters

config file selected in driver.launch:
```
<rosparam file="$(find seabot)/param/config_$(env HOSTNAME).yaml"/>
```

`HOSTNAME` is `ifremer1`

`src/seabot/param/config_ifremer1.yaml` does not contain much at the moment,
only relevant part seems to be:

```
physics:
  # Float
  m: 18.0
  piston_ref_eq: 2100
```

Bcp de paramètres en dur dans `state_feedback.cpp`:

```
const double frequency = n_private.param<double>("frequency", 25.0);
// Frequency divider
const int divider_frequency = n_private.param<int>("divider_frequency", 5);

const double delta_velocity_lb = n_private.param<double>("delta_velocity_lb", 0.0);
const double delta_velocity_ub = n_private.param<double>("delta_velocity_ub", 0.0);

const double delta_position_lb = n_private.param<double>("delta_position_lb", 0.0);
const double delta_position_ub = n_private.param<double>("delta_position_ub", 0.0);

// Physical characteristics
const double rho = n_private.param<double>("rho", 1025.0);
const double g = n_private.param<double>("g", 9.81);
const double m = n_private.param<double>("m", 8.800);
const double diam_collerette = n_private.param<double>("diam_collerette", 0.24);
const double screw_thread = n_private.param<double>("screw_thread", 1.75e-3);
const double tick_per_turn = n_private.param<double>("tick_per_turn", 48);
const double piston_diameter = n_private.param<double>("piston_diameter", 0.05);
const double piston_max_value = n_private.param<double>("piston_max_value", 2400);
const double Cf = M_PI*pow(diam_collerette/2.0, 2);
tick_to_volume = (screw_thread/tick_per_turn)*pow(piston_diameter/2.0, 2)*M_PI;
coeff_A = g*rho/m;
coeff_B = 0.5*rho*Cf/m;

// Compute regulation constant
s = n_private.param<double>("root_regulation", -1.0);
double limit_depth_regulation = n_private.param<double>("limit_depth_controller", 0.5);
double speed_volume_sink = n_private.param<double>("speed_volume_sink", 2.0);

const double hysteresis_piston = n_private.param<double>("hysteresis_piston", 0.6);
const double piston_max_velocity = n_private.param<double>("piston_speed_max_tick", 30)*tick_to_volume; // in m3/sec

// Hold depth parameters
const bool hold_depth_enable = n_private.param<bool>("hold_depth", false);
const double hold_depth_value_enter = n_private.param<double>("hold_depth_value_enter", 0.05);
const double hold_depth_value_exit = n_private.param<double>("hold_depth_value_exit", 0.1);
```


