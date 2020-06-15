
# Notes on seabot


## launch 

`base.launch` calls:

- `driver.launch`
- `filter.launch`
- `kalman.launch`

`regulation.launch` lance deux noeuds:
- `depth_controler` (uses `seabot_depth_controler`)
-  `waypoint`

Parameters are remapped.

`mission.launch` calls:

- `base.launch`
- `regulation.launch`
- `safety.launch`
- `iridium.launch`


## mission

```
roslaunch seabot mission.launch
```

Quel xml va être utilisé? celui qui correspond au hostname.


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


## discussion TLM

tick = encoche = pulse
tick_per_turn=1
screw_thread doit être 12cm/1415 = 8.48*1e-5

- chercher les infos:
pas de la tige filtée
nombre d'encoches


jeu a une influence sur la connaissance de la position du piston et la régulation

GPS - Iridium

Passage gros piston 

waypoints -100m à 5m

faire remonter le niveau de batterie via le node du piston
dans le noeud du piston, publier dans /driver/power/battery
ajouter un publisher dans le driver du piston
autre l'autre solution, changer

garder le noeud power allumé mais commenter tout ce dont je ne me sers pas: flasher

driver.launch: activer power

power.cpp:
- commenter tous les commandes i2c non pertinentes
- changer adresse dans get_batteries

power.h:
- changer adresse i2c PIC
- ADC_BATTERY_LEVEL_CONV

lancer une mission complète (mission_empty.xml, mission_depth_only)


## Paramètres kalman dans xml:

```
  gamma_alpha_velocity: 1.0e-3 # Error of model
  gamma_alpha_depth: 1.0e-5 # Error of model
  gamma_alpha_offset: 5.0e-2 # in ticks
  gamma_alpha_chi: 1.0e-3 # in ticks
  gamma_alpha_chi2: 1.0e-3 # in ticks
  gamma_alpha_cz: 1.0e-3 # in ticks

  gamma_init_velocity: 1.0e-1
  gamma_init_depth: 1.0e-2
  gamma_init_offset: 500.0 # in ticks
  gamma_init_chi: 30.0 # in ticks
  gamma_init_chi2: 30.0 # in ticks
  gamma_init_cz: 0.1

  gamma_beta_depth: 1.0e-3
```

Most volume parameters should be normalized by `tick_to_volume`, for example:

```
  const double gamma_init_chi = n_private.param<double>("gamma_init_chi", 30.0)*tick_to_volume; // 20
```

```
  tick_to_volume = (screw_thread/tick_per_turn)*pow(piston_diameter/2.0, 2)*M_PI;
```

In our case:

```
  screw_thread: 8.48e-5
  tick_per_turn: 1
  piston_diameter: 0.0195
```

Hence `tick_to_volume = 8.48e-5*(0.0195/2)**2*3.1415 = 2.53e-8`


