# Tests Ifremer 07/2020

--- 

## cheat sheet

**!! télécharger code et recompiler si besoin ;) !!**

**!! Check internal pressure !!**

**connection via hostpot mac**:

```
ping 192.168.2.3
ssh pi@192.168.2.3
```

**déplace piston**:

Course piston: 1415
Gros piston: 354
Position balastage: 619.25 (02 6b)

```
# rentré: 1415 -> 05 87
i2cset -y 1 0x38 0x10 0x87 0x05 i
# limite gros piston: 354 -> 01 62
i2cset -y 1 0x38 0x10 0x62 0x01 i
# sortie: 0
i2cset -y 1 0x38 0x10 0x00 0x00 i
#position balastage:
i2cset -y 1 0x38 0x10 0x6b 0x02 i
```

Alternative avec ros (permet de vérifier pression interne):

screen 0:
```
roslaunch seabot driver.launch
```

screen1:
```
rostopic echo /driver/sensor_internal
set_piston_position 619
```

**balastage**:

- Facteur feuille plomb eau/air: 0.9
- Poids amant + scotch air = 11g. Aimant seul 3g.

**reset motor speed**:

```
i2cset -y 1 0x38 0x12 0x1E
#30 -> 1E
```

**lance mission**:

```
roslaunch seabot mission.launch >20200701_m0.log 2>&1
```


```
# rapatrie bags et logs
scp pi@192.168.2.3:'.ros/2020-07-01*.bag' .
scp pi@192.168.2.3:'202007*.log' .
```

Inspection du bag:

```
pybag ...
```



---

Check pressure:

```
pressure: 629.714904785
temperature: 22.4779701233
humidity: 59.9448509216
```

Balastage:

set_piston_position 619
set_piston_position 354
set_piston_position 1415


## mission 0:

5m 300s
2.5m 300s

roslaunch seabot mission.launch >20200701_m0.log 2>&1

coule au fond et y reste

inspection bag le flotteur a l'air mal balaste

pesee la surface en position balastage: pese 7g
donc besoin d'enlever 7/0.9 = 7.7 (air) 
arrondi a 8g

on enleve 107g et rajoute 72+26=98g
soit une difference de 9g


attention aux bulles d'air sous la collerette pendant le balastage
elles peuvent empecher le flotteur de couler


## mission 1:
roslaunch seabot mission.launch >20200701_m1.log 2>&1

larges oscillations


## mission 2:
roslaunch seabot mission.launch >20200701_m2.log 2>&1

on augment approach velocity
on pourrait diminuer limit velocity
on pourrait diminuer root regulation

ne remonte pas, pb diagnostique dans le driver piston relié au cas big piston

## mission 3

roslaunch seabot mission.launch >20200701_m3.log 2>&1

pb big piston devrait etre corrige

il faudra changer la derniere profondeur de la mission la prochaine fois

looks good !

## mission 4

roslaunch seabot mission.launch >20200701_m4.log 2>&1

pareil avec plus de paliers

looks good !!

## mission 5 

roslaunch seabot mission.launch >20200701_m5.log 2>&1

try hold depth avec contrainte sur la vitesse

mais rate à caus de la valeur de hold_depth_exit (0.)

50cm / 1cm/s = 50 s 


sonde:
sal = 36.6
temp = 16.6
a la surface




##

roslaunch seabot mission.launch >20200701_m6.log 2>&1

hold depth still not on

found bug in velocity condition

##

decrease traveling velocity

roslaunch seabot mission.launch >20200701_m7.log 2>&1

larger depth differences

##

try increasing the reactivity of the float
root = 0.2
roslaunch seabot mission.launch >20200701_m8.log 2>&1

##

reincrease velocity to 10cm/s
roslaunch seabot mission.launch >20200701_m9.log 2>&1




# bassin3

scp pi@192.168.2.3:'.ros/2020-07-02*.bag' .
scp pi@192.168.2.3:'20200702*.log' .


## mission0

roslaunch seabot mission.launch >20200702_m0.log 2>&1

condition sur la vitesse enlevee
big piston considere dans state regulation


## mission1

roslaunch seabot mission.launch >20200702_m1.log 2>&1

essaie les delta
eteint hold depth

    delta_velocity_lb: -0.01
    delta_velocity_ub:  0.01

    delta_position_lb: -0.1
    delta_position_ub:  0.1

    hysteresis_piston: 0.6
    speed_volume_sink: 10.0

    hold_depth_enable: False
    hold_depth_value_enter: 0.30
    hold_depth_value_exit: 0.30

## mission2

roslaunch seabot mission.launch >20200702_m2.log 2>&1

idem mais avec modif dans state_regulation

pb: a essaye d'utiliser le gros piston pour se reguler

## mission 3

roslaunch seabot mission.launch >20200702_m3.log 2>&1

interdiction d'utiliser le gros piston pendant la regulation

    root_regulation: -0.2

rate pour le pb de gros piston
regul meilleure


## mission 4

roslaunch seabot mission.launch >20200702_m4.log 2>&1



