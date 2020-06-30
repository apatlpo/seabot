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
# rapatrie bags
scp pi@192.168.2.3:'.ros/2020-07-01*.bag' .
```

Inspection du bag:

```
pybag ...
```
