# Tests Ifremer 06/2020

--- 

## cuve acoustique: 04/06

Apres fermeture a la maison:

```
   pression = 610
   tension = 12.27
   @16h
```

Le lendemain l'ifremer: pression 580

Premiere pesee dans la cuve acoustique: flotteur trop lourd de 75g

Course piston: 1415
Gros piston: 354
Position balastage:

- 354 + (1415-354)/4 = 619.25
- 618 -> 02 6b

**connection via hostpot mac**:

```
ping 192.168.2.3
ssh pi@192.168.2.3
```

**reset motor speed**:

```
i2cset -y 1 0x38 0x12 0x1E
#30 -> 1E
```

**adjust piston position**:

```
#1415 -> 05 87
i2cset -y 1 0x38 0x10 0x87 0x05 i

#354 -> 01 62
i2cset -y 1 0x38 0x10 0x62 0x01 i

#0
i2cset -y 1 0x38 0x10 0x00 0x00 i

#position balastage:
# 354 + (1415-354)/4 = 619.25
# 618 -> 02 6b
i2cset -y 1 0x38 0x10 0x6b 0x02 i
```

**balastage**:

Avec `72 + 51 = 123g` (air, 109g eau) pese 30g dans l'eau doit donc enlever 30g (eau):
109 - 30 eau = 79 eau = 88g air

107g dans l'air = 95g dans l'eau.
Donc **facteur 0.89**

Une fois corrige est neutre dans l'eau (visuel + verif balance avec poids supplementaire)

Poids amant + scotch air = 11g. Aimant seul 3g.


### mission 0

- plonge vite
- "tape" le fond
- reste 5min
- resurface, replonge et reste quasiment en dessous de la surface
- replonge vitesse reduite et retouche le fond
- remonte 
- resurface

```
# rapatrie bags
scp pi@192.168.2.3:'.ros/*.bag' .
```

Inspection du bag:

```
pybag ...
```

Piston rentre bien a 1000 comme on lui demande ce qui est surement un peu trop
diminuer le parametre d'equilibre dans le yaml.
*On passera à 500*

La profondeur de la cuve est telle que le flotteur ne peu atteindre une profondeur que de 80cm
pb capteur de pression ou profondeur reelle?
Solution: le faire plutot flottant

Le piston resort bien lorsqu'il faut remonter 
lorsqu'il depasse la consigne par contre il rerentre son piston de maniere lineaire je ne comprends pas pourquoi.

Biais bizar pour le capteur de pression.
Explication possibles:
pb - driver; pb - soit capteur.

Repasse en mode sink:
`limit_depth_controller` trop grand: 0.5
*on passera a 0.2 m/s et kalman a 0.1*

Bruit capteur pression relatif de 10cm

Vitesse regulation trop importante, a changer dans le fichier de mission (`.xml`):
actuellement 5 cm/s (onglet regulation 1).
Mais dernier truc a regarder d'après Thomas.

u sature.

`root_regulation` peut etre a modifier pour cette saturation.
-1 tres reactif (valeurs en Hz).

`delta_velocity`: autorise une consigne dans un interval.
Mettre a 0 pour desactiver

`speed_volume_sink`: 2.0 tick par seconde pour couler.
Responsable de la faible pente apres arriver a la pression d'equilibre qd en mode sink (i.e. moins profonde que 50cm)

`hold_depth`: arrete de bouger le piston quand dans une plage donnee
pour plus tard quand 

kalman:

- chi et chi2 sont plus faible pour nous
- valeurs coherentes 
- tick par metre 
- faire le calcul ...

Mesure metre ruban profondeur capteur pression qd au fond: 85cm

### mission 1

piston rentre bien a 1000 comme on lui demande ce qui est surement un peu trop
diminuer le parametre d'equilibre dans le yaml
*On passera a 500*

`limit_depth_controller` trop grand: 0.5
*on passera a 0.2 m/s et kalman a 0.1*

mais oublie de modifier profondeurs de mission

### mission 2

idem mais a 60cm et 30cm de profondeur et `limit_velocity=3cm/s`

### mission 3

en diminuant la rapidite de regulation: `root_regulation` a 0.1

- touche le fond
- remonte


### mission 4

`root_regulation` a 0.05

roslaunch driver: pression a 0.19



---

## Bassin: 10/06

Pression interne: 615; vide ok

Gros leste: 566g (eau) - 621 (air): **facteur 0.91**

Flotteur + gros leste: 100-105g.
Il faut donc rajouter 566-102 = 464g (eau) soit 510g (air)

On ajoute 400g+107g (air).
Visuellement neutre

Edition seabot:

- Valeur initiale chi filtre Kalman: -1.0 tick_to_volume
- recompilation
- change mission: 300s a 1m et 300s a 2m
- change parametre kalman:

```
  gamma_alpha_chi2: 1.0e-5 # in ticks
  gamma_init_chi: 1.0 # in ticks
  gamma_init_chi2: 1.0e-2 # in ticks
  gamma_beta_depth: 1.0e-2
```

### mission 1

Lance mission et enregistre log:

```
roslaunch seabot mission.launch >20200610_m1.log 2>&1 
    piston (seabot[ERROR] [1591777677.134601003]: [GPSD_Client] Failed to open GPSd
[driver/gpsd_client-8] process has died [pid 4721, exit code 255, cmd /home/pi/seabot/devel/lib/gpsd_client/gpsd_client __name:=gpsd_client __log:=/home/pi/.ros/log/4592df26-aaf4-11ea-a3b7-e84e064b777b/driver-gpsd_client-8.log].
log file: /home/pi/.ros/log/4592df26-aaf4-11ea-a3b7-e84e064b777b/driver-gpsd_client-8*.log
Error: open: Permission denied
```

erreur mais fichier bien enregistré en fin de mission

Oscillations pres de la surface


### mission 2

- change pression d'equilibre 1000
- `gamma_beta_depth: 1.0e-1`

### mission 3


- augmente reactivite `root_regulation: -0.1`
- reset piston speed a 30: `i2cset -y 1 0x38 0x12 0x1E`

### mission 4

augment vitesse d'approche: 0.1 


### mission 5

- ` gamma_alpha_offset: 5.0e-1 # in ticks`

m3 de variation de secondes

### tests balastage:

met piston a 5600 (1400)
i2cset -y 1 0x38 0x10 0xE0 0x15 i

ne va pas jusqu'au fond initialement


### mission 6

on ajoute 15g
pb avec un capteur passe en safety

2m puis 1m

quelques rebonds au fond

2m trop profond

### mission 6bis idem

- 1.5m puis 1.m
- `gamma_alpha_offset: 5.0e0 # in ticks`

### mission 7


### mission 8

- augmente gamma_alpha_chi: **combien??**
- ` gamma_alpha_chi: 1.0e-2 # in ticks`


### mission 9

```
    root_regulation: -0.5

    delta_velocity_lb: 0.0 
    delta_velocity_ub: 0.0 

    delta_position_lb: 0.0  
    delta_position_ub: 0.0  
```

Le flotteur n'a pas la flottabilité pour remonter.
Un test en position balastage confirme que le flotteur coule

### mission 10:

on enleve 15g



### Autres tests:

mettre toutes les delta_velocity de la regulation a 0

aller a 2m d'abord

verifier que 1400 est bien 
simuler butees dans le pic

si mauvaise conversion et nombre de tick peut aussi poser pb

aller a 5m tout de suite pour voir si le flotteur y va



---

## bassin 18/06

Mesure jeu course:

**reset motor speed and error_interval**:

```
i2cset -y 1 0x38 0x12 0x1E
#30 -> 1E
i2cset -y 1 0x38 0x05 0x00
python monitor_pic.py
```

**position de reference**: 
gros piston (354) + 1cm (1415/12=118) = 472 ~ 470
adjuste manually: 1mm = 12 pulses

Create function in .zchrc to set position
rostopic pub -r 1 /driver/piston/position seabot_piston_driver/PistonPosition '{position: 0, stamp: {secs: 1, nsecs: 0 }}'
set_piston_position() {
echo "{position: $1, stamp: {secs: 1, nsecs: 0 }}"
#rostopic pub -r 1 /driver/piston/position seabot_piston_driver/PistonPosition "{position: $1, stamp: {secs: 1, nsecs: 0 }}"
}


2 tests a chaque fois:
- sortie puis rentree
set_piston_position 500
set_piston_position 470
set_piston_position 475
set_piston_position 480

...
- rentree puis sortie
set_piston_position 0
set_piston_position 470
set_piston_position 465
set_piston_position 460
...

essayer plusieurs intervals:
5, 10, 15

mesurer les positions a chaque deplacement


### mission 0:

follow Thomas advice:

  gamma_alpha_velocity: 1.0e-3 # Error of model
  gamma_alpha_depth: 1.0e-5 # Error of model
  gamma_alpha_offset: 5.0e0 # in ticks
  gamma_alpha_chi: 1.0e-3 # in ticks
  gamma_alpha_chi2: 1.0e-5 # in ticks
  gamma_alpha_cz: 1.0e-4

  gamma_init_velocity: 1.0e-1
  gamma_init_depth: 1.0e-2
  gamma_init_offset: 500.0 # in ticks
  gamma_init_chi: 1.0 # in ticks
  gamma_init_chi2: 1.0e-2 # in ticks
  gamma_init_cz: 0.1

  gamma_beta_depth: 0.5

  init_chi: -1.0
  init_chi2: 0.0


### mission 1:

augmenter gamma_alpha_offset

diminuer approach velocity pour imposer des vitesses superieures a l'erreur de mesure liee au capteur de pression

