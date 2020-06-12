

# apres fermeture a la maison:
#   pression = 610
#   tension = 12.27
#   @16h


# le lendemain l'ifremer: pression 580

# premiere pesee dans la cuve acoustique: flotteur trop lourd de 75g

# course piston 1415
# gros piston 354
# position balastage:
354 + (1415-354)/4 = 619.25
618 -> 02 6b

# connection via hostpot mac 
ping 192.168.2.3
ssh pi@192.168.2.3


# reset motor speed
#i2cset -y 1 0x38 0x12 0x1E
#30 -> 1E

i2cset -y 1 0x38 0x10 0x87 0x05 i
1415 -> 05 87

i2cset -y 1 0x38 0x10 0x62 0x01 i
354 -> 01 62

i2cset -y 1 0x38 0x10 0x00 0x00 i

position balastage:
354 + (1415-354)/4 = 619.25
618 -> 02 6b
i2cset -y 1 0x38 0x10 0x6b 0x02 i

# avec 72 + 51 = 123g (air, 109g eau) pese 30g dans l'eau
doit donc enlever 30g (eau):
109 - 30 eau = 79 eau = 88g air

107g dans l'air = 95g dans l'eau
facteur 0.89

une fois corrige est neutre dans l'eau (visuel + verif balance avec poids supplementaire)

poids amant + scotch air = 11g
aimant seul 3g

plonge vite
"tape" le fond
reste 5min
resurface, replonge et reste quasiment en dessous de la surface
replonge vitesse reduite et retouche le fond
remonte 
resurface

scp pi@192.168.2.3:'.ros/*.bag' .

Inspection du bag:
pybag ...

piston rentre bien a 1000 comme on lui demande ce qui est surement un peu trop
diminuer le parametre d'equilibre dans le yaml
** passe 500 

la profondeur de la cuve est telle que le flotteur ne peu atteindre une profondeur que de 80cm
pb capteur de pression ou profondeur reelle?
solution: le faire plutot flottant

le piston resort bien lorsqu'il faut remonter 
lorsqu'il depasse la consigne par contre il rerentre son piston de maniere lineaire je ne comprends pas pourquoi

biais bizar pour le capteur de pression
pb - driver
pb - soit capteur

repasse en mode sink
limit_depth_controller trop grand: 0.5
** on passe a 0.2 m/s 
** et kalman a 0.1

bruit capteur pression relatif 10cm

vitesse regulation trop importante, a changer dans le fichier de mission
actuellement 5 cm/s (onglet regulation 1)
dernier truc a regarder
u sature: 

root regulation peut etre a modifier pour cette saturation
-1 tres reactif
valeurs en Hz

delta_velocity: autorise une consigne dans un interval
mettre a 0 pour desactiver

speed_volume_sink:
2.0
tick par seconde pour couler
responsable de la faible pente apres arriver a la pression d'equilibre qd en mode sink (i.e. moins profonde que 50cm)

hold depth: arrete de bouger le piston quand dans une plage donnee
pour plus tard quand 

kalman:
chi et chi2 sont plus faible pour nous
valeurs coherentes 
tick par metre 
faire le calcul ...

Mesure metre ruban profondeur capteur pression qd au fond: 85cm

--- Deuxieme mission:

piston rentre bien a 1000 comme on lui demande ce qui est surement un peu trop
diminuer le parametre d'equilibre dans le yaml
** passe 500

limit_depth_controller trop grand: 0.5
** on passe a 0.2 m/s
** et kalman a 0.1

mais oublie de modifier profondeurs de mission

--- troisieme mission

idem mais a 60cm et 30cm de profondeur et limit_velocity=3cm/s

--- 4eme mission:
en diminuant la rapidite de regulation
root a 0.1

touche le fond
remonte


--- 5eme mission:

regulation root a 0.05


roslaunch driver: pression a 0.19

------------------------------------------------------------------------------

Bassin

Pression interne: 615
Vide ok

Gros leste: 566g (eau) - 621 (air): facteur 0.91
Flotteur + gros leste: 100-105g
Il faut donc rajouter 566-102 = 464g (eau) soit 510g (air)

On ajoute 400g+107g (air)
Visuellement neutre

Edition seabot:
- Valeur initiale chi filtre Kalman: -1.0 tick_to_volume
- recompilation
- change mission: 300s a 1m et 300s a 2m
- change parametre kalman:
  gamma_alpha_chi2: 1.0e-5 # in ticks
  gamma_init_chi: 1.0 # in ticks
  gamma_init_chi2: 1.0e-2 # in ticks
  gamma_beta_depth: 1.0e-2

roslaunch seabot mission.launch >20200610_m1.log 2>&1 
    piston (seabot[ERROR] [1591777677.134601003]: [GPSD_Client] Failed to open GPSd
[driver/gpsd_client-8] process has died [pid 4721, exit code 255, cmd /home/pi/seabot/devel/lib/gpsd_client/gpsd_client __name:=gpsd_client __log:=/home/pi/.ros/log/4592df26-aaf4-11ea-a3b7-e84e064b777b/driver-gpsd_client-8.log].
log file: /home/pi/.ros/log/4592df26-aaf4-11ea-a3b7-e84e064b777b/driver-gpsd_client-8*.log
Error: open: Permission denied

oscillations pres de la surface


--- mission 2:

change pression d'equilibre 1000
  gamma_beta_depth: 1.0e-1

--- mission 3:

i2cset -y 1 0x38 0x12 0x1E

augmente reactivite
    root_regulation: -0.1 
reset piston speed a 30

--- mission 4
augment vitesse d'approche: 0.1 


--- mission 5
  gamma_alpha_offset: 5.0e-1 # in ticks

m3 de variation de secondes

--- test balastage:

met piston a 5600 (1400)
i2cset -y 1 0x38 0x10 0xE0 0x15 i

ne va pas jusqu'au fond initialement

--- mission 6

on ajoute 15g
pb avec un capteur passe en safety

2m puis 1m

--- mission 6bis idem

quelques rebonds au fond
2m trop profond

--- mission 6bis idem
1.5m puis 1.m
  gamma_alpha_offset: 5.0e0 # in ticks

--- mission 7


--- mission 8
augmente gamma_alpha_chi
  gamma_alpha_chi: 1.0e-2 # in ticks


--- mission 9
    root_regulation: -0.5

    delta_velocity_lb: 0.0 
    delta_velocity_ub: 0.0 

    delta_position_lb: 0.0  
    delta_position_ub: 0.0  

le flotteur n'a pas la flottabilit√pour remonter
un test en position balastage confirme que le flotteur coule

---

on enleve 15g



--- Autres tests:

mettre toutes les delta_velocity de la regulation a 0

aller a 2m d'abord

verifier que 1400 est bien 
simuler butees dans le pic

si mauvaise conversion et nombre de tick peut aussi poser pb

aller a 5m tout de suite pour voir si le flotteur y va

