# ELE3000-Self-balanced-bike
Code pour le vélo autoéquilibré pour mon projet dans le cadre du cours ELE3000

## Tâches à faire
- [x] Identifier le moteur
- [] Simulations
    - [x] Contrôleur du moteur
    - [] Contrôleur du vélo
- [] Déterminer la fréquence du système
- [x] Code embarqué
  - [x] Contrôleur en accélération
  - [x] Contrôleur de l'angle


## Code embarqué
### Commandes possibles
**Toutes les commandes commencent par '#'**

#### Commandes relatives au moteur
- On
- Off
- setSpeed _newTargetSpeed
- setKpM _newKp
- setKdM _newKd
- setKiM _newKi
- step _stepVoltage
- accel _ramp

#### Commandes relatives au vélo
- stabilize
- setKpV _newKp
- setKdV _newKd
- setKiV _newKi


## Interface Python
### Commandes possibles
- help
- run in_type
  - s : Stabilize bike
  - ramp: Follow ramp
  - step: Follow step
- step stepVal
  - Follow step and identify motor
- setKxV newValue
  - Change Kx of bike
- setKxM newValue
  - Change Kx of motor
  

## Télécommande infrarouge