# ELE3000-Self-balanced-bike
Code pour le vélo autoéquilibré réalisé dans le cadre du cours ELE3000

## Code embarqué - Dossier Self_Balanced_Bike
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


## Interface Python - Dossier Read_Data
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
- show
  - Affiche les graphiques

  
## Télécommande infrarouge
- OFF : Active ou désactive la stabilisation
- UP : Accélère le vélo vers l'avant
- DOWN : Accélère le vélo vers l'arrière

## Simulations
### Schémas Simulink
- Moteur : Modélisation du moteur
- Moteur_BF : Contrôle du moteur en BF
- Moteur_BO : Contrôle du moteur en BO
- Velo : Modélisation du vélo
- Velo_BF : Contrôle du vélo en BF, sans le moteur
- Velo_BO : Réponse du vélo en BO
- Velo_Complet_BF : Réponse en BF du système comprenant le vélo et le moteur

### Fichiers de simulations
- parametres : À exécuter en premier pour mettre les paramètres dans le workspace
- simulation_moteur : Pour la conception du contrôleur du moteur
- simulation_velo : Pour la conception du contrôleur du vélo
- tests_validation : Tests de validation