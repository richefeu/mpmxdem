# Structure de MPMbox (2D)

Ce document décrit, dans les grandes lignes, comment le code est organisé.
L'idée directrice tient en une phrase :

> **MPMbox est un chef d'orchestre qui fait avancer une matière décrite par des
> points matériels sur une grille fixe, et tout ce qui relève d'un *choix de
> modélisation* est un ingrédient interchangeable, choisi par un mot-clé dans le
> fichier d'entrée.**

Chaque « famille » d'ingrédients possède une classe de base (le *contrat* : ce
que l'ingrédient doit savoir faire) et autant de variantes que de choix
disponibles. Ajouter une variante ne modifie pas le cœur du code.

---

## 1. Vue d'ensemble

```mermaid
flowchart TB
  BOX["<b>MPMbox</b><br/><i>le chef d'orchestre</i><br/>lit les données, boucle en temps, sauvegarde"]

  subgraph ETAT["Ce qui est simulé (les données)"]
    direction LR
    MP["<b>MaterialPoint</b><br/>points matériels lagrangiens<br/>masse, position, vitesse,<br/>déformation, contrainte"]
    GRID["<b>grid / node / element</b><br/>grille eulérienne fixe<br/>support des calculs de gradients"]
    OBS["<b>Obstacle</b><br/>corps rigides<br/>qui bornent l'écoulement"]
  end

  subgraph CHOIX["Ce que l'utilisateur choisit (les ingrédients)"]
    direction TB
    CM["<b>ConstitutiveModel</b><br/>comportement du matériau"]
    SF["<b>ShapeFunction</b><br/>transfert points &#8596; grille"]
    OS["<b>OneStep</b><br/>schéma d'intégration en temps"]
    BFL["<b>BoundaryForceLaw</b><br/>loi de contact aux obstacles"]
    CMD["<b>Command</b><br/>mise en données"]
    SCH["<b>Scheduler</b><br/>scénario au cours du temps"]
    SPY["<b>Spy</b><br/>mesures pendant le calcul"]
  end

  BOX --> ETAT
  BOX --> CHOIX
  MP -. "chaque point porte son" .-> CM
  OBS -. "chaque obstacle porte sa" .-> BFL
```

Ces sept familles sont toutes bâties sur le même patron : une classe de base
abstraite, des variantes concrètes, et un nom textuel qui permet de les demander
depuis le fichier d'entrée.

> Le répertoire `VtkOutputs/` existe dans l'arbre des sources mais n'est ni
> compilé (absent du `file(GLOB ...)` de `CMakeLists.txt`) ni accessible depuis le
> fichier d'entrée. Il n'est donc pas décrit ici.

---

## 2. Le comportement du matériau — `ConstitutiveModel`

C'est la physique du matériau : à partir de l'incrément de déformation d'un point
matériel, on en déduit la contrainte.

```mermaid
classDiagram
  direction TB
  class ConstitutiveModel {
    <<abstrait>>
    donne la contrainte à partir de la déformation
  }
  ConstitutiveModel <|-- HookeElasticity
  ConstitutiveModel <|-- KelvinVoigt
  ConstitutiveModel <|-- VonMisesElastoPlasticity
  ConstitutiveModel <|-- MohrCoulomb
  ConstitutiveModel <|-- SinfoniettaClassica
  ConstitutiveModel <|-- SinfoniettaCrush
  ConstitutiveModel <|-- CHCL_DEM

  class HookeElasticity {
    élasticité linéaire
  }
  class KelvinVoigt {
    élasticité + viscosité
  }
  class VonMisesElastoPlasticity {
    plasticité des métaux
  }
  class MohrCoulomb {
    plasticité frottante des sols
  }
  class SinfoniettaClassica {
    loi avancée pour les sols
  }
  class SinfoniettaCrush {
    idem + écrasement des grains
  }
  class CHCL_DEM {
    aucune loi analytique
    un échantillon DEM 3D répond à sa place
  }
```

Le cas `CHCL_DEM` (*Computationally Homogenised Constitutive Law*) est la
spécificité du couplage MPM×DEM : le point matériel ne suit aucune loi écrite à
l'avance, il embarque un échantillon granulaire périodique 3D (`PBC3Dbox`) qu'on
soumet à la déformation imposée et dont on relit la contrainte moyenne.

---

## 3. Les frontières — `Obstacle` et `BoundaryForceLaw`

Deux questions séparées : *quelle est la forme de l'obstacle ?* et *que se
passe-t-il quand un point matériel le touche ?*

```mermaid
classDiagram
  direction LR
  class Obstacle {
    <<abstrait>>
    forme et mouvement de la frontière
    dit si un point matériel le touche
  }
  Obstacle <|-- Line
  Obstacle <|-- Circle
  Obstacle <|-- Polygon

  class BoundaryForceLaw {
    <<abstrait>>
    calcule la force de contact
  }
  BoundaryForceLaw <|-- frictionalViscoElastic
  BoundaryForceLaw <|-- frictionalViscoElastofragile
  BoundaryForceLaw <|-- frictionalNormalRestitution

  Obstacle *-- BoundaryForceLaw : possède une

  class Line {
    paroi droite
  }
  class Circle {
    disque
  }
  class Polygon {
    contour quelconque
  }
  class frictionalViscoElastic {
    ressort + amortisseur + frottement
  }
  class frictionalViscoElastofragile {
    idem + rupture de la cohésion
  }
  class frictionalNormalRestitution {
    frottement + restitution au choc
  }
```

Un obstacle peut être piloté (vitesse imposée) ou libre : dans ce dernier cas il
répond aux forces de contact avec sa masse et son inertie.

---

## 4. Le lien points ↔ grille — `ShapeFunction`

Les points matériels portent la matière, mais les dérivées spatiales se calculent
sur la grille. Les fonctions de forme font l'aller-retour entre les deux.

```mermaid
classDiagram
  direction TB
  class ShapeFunction {
    <<abstrait>>
    poids d'interpolation entre un point et les nœuds
  }
  ShapeFunction <|-- Linear
  ShapeFunction <|-- RegularQuadLinear
  ShapeFunction <|-- BSpline

  class Linear {
    interpolation linéaire sur 4 nœuds
  }
  class RegularQuadLinear {
    même chose, optimisée pour grille régulière
  }
  class BSpline {
    support étendu à 16 nœuds
    plus régulier, moins de bruit numérique
  }
```

C'est ce choix qui gouverne la qualité du transfert et une bonne part du bruit
observé quand un point traverse une frontière d'élément.

---

## 5. La marche en temps — `OneStep`

Un pas de temps MPM suit toujours le même cycle ; les variantes diffèrent par
l'ordre dans lequel la contrainte est actualisée.

```mermaid
classDiagram
  direction TB
  class OneStep {
    <<abstrait>>
    fait avancer la simulation d'un pas de temps
  }
  OneStep <|-- UpdateStressFirst
  OneStep <|-- UpdateStressLast
  OneStep <|-- ModifiedLagrangian

  class UpdateStressFirst {
    USF
    contrainte actualisée avant le mouvement
  }
  class UpdateStressLast {
    USL
    contrainte actualisée après le mouvement
  }
  class ModifiedLagrangian {
    MUSL, variante lissée
    seul schéma compatible avec le couplage MPM x DEM
  }
```

Le cycle réalisé à chaque pas :

```mermaid
flowchart LR
  A["1. points → grille<br/>masse et quantité de mouvement"] --> B["2. forces sur la grille<br/>internes (contraintes),<br/>externes (gravité, contacts)"]
  B --> C["3. résolution sur la grille<br/>accélérations, vitesses"]
  C --> D["4. grille → points<br/>vitesses et positions"]
  D --> E["5. déformation puis contrainte<br/>via le ConstitutiveModel"]
  E --> F["6. on oublie la grille<br/>elle est remise à zéro"]
  F --> A
```

La grille est réinitialisée à chaque pas : c'est ce qui permet aux grandes
déformations de ne jamais distordre le maillage.

---

## 6. La mise en données et le scénario — `Command` et `Scheduler`

`Command` agit **une fois**, au moment où le fichier d'entrée est lu.
`Scheduler` est **relu à chaque pas** et agit quand sa condition est remplie.

```mermaid
classDiagram
  direction LR
  class Command {
    <<abstrait>>
    exécutée une seule fois, à la lecture
  }
  Command <|-- set_MP_grid
  Command <|-- set_MP_polygon
  Command <|-- add_MP_ShallowPath
  Command <|-- new_set_grid
  Command <|-- set_node_grid
  Command <|-- set_BC_line
  Command <|-- set_BC_column
  Command <|-- set_K0_stress
  Command <|-- set_uniform_pressure
  Command <|-- move_MP
  Command <|-- reset_model
  Command <|-- select_tracked_MP
  Command <|-- select_controlled_MP
```

| Rôle | Commandes |
|---|---|
| Créer la matière | `set_MP_grid`, `set_MP_polygon`, `add_MP_ShallowPath` |
| Définir la grille | `new_set_grid`, `set_node_grid` |
| Conditions aux limites | `set_BC_line`, `set_BC_column` |
| État initial | `set_K0_stress`, `set_uniform_pressure` |
| Retouches | `move_MP`, `reset_model` |
| Marquer des points | `select_tracked_MP`, `select_controlled_MP` |

```mermaid
classDiagram
  direction LR
  class Scheduler {
    <<abstrait>>
    consultée à chaque pas de temps
  }
  Scheduler <|-- GravityRamp
  Scheduler <|-- MoveObstacle
  Scheduler <|-- RemoveObstacle
  Scheduler <|-- RemoveMaterialPoint
  Scheduler <|-- PICDissipation
  Scheduler <|-- PICDissipationByPIC
  Scheduler <|-- ReactivateCHCLBonds

  class GravityRamp {
    gravité montée progressivement
  }
  class MoveObstacle {
    déclenche le mouvement d'un obstacle
  }
  class RemoveObstacle {
    retire une paroi, provoque un lâcher
  }
  class RemoveMaterialPoint {
    évacue des points matériels
  }
  class PICDissipation {
    dosage de l'amortissement numérique
  }
  class PICDissipationByPIC {
    idem, avec un autre critère
  }
  class ReactivateCHCLBonds {
    recolle les liens dans les échantillons DEM
  }
```

C'est avec les `Scheduler` que l'on écrit un scénario : « laisser consolider,
puis retirer la paroi, puis couper l'amortissement ».

---

## 7. L'observation — `Spy`

Les sondes produisent des **mesures scalaires au cours du temps**, dans des
fichiers texte. Les champs sur toute la scène ne passent pas par une sonde : ils
sont écrits dans les *conf-files* sauvés tous les `confPeriod` pas.

```mermaid
classDiagram
  direction LR
  class Spy {
    <<abstrait>>
    mesure, enregistre, clôture
  }
  Spy <|-- MeanStress
  Spy <|-- Work
  Spy <|-- EnergyBalance
  Spy <|-- MPTracking
  Spy <|-- ObstacleTracking
  Spy <|-- ElasticBeamDev

  class MeanStress {
    contrainte moyenne
  }
  class Work {
    travail des efforts
  }
  class EnergyBalance {
    bilan d'énergie
  }
  class MPTracking {
    suivi de points matériels choisis
  }
  class ObstacleTracking {
    efforts et mouvement d'une paroi
  }
  class ElasticBeamDev {
    flèche d'une poutre, cas test
  }
```

---

## 8. Ce qu'il faut retenir

1. **Un seul cœur, beaucoup d'ingrédients.** Le moteur (`MPMbox`) ne connaît que
   des *contrats* : « un modèle sait rendre une contrainte », « un obstacle sait
   dire si on le touche ». Il ignore quelle variante est réellement utilisée.
2. **Le fichier d'entrée choisit les ingrédients par leur nom.** `MohrCoulomb`,
   `BSpline`, `Circle` … sont des mots-clés qui désignent directement une classe.
3. **Ajouter une physique n'oblige pas à toucher au moteur** : on écrit une
   nouvelle variante, on l'enregistre sous un nom, elle devient disponible.
4. **Le couplage MPM×DEM entre par la même porte** : `CHCL_DEM` n'est qu'une
   variante de plus dans la famille des lois de comportement.
