# MPMbox (mpm/2D) — bugs et bugs potentiels

Revue de l'intégralité des sources de `mpm/2D` (hors `deps/`, hors `BUILD/`) : `Core/`,
`OneStep/`, `ConstitutiveModels/`, `ShapeFunctions/`, `Obstacles/`, `BoundaryForceLaw/`,
`Commands/`, `Spies/`, `Schedulers/`, `Runners/`, `See/`.

Le compilateur ne signale rien : `g++-16 -O2 -Wall -Wextra -Wshadow -Wnull-dereference` sur
l'ensemble des fichiers ne produit aucun avertissement. Tout ce qui suit a été trouvé par
lecture. Les défauts déjà consignés dans l'annexe B du manuel utilisateur sont rappelés en
fin de document (§ D12) mais ne sont pas répétés dans le corps.

## Comment lire ce document

| Priorité | Signification |
|---|---|
| **A — critique** | Comportement indéfini : lecture hors bornes, pointeur nul, division entière par zéro. Peut planter, ou pire, ne pas planter. |
| **B — majeur** | Le calcul tourne et produit un résultat faux, sans aucun message. |
| **C — moyen** | Robustesse, reprise (restart), fuites mémoire, cas limites. |
| **D — mineur** | Incohérences, code mort, messages trompeurs. |

Chaque entrée porte un identifiant stable (`A1`, `B7`, …) pour pouvoir désigner
précisément ce qui doit être corrigé.

---

## Journal des corrections

### 2026-08-05 — A1, localisation de l'élément

Voir le détail dans l'entrée **A1**. Le calcul de l'indice d'élément a été factorisé dans
`ShapeFunction::locateElement`, seul endroit où il est désormais vérifié.

### 2026-08-05 — A4, A5, A6, A9, A10, C12, C14 : validation des entrées

Une passe unique sur tout ce qui, dans le fichier d'entrée, pouvait provoquer une lecture
hors bornes ou un pointeur nul. **Aucune de ces corrections ne change le résultat d'un
calcul valide** : elles transforment des corruptions silencieuses en messages explicites.

| Défaut | Ce qui a été fait |
|---|---|
| **A4** | Les cinq recherches de modèle (`Core/MPMbox.cpp`, `set_MP_grid`, `set_MP_polygon`, `reset_model`, `add_MP_ShallowPath`) s'arrêtent au lieu de déréférencer `models.end()`. |
| **A5** | Nouvelle méthode `MPMbox::checkSettings()`, appelée par `run()` : refuse de démarrer si un couple (groupe MP, groupe obstacle) réellement présent sort de la table, et avertit si `kn` ou `mu` n'ont jamais été donnés pour ce couple. |
| **A6** | Le nom de la loi de contact est vérifié. Chaque obstacle reçoit **sa propre** instance au lieu d'en partager une, et la loi par défaut posée par le constructeur d'`Obstacle` est libérée : l'appartenance devient sans ambiguïté. Un avertissement signale une loi assignée à un groupe sans obstacle. Même contrôle ajouté sur `oneStepType` et `ShapeFunction`. |
| **A9** | `set_BC_line` et `set_BC_column` vérifient leurs indices et s'arrêtent au lieu de se contenter d'un avertissement quand la grille n'existe pas encore. |
| **A10** | `checkSettings()` refuse `confPeriod`, `proxPeriod`, `Spy::nstep` ou `Spy::nrec` nuls ou négatifs. Les valeurs par défaut de `Spy` passent de 0 à 1. |
| **C12** | `set <nom>` refuse un nom de paramètre inconnu au lieu d'en créer un nouveau en silence. |
| **C14** | La section `Nodes` s'arrête si la grille n'est pas encore construite, et vérifie chaque numéro de nœud. La section `Elem` vérifie le nombre de nœuds par élément. |
| **D6** | Les quatre `exit(0)` sur des chemins d'erreur (`set_MP_grid`, `set_node_grid`, `new_set_grid`) deviennent `exit(EXIT_FAILURE)` : un script d'enchaînement voyait un succès. Message du rapport de taille MP/maille corrigé (il était inversé), et contrôle de validité de la grille ajouté aux deux commandes. |

`checkSettings()` est appelée depuis `run()` et non depuis `read()`, pour que le visualiseur
— qui lit des conf-files mais ne lance rien — ne soit jamais arrêté par ces contrôles.

**Vérification** : les 21 tests de `Tests/` passent (16 PASS, 0 FAIL, 7 XFAIL inchangés à la
valeur numérique près) et les sept exemples de `Examples/` démarrent et tournent, y compris
le cas double échelle `SmallOedo-MPMxDEM`.

Un avertissement inattendu est apparu sur `Examples/BoulderImpact_NRJ/input.txt` :

```
[Warn] 'BoundaryForceLaw frictionalViscoElastic 1': no obstacle belongs to group 1
```

Ce n'est pas un faux positif. Tous les obstacles de cet exemple sont déclarés en **groupe 0**
(lignes 25 et 28 à 30) et ses paramètres d'interaction portent sur le couple (0, 0), mais la
loi de contact est assignée au **groupe 1**. L'assignation n'a donc jamais eu d'effet : le
cas tourne depuis toujours avec `frictionalNormalRestitution`, la loi posée par défaut, au
lieu de `frictionalViscoElastic`. À corriger dans le fichier d'exemple.

### 2026-08-05 — B9 et B10 : historique de contact

Les trois obstacles reconstruisaient leur liste de voisins avec une comparaison inversée
(**B9**), et `Circle` et `Polygon` ne restauraient que `fn` et `ft` là où `Line` recopiait
tout le `Neighbor` (**B10**). Le code fautif étant en trois exemplaires — c'est bien pour
cela que `Line` avait été corrigé et pas les deux autres — la fusion a été factorisée dans
`Obstacle::restoreNeighborHistory`.

**Preuve que c'est bien corrigé** : la dérive d'un bloc posé sur une pente à 18° avec
μ = 1,0, mesurée pour quatre valeurs de `proxPeriod`.

| `proxPeriod` | avant | après |
|---|---|---|
| 5 | — | −0,00807009 m/s |
| 10 | −0,0099 m/s | −0,00807009 m/s |
| 100 | −0,0135 m/s | −0,00807009 m/s |
| 1000 | −0,00021 m/s | −0,00807009 m/s |

La fréquence de reconstruction des listes de voisins est un réglage purement numérique :
elle ne doit rien changer. Les quatre valeurs coïncident désormais **à la sixième
décimale**, contre un facteur 47 auparavant.

Le résidu de 8 mm/s n'est pas du fluage mais la fin du transitoire de mise en place : le
bloc est lâché au-dessus de la pente. Poussé jusqu'à t = 2 s, il tombe à 0,4 mm/s en
décroissant, avec un rapport Δy/Δx ≈ 1,2 très supérieur à la pente (0,33) — c'est du
tassement du contact pénalisé, pas du glissement. Le test `T16` a été refait en
conséquence : il mesure sur le dernier tiers d'un calcul de 1,5 s, et son assertion
principale est l'indépendance à `proxPeriod`.

**Cette correction change des résultats existants.** Mesuré en compilant les deux versions :

| Cas | Écart max sur les positions | Sur les contraintes | Centre de masse |
|---|---|---|---|
| `Examples/helloWorld` (3 `Line`, `frictionalViscoElastic`) | 3,5 × 10⁻⁸ m | 141 Pa | inchangé à 10⁻⁶ m |
| Bloc sur un `Circle` avec `frictionalNormalRestitution` | 8,7 × 10⁻³ m | 1,35 × 10⁵ Pa | remonté de 8,6 mm |

L'écart est négligeable avec des obstacles `Line`, qui ne souffraient que de **B9**
(`Line` recopiait déjà tout le `Neighbor`). Il est important avec un `Circle`, où **B10**
s'ajoutait : `dn` remis à zéro à chaque reconstruction faisait prendre à
`frictionalNormalRestitution` la mauvaise branche charge/décharge. **Tout cas utilisant un
obstacle `Circle` doit être rejoué.**

### 2026-08-05 — B3, B5, B6

**B6** (`GravityRamp`) et **B3** (`prev_pos`) sont deux corrections d'une ligne, sans effet
de bord.

- **B6** : le terme constant manquait dans l'interpolation. Une garde
  `rampStop > rampStart` a été ajoutée à la lecture.
- **B3** : `MP[p].prev_pos = MP[p].pos;` a été ajouté dans `ModifiedLagrangian`, exactement
  là où `UpdateStressFirst` et `UpdateStressLast` le font déjà. Vérification par le test
  `T11` : le travail du poids cumulé passe d'un facteur 1154 à la valeur attendue à 0,1 %
  près — l'écart résiduel est le pas d'avance du spy sur la sauvegarde, pas une erreur.

**B5 mérite une lecture attentive : c'est la seule correction qui ralentit des calculs
existants.**

Le code prenait `std::max` des trois critères là où son propre commentaire disait
« the smallest », et calculait `sqrt(massMin / knMax)` avec `knMax` resté à `-DBL_MAX`
lorsqu'il n'y a aucun obstacle. La correction retient le plus petit des critères
effectivement calculables, chacun étant omis plutôt que remplacé par un repli quand la
donnée manque.

Conséquence mesurée sur les exemples, portés à `finalTime 0.3` :

| Exemple | `dt` demandé | `dt` retenu | Facteur | Critère limitant |
|---|---|---|---|---|
| `helloWorld` | 10⁻⁵ | 9,27 × 10⁻⁷ | **× 10,8** | CFL |
| `collapse` | 10⁻⁵ | 9,27 × 10⁻⁷ | **× 10,8** | CFL |
| `CantileverBeam` | 10⁻⁵ | 2,32 × 10⁻⁶ | **× 4,3** | CFL |
| `helloSlumpTest` | 10⁻⁵ | 10⁻⁵ | — | aucun |
| `debug` | 10⁻⁵ | 10⁻⁵ | — | aucun |

Le facteur est le rapport de coût : `helloWorld` passe de 10 s à 98 s pour la même durée
physique.

Ces trois cas tournaient donc **au-dessus de la limite CFL**. Pour `helloWorld`,
$E = 10^9$ Pa, $\nu = 0{,}42$, $\rho = 2700$ kg/m³ donnent une célérité de 1521 m/s ; avec un
rayon équivalent de point de 2,82 mm, $dt_\text{CFL} = 1{,}85 \times 10^{-6}$ s. Le `dt` de
$10^{-5}$ était 10,8 fois trop grand. Le calcul se poursuivait quand même, l'amortissement
PIC étant tolérant, mais rien ne le garantissait.

Deux points à connaître avant d'adopter :

1. **Le critère CFL est ici bâti sur le rayon du point matériel** (2,82 mm), et non sur le
   pas de la grille (10 mm) comme le veut la formulation habituelle. Cela le rend environ
   3,5 fois plus sévère. C'est un choix antérieur, qui n'a pas été touché — mais même avec
   la formulation classique, le `dt` de `helloWorld` reste 3 fois trop grand.
2. **Le solveur écrase le `dt` de l'utilisateur** au lieu de simplement l'avertir. C'est
   également le comportement d'origine. Si on préfère garder la main, il suffit de
   remplacer l'affectation par un avertissement — la décision revient à l'utilisateur.

Une marge relative de 1 % a été ajoutée à la condition de déclenchement. Sans elle,
`dt` venant d'être fixé à `0,5 · criticalDt`, la moindre variation de `criticalDt` dans ses
derniers chiffres relançait l'ajustement : `helloWorld` produisait **325 027 messages**
d'ajustement pour une variation totale de 1 %. Il en produit un seul.

### 2026-08-05 — B1, B2, B11

**B1** — `shearLimit` passe de `0.0` à `-1.0`, valeur qui désactive le mécanisme, et la
condition devient `shearLimit > 0.0 && (…)`. Les trois paramètres de découpage
(`splitCriterionValue`, `MaxSplitNumber`, `shearLimit`) sont désormais écrits dans les
conf-files : sans cela, une reprise repartait sur les valeurs par défaut, et le changement
de défaut aurait rendu la reprise incohérente avec le calcul d'origine. Ils étaient déjà
reconnus par le lecteur, l'ajout est donc rétrocompatible. C'est une part de **C3**.

**B2** — la remise à zéro de `velGrad` a été déplacée dans
`MPMbox::updateVelocityGradient`, en tête de la boucle qui accumule. Aucun schéma
d'intégration ne peut plus l'oublier ; le `reset` redondant de `ModifiedLagrangian` a été
retiré et les commentaires trompeurs de `UpdateStressFirst`/`UpdateStressLast` corrigés.

Effet mesuré sur une colonne élastique, écart maximal de $\det F$ à 1 :

| Schéma | à $t = 0{,}02$ s (avant) | à $t = 0{,}4$ s (après) |
|---|---|---|
| `ModifiedLagrangian` | — | 2,3 × 10⁻⁴ |
| `UpdateStressFirst` | divergeait | 4,3 × 10⁻⁵ |
| `UpdateStressLast` | 0,187 puis explosion à $t = 0{,}03$ s | **diverge encore, voir B15** |

**B11 — mon analyse initiale était fausse sur un point, et il faut le dire.**

J'avais écrit que le découpage « oublie de diviser `vol0` et `size` ». C'est **faux**, et
la correction correspondante aurait cassé la géométrie. Le point matériel est un
parallélogramme de sommets $\mathbf{x} \pm F\,(\pm s/2, \pm s/2)$, donc d'aire
$|\det F|\,s^2$. Au découpage, `F.xx` et `F.yx` sont divisés par deux : $\det F$ est
divisé par deux, et l'aire de chaque moitié vaut bien la moitié de l'aire d'origine.
`vol0` est l'empreinte de **référence** et doit rester intacte — c'est ce qui fait que
`vol = det(F) * vol0` reste vrai dans `UpdateStressFirst`/`Last`, et que
`size = sqrt(vol0)` reste correct à la relecture d'un conf-file. Il n'y a donc **ni
défaut de reprise ni non-conservation** de ce côté.

La grandeur conservée est la somme des `vol`, pas la somme des `vol0` — cette dernière
n'a aucune raison de l'être. Le test `T14`, bâti sur la mauvaise prémisse, a été réécrit.

Restent trois défauts, bien réels, tous corrigés :

1. **`MP2.nb` héritait du numéro du point d'origine**, donc des identifiants dupliqués. Le
   prochain numéro libre est calculé en tête de fonction.
2. **La borne de la boucle était relue à chaque tour** : un point tout juste créé était
   ré-examiné dans la même passe et redécoupé aussitôt, jusqu'à `MaxSplitNumber`. Le
   résultat dépendait de l'ordre de rangement des points. La borne est maintenant figée
   avant la boucle, et un point créé attend l'appel suivant.
3. **`MP2.PBC` était un pointeur copié** : en double échelle, les deux moitiés partageaient
   la même cellule DEM, déformée deux fois par pas et par deux threads à la fois dans la
   boucle OpenMP de `ModifiedLagrangian`. Les points double échelle ne sont plus découpés,
   avec un avertissement émis une seule fois.

### 2026-08-05 — B7 et B15

**B7 — `KelvinVoigt`.** La contrainte visqueuse est instantanée : elle vaut $\eta\,\dot\varepsilon$
et n'a pas à s'accumuler. Le modèle applique maintenant

$$\sigma_n = \bigl(\sigma_{n-1} - \eta\,\dot\varepsilon_{n-1}\bigr) + C{:}\mathrm{d}\varepsilon_n + \eta\,\dot\varepsilon_n$$

c'est-à-dire qu'il retire la contribution visqueuse du pas précédent avant d'ajouter celle
du pas courant. Un champ `MaterialPoint::viscousStress` (plus sa composante hors plan) la
mémorise d'un pas sur l'autre.

Effet mesuré sur une colonne, contrainte $\sigma_{yy}$ moyenne à $t = 0{,}02$ s :

| | avant | après |
|---|---|---|
| $dt = 10^{-5}$ | −683,5 Pa | −1025,8 Pa |
| $dt = 5 \times 10^{-6}$ | −1004,1 Pa | −1025,2 Pa |
| écart | **47 %** | **0,06 %** |

Le modèle se comporte enfin comme un Kelvin-Voigt : le résultat ne dépend plus du pas de
temps. **Les résultats de tout calcul utilisant `KelvinVoigt` changent**, et d'autant plus
que $\eta$ est grand.

Réserve à connaître : `viscousStress` **n'est pas encore écrit dans les conf-files**. Après
une reprise, la contribution visqueuse du dernier pas avant sauvegarde n'est pas retirée et
reste comme un décalage constant, d'amplitude $\eta\,\dot\varepsilon$ au moment de la
sauvegarde. C'est négligeable dès que le calcul est proche de l'équilibre, mais il faut
ajouter ce champ au chantier **C3**.

**B15 — `UpdateStressLast`.** Les vitesses nodales sont recalculées après la mise à jour
des quantités de mouvement, et le calcul du gradient de transformation a été descendu après
ce rafraîchissement. Le diagnostic est confirmé : le schéma passe de « éjecte un point hors
de la grille » à un comportement meilleur que celui de `ModifiedLagrangian`.

Écart maximal de $\det F$ à 1 sur une colonne élastique à $t = 0{,}4$ s :

| Schéma | avant B2 | après B2 | après B15 |
|---|---|---|---|
| `ModifiedLagrangian` | 2,3 × 10⁻⁴ | 2,3 × 10⁻⁴ | 2,3 × 10⁻⁴ |
| `UpdateStressFirst` | divergeait | 4,3 × 10⁻⁵ | 4,3 × 10⁻⁵ |
| `UpdateStressLast` | divergeait | éjectait un point | **7,2 × 10⁻⁵** |

Les deux schémas portant l'entête « NOT anymore used … AVOID TO USE IT » sont donc
maintenant utilisables. La question de les maintenir ou de les retirer de la fabrique
reste ouverte, mais elle ne se pose plus dans les mêmes termes.

---

## ~~B15~~ — `UpdateStressLast` diverge : la vitesse nodale n'est pas rafraîchie

> **CORRIGÉ le 2026-08-05**, le jour même où il a été découvert — voir le journal en tête
> du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichier** : `OneStep/UpdateStressLast.cpp:98-103` et `:164-166`

Défaut découvert en vérifiant **B2**. Une fois `velGrad` correctement remis à zéro,
`ModifiedLagrangian` et `UpdateStressFirst` se comportent bien sur une colonne élastique
($\det F$ à $4 \times 10^{-5}$ de 1 après 0,4 s), mais `UpdateStressLast` diverge encore et
finit par éjecter un point de la grille.

L'ordre des opérations en est la cause probable :

```cpp
// ligne 98 : vitesses nodales, AVANT la mise à jour des quantités de mouvement
nodes[n].vel = nodes[n].q / nodes[n].mass;
...
// ligne 146 : mise à jour des quantités de mouvement
nodes[n].q += nodes[n].qdot * dt;
...
// ligne 165 : mise à jour des contraintes, qui relit nodes[].vel — jamais rafraîchie
MP[p].constitutiveModel->updateStrainAndStress(MPM, p);
```

Tout l'intérêt d'un schéma « update stress last » est de bâtir l'incrément de déformation
sur le champ de vitesse de **fin** de pas. Ici il utilise celui du début de pas, tout en
appliquant les forces internes du pas précédent : c'est une combinaison connue pour être
instable. `ModifiedLagrangian` fait bien le rafraîchissement (ligne 189, « Calculate
updated velocity in nodes to compute deformation ») avant sa mise à jour des contraintes ;
`UpdateStressFirst` n'en a pas besoin, puisqu'il calcule les contraintes avant les forces.

**Correction proposée** — recalculer les vitesses nodales après la mise à jour des
quantités de mouvement, juste avant la mise à jour des contraintes :

```cpp
// 4bis) ==== Vitesses nodales de fin de pas, pour l'incrément de déformation
for (size_t n = 0; n < liveNodeNum.size(); n++) {
  if (nodes[liveNodeNum[n]].mass > tolmass) {
    nodes[liveNodeNum[n]].vel = nodes[liveNodeNum[n]].q / nodes[liveNodeNum[n]].mass;
  } else {
    nodes[liveNodeNum[n]].vel.reset();
  }
}

// 5) ==== Update strain and stress
```

Noter que `MPM.updateTransformationGradient()` (ligne 118) lit lui aussi `nodes[].vel` et
souffre du même décalage : il faudrait probablement le déplacer après ce rafraîchissement.

Ce diagnostic n'a **pas été vérifié** : `UpdateStressLast` porte l'entête « NOT anymore
used … AVOID TO USE IT », et la correction demande de valider un schéma d'intégration
complet. Le test `T17` marque le défaut. La question préalable est de savoir si ce schéma
doit être maintenu ou retiré de la fabrique.

### 2026-08-05 — Mise à niveau des schémas d'intégration (et **B4**)

`ModifiedLagrangian` avait accumulé des fonctionnalités que les deux autres schémas, plus
anciens, n'avaient jamais reçues. Le code étant en trois exemplaires, ces pièces ont été
factorisées dans `OneStep` — `updateMPVelocity`, `updateStrainAndStress`,
`updateDensityFromVolume` — plutôt que recopiées deux fois de plus.

État avant :

| Fonctionnalité | MUSL | USF | USL |
|---|---|---|---|
| Mélange FLIP/PIC (`enablePIC`, schedulers `PICDissipation*`) | ✓ | **✗** | **✗** |
| Répartition double échelle en deux boucles OpenMP | ✓ | série | série |
| Mise à jour de la masse volumique | ✓ | **✗** | **✗** |
| Rafraîchissement des coins (`updateCornersFromF`) | **✗** (B4) | ✓ | ✓ |
| Chronomètres du profileur | ✓ | ✗ | ✗ |

**FLIP/PIC.** C'est le manque le plus lourd : `enablePIC`, `disablePIC` et les deux
schedulers de dissipation étaient **silencieusement sans effet** avec `UpdateStressFirst` et
`UpdateStressLast`. Mesuré sur une colonne élastique, énergie cinétique résiduelle à
$t = 0{,}1$ s :

| Schéma | sans PIC | avec `enablePIC 0.9` | facteur |
|---|---|---|---|
| `UpdateStressFirst` **avant** | 2,01 × 10⁻⁶ | 2,01 × 10⁻⁶ | **1** |
| `UpdateStressFirst` après | 2,01 × 10⁻⁶ | 8,1 × 10⁻¹⁷ | 2,5 × 10¹⁰ |
| `ModifiedLagrangian` | 6,1 × 10⁻⁵ | 8,9 × 10⁻¹⁷ | 6,9 × 10¹¹ |
| `UpdateStressLast` | 8,8 × 10⁻⁷ | 3,2 × 10⁻²⁹ | — |

Un facteur exactement 1 : le mot-clé était lu, rangé, et jamais consulté.

**Masse volumique.** `UpdateStressFirst` et `UpdateStressLast` mettaient le volume à jour
sans toucher à `density`, qui restait à sa valeur initiale pour toujours. La masse
reconstruite depuis un conf-file dérivait donc : 6,3 × 10⁻⁵ en 10 000 pas, contre zéro
maintenant. `density` est ce que le visualiseur affiche et ce qui alimente `rhoMin` dans
`convergenceConditions`. Les trois schémas la dérivent désormais de `mass / vol`, ce qui
garde `mass = vol × density` exact quelle que soit la durée du calcul — la forme
multiplicative qu'employait `ModifiedLagrangian` dérivait lentement.

**Double échelle.** À corriger une idée reçue, y compris la mienne : les modèles CHCL
*fonctionnaient* déjà avec les deux autres schémas, la boucle sur les modèles de
comportement étant générique. Ce qui manquait, c'est la répartition en deux listes —
points simples d'un côté, cellules DEM de l'autre — et sa parallélisation OpenMP : une
cellule DEM coûte des ordres de grandeur de plus qu'une loi analytique, les mélanger dans
une seule boucle laisse la plupart des fils à attendre. Vérifié sur
`Examples/SmallOedo-MPMxDEM` : `UpdateStressFirst` mène le calcul à son terme, 16 points
double échelle, aucune valeur non finie.

Une réserve subsiste pour `UpdateStressLast` : il calcule le gradient de transformation en
fin de pas, donc le limiteur de pas de temps DEM (`CHCL.limitTimeStepFactor`) ne contraint
que le pas **suivant**. Un avertissement est émis au premier pas.

**B4** — `ModifiedLagrangian` était le seul à ne pas rafraîchir `MaterialPoint::corner[]`,
sur lequel `Polygon::getContactFrame` construit son repère de contact. L'appel a été ajouté
en fin de pas, ce qui complète le tableau dans les deux sens.

Les entêtes « NOT anymore used … AVOID TO USE IT » des deux fichiers ont été remplacés par
une description de ce que chaque schéma fait et de ses réserves. Vérification : les 26 tests
passent, dont le nouveau `T18` qui contrôle les deux propriétés sur les trois schémas.

---

## Récapitulatif

| ID | Défaut | Fichier principal |
|---|---|---|
| ~~A1~~ | ~~Indice d'élément jamais borné + garde de sécurité morte (`&&` au lieu de `||`)~~ — **corrigé le 2026-08-05** | `ShapeFunctions/ShapeFunction.cpp` |
| ~~A2~~ | ~~`BSpline` : la branche d'erreur n'empile rien → lecture hors bornes de `Phi`~~ — **corrigé le 2026-08-05** | `ShapeFunctions/BSpline.cpp:43` |
| ~~**A3**~~ | ~~Éléments de bord à 16 nœuds : `I[4..15]` restent à 0~~ **corrigé** | `Core/MPMbox.cpp` (`buildGrid`) |
| ~~A4~~ | ~~Déréférencement de `models.end()` (5 occurrences)~~ — **corrigé le 2026-08-05** | `Core/MPMbox.cpp:529` |
| ~~A5~~ | ~~`DataTable::get` hors bornes dès qu'un groupe n'a pas de `set`~~ — **corrigé le 2026-08-05** | `BoundaryForceLaw/*.cpp` |
| ~~A6~~ | ~~`BoundaryForceLaw` inconnu → pointeur nul déréférencé à chaque pas~~ — **corrigé le 2026-08-05** | `Core/MPMbox.cpp:444` |
| ~~**A7**~~ | ~~`RemoveMaterialPoint` laisse des indices périmés dans les listes de voisins~~ **corrigé** | `Schedulers/RemoveMaterialPoint.cpp` |
| ~~**A8**~~ | ~~`ReactivateCHCLBonds` déréférence `PBC` sans vérifier `isDoubleScale`~~ **corrigé** | `Schedulers/ReactivateCHCLBonds.cpp` |
| ~~A9~~ | ~~`set_BC_line` / `set_BC_column` : avertissent puis continuent, aucune borne~~ — **corrigé le 2026-08-05** | `Commands/set_BC_line.cpp:8` |
| ~~A10~~ | ~~Périodes à zéro (`confPeriod`, `proxPeriod`, `nstep`, `nrec`) → `SIGFPE`~~ — **corrigé le 2026-08-05** | `Core/MPMbox.cpp:817` |
| **A11** | `cut.cpp` : accès à `corner[4]` sur un tableau de 4 — *mais `cut.cpp` n'est compilé par aucune cible* | `See/cut.cpp:63` |
| ~~B1~~ | ~~`shearLimit` vaut 0 par défaut → `F` remis à l'identité à chaque pas~~ — **corrigé le 2026-08-05** | `Core/MPMbox.cpp:1100` |
| ~~B2~~ | ~~`velGrad` jamais remis à zéro dans `UpdateStressFirst` / `UpdateStressLast`~~ — **corrigé le 2026-08-05** | `OneStep/UpdateStressFirst.cpp:49` |
| ~~B3~~ | ~~`prev_pos` jamais mis à jour par `ModifiedLagrangian`~~ — **corrigé le 2026-08-05** | `OneStep/ModifiedLagrangian.cpp` |
| ~~B4~~ | ~~`corner[]` jamais mis à jour par `ModifiedLagrangian`~~ — **corrigé le 2026-08-05**, puis rendu sans objet le 2026-08-06 (champ supprimé) | `OneStep/ModifiedLagrangian.cpp` |
| ~~B5~~ | ~~`convergenceConditions` : `std::max` au lieu de `std::min`, et `knMax` négatif sans obstacle~~ — **corrigé le 2026-08-05** | `Core/MPMbox.cpp:966` |
| ~~B6~~ | ~~`GravityRamp` : interpolation sans le terme constant~~ — **corrigé le 2026-08-05** | `Schedulers/GravityRamp.cpp:31` |
| ~~B7~~ | ~~`KelvinVoigt` : la contrainte visqueuse est cumulée au lieu d'être instantanée~~ — **corrigé le 2026-08-05** | `ConstitutiveModels/KelvinVoigt.cpp:42` |
| **B8** | `VonMises` : `plasticStrain` écrasée au lieu d'être cumulée | `ConstitutiveModels/VonMisesElastoPlasticity.cpp:93` |
| ~~B9~~ | ~~Historique de contact perdu : comparaison inversée dans la reconstruction des voisins~~ — **corrigé le 2026-08-05** | `Obstacles/Circle.cpp:82` |
| ~~B15~~ | ~~`UpdateStressLast` diverge : la vitesse nodale n'est pas rafraîchie avant la mise à jour des contraintes~~ — **corrigé le 2026-08-05** | `OneStep/UpdateStressLast.cpp:98` |
| ~~B10~~ | ~~`Circle` / `Polygon` ne restaurent que `fn` et `ft`~~ — **corrigé le 2026-08-05** | `Obstacles/Circle.cpp:90` |
| ~~B11~~ | ~~Découpage : `nb` dupliqué, borne de boucle mouvante, `PBC` partagé~~ — **corrigé le 2026-08-05** (le point sur `vol0`/`size` était **erroné**) | `Core/MPMbox.cpp:1094` |
| **B12** | Les spies ouvrent (donc vident) leurs fichiers en mode visualisation | `Spies/MeanStress.cpp:15` |
| **B13** | `add_MP_ShallowPath` : borne en x fausse et `init()` du modèle non appelée | `Commands/add_MP_ShallowPath.cpp:31` |
| **B14** | `set_MP_polygon` : un `PBC3Dbox` alloué et chargé pour chaque point rejeté | `Commands/set_MP_polygon.cpp:40` |
| **C1** | `clean()` ne libère ni les spies, ni les schedulers, ni la fonction de forme… | `Core/MPMbox.cpp:293` |
| **C2** | `Elem` non vidé (cas 16 nœuds) et `liveNodeNum` empilé sans remise à zéro | `Commands/set_node_grid.cpp:75` |
| **C3** | `save()` incomplet : écrouissage, points suivis, paramètres de splitting, état DEM | `Core/MPMbox.cpp:614` |
| **C4** | `Polygon` : `rot` écrit en radians, relu en degrés ; `Area()` fausse | `Obstacles/Polygon.cpp:45` |
| **C5** | `postProcess` divise par la masse nodale sans tolérance | `Core/MPMbox.cpp:1234` |
| **C6** | `MohrCoulomb` : `apex` divisé par `sin(phi)`, non-convergence silencieuse | `ConstitutiveModels/MohrCoulomb.cpp:91` |
| **C7** | `adaptativeRefinement` : division par une extension nulle | `Core/MPMbox.cpp:1111` |
| **C8** | `set_K0_stress` : normalisation d'une gravité nulle | `Commands/set_K0_stress.cpp:12` |
| **C9** | `PICDissipation` lit un ratio FLIP là où `enablePIC` lit un ratio PIC | `Schedulers/PICDissipation.cpp:6` |
| **C10** | `MPTracking` : sélecteur exécuté à la lecture, donc avant la création des MP | `Spies/MPTracking.cpp:27` |
| **C11** | `MaterialPoint::nb` non unique | `Commands/set_MP_grid.cpp:48` |
| ~~C12~~ | ~~`set <nom>` accepte silencieusement n'importe quel nom de paramètre~~ — **corrigé le 2026-08-05** | `Core/MPMbox.cpp:392` |
| ~~**C13**~~ | ~~Pointeurs et membres non initialisés~~ **corrigé** | 24 en-têtes |
| ~~C14~~ | ~~`Nodes` lu avant la grille : avertissement puis accès hors bornes~~ — **corrigé le 2026-08-05** | `Core/MPMbox.cpp:491` |
| **C15** | `frictionalViscoElastofragile` : seuil non homogène, `sigma_n` toujours positif | `BoundaryForceLaw/frictionalViscoElastofragile.cpp:43` |
| **D1** | Deuxième bloc `Nodes` mort dans `read()` | `Core/MPMbox.cpp:537` |
| **D2** | `planeStrain` lu, sauvegardé, jamais utilisé | `Core/MPMbox.cpp:340` |
| **D3** | `extremeShearing` : critère calculé, branche vide | `Core/MPMbox.cpp:1175` |
| **D4** | `VonMises` interpole `q/masse` là où les autres modèles utilisent `nodes[].vel` | `ConstitutiveModels/VonMisesElastoPlasticity.cpp:26` |
| **D5** | Commentaire de la matrice `De` faux | `ConstitutiveModels/MohrCoulomb.cpp:64` |
| ~~D6~~ | ~~`set_MP_grid` : message inversé et `exit(0)` sur une erreur~~ — **corrigé le 2026-08-05** | `Commands/set_MP_grid.cpp:12` |
| ~~D7~~ | ~~`move_MP` : coins traités comme des coordonnées locales~~ — **corrigé le 2026-08-06** | `Commands/move_MP.cpp:45` |
| **D8** | Rayon du MP : `sqrt(vol)` pour `Circle`, `size` pour `Line` | `Obstacles/Circle.cpp:43` |
| **D9** | `cut.cpp` : `max(norm(d1), norm(d1))`, `sprintf` | `See/cut.cpp:68` |
| **D10** | `Neighbor::dt` jamais alimenté, `contactf` écrasé | `BoundaryForceLaw/frictionalViscoElastic.cpp:60` |
| **D11** | `t += dt` : le dernier conf-file peut manquer | `Core/MPMbox.cpp:857` |
| ~~D13~~ | ~~`MaterialPoint::q` n'est utilisé nulle part~~ — **corrigé le 2026-08-06** | `Core/MaterialPoint.hpp:36` |
| **D14** | Un plantage sort avec le code de retour 0 | `Runners/run.cpp` |
| **D15** | Deuxième lecteur de `Nodes`, inatteignable | `Core/MPMbox.cpp:594` |
| **D12** | Rappel des défauts déjà documentés (annexe B du manuel) | — |

---

# A — Critique (comportement indéfini)

## ~~A1~~ — Indice d'élément jamais borné, et garde de sécurité morte

> **CORRIGÉ le 2026-08-05.** Le calcul de l'indice a été sorti des trois fonctions de forme
> et factorisé dans `ShapeFunction::locateElement` (`ShapeFunctions/ShapeFunction.cpp`),
> seul endroit où il est désormais vérifié. `MPMbox::MPinGridCheck` a été aligné sur les
> mêmes bornes (`>=` au lieu de `>` sur les côtés haut et droit) pour que l'avertissement
> préalable soit d'accord avec le contrôle qui arrête le calcul.
>
> - En calcul, un point sorti de la grille arrête le programme avec le code 1 et un message
>   donnant le numéro du point, sa position, sa vitesse, l'étendue de la grille et le pas.
> - En visualisation (`computationMode == false`), l'indice est ramené à l'élément le plus
>   proche avec un avertissement : refuser d'ouvrir un conf-file parce qu'un point est mal
>   placé n'aiderait personne.
> - Les coordonnées sont bornées avant la conversion en `size_t`, ce qui rend la conversion
>   définie même pour une position `NaN` — les comparaisons sont écrites sous forme niée
>   exprès, `!(x >= 0.0 && x < W)` étant vrai pour `NaN`.
>
> Vérifié par le test `T25`, passé de `SIGBUS` à `PASS` ; les 8 autres invariants de la
> suite sont restés au vert et les 12 autres `XFAIL` inchangés à la valeur près.
>
> Le texte ci-dessous décrit l'état d'avant correction ; il est conservé pour mémoire.

**Fichiers** : `ShapeFunctions/RegularQuadLinear.cpp:14` et `:23`, `ShapeFunctions/Linear.cpp:25`,
`ShapeFunctions/BSpline.cpp:18`

Les trois fonctions de forme calculent l'élément contenant le point sans jamais vérifier
le résultat :

```cpp
MPM.MP[p].e = (size_t)(trunc(MPM.MP[p].pos.x / MPM.Grid.lx)
                     + trunc(MPM.MP[p].pos.y / MPM.Grid.ly) * (double)MPM.Grid.Nx);
size_t* I = &(MPM.Elem[MPM.MP[p].e].I[0]);   // aucun contrôle
```

Si `pos.x < 0`, `trunc` donne une valeur négative, la conversion en `size_t` produit un
entier gigantesque et `Elem[e]` lit n'importe où en mémoire. Si le point sort par le haut
ou par la droite (y compris exactement sur `Nx*lx`, où `trunc` donne `Nx`), `e >= Elem.size()`.
C'est le mode de défaillance le plus courant d'un code MPM : un point qui s'échappe de la
grille ne provoque pas un message mais une corruption silencieuse, ou un `Segmentation fault`
loin de la cause.

`RegularQuadLinear` contient bien une garde, mais elle ne peut **jamais** être vraie :

```cpp
if (MPM.MP[p].pos.x < 0.0 && MPM.MP[p].pos.x > (double)MPM.Grid.Nx * MPM.Grid.lx &&
    MPM.MP[p].pos.y < 0.0 && MPM.MP[p].pos.y > (double)MPM.Grid.Ny * MPM.Grid.ly) {
```

`pos.x` ne peut pas être simultanément négatif et supérieur à la largeur de la grille. Il
faut des `||`. `MPMbox::MPinGridCheck()` fait le bon test mais uniquement avant le premier
pas, et se contente d'un avertissement.

**Correction proposée** — une fonction unique, appelée par les trois fonctions de forme :

```cpp
// ShapeFunction.hpp
protected:
  // Retourne false si le MP est sorti de la grille (message + MP à retirer/arrêt)
  bool locateElement(MPMbox& MPM, size_t p);
```

```cpp
bool ShapeFunction::locateElement(MPMbox& MPM, size_t p) {
  const double x = MPM.MP[p].pos.x;
  const double y = MPM.MP[p].pos.y;
  const double W = (double)MPM.Grid.Nx * MPM.Grid.lx;
  const double H = (double)MPM.Grid.Ny * MPM.Grid.ly;
  if (x < 0.0 || x >= W || y < 0.0 || y >= H) {
    Logger::critical("MP {} sorti de la grille : pos = ({}, {}), grille = [0,{}]x[0,{}]",
                     p, x, y, W, H);
    return false;
  }
  size_t i = (size_t)trunc(x / MPM.Grid.lx);
  size_t j = (size_t)trunc(y / MPM.Grid.ly);
  MPM.MP[p].e = i + j * MPM.Grid.Nx;
  return true;
}
```

Noter le `>=` sur les bords haut et droit : c'est ce qui manque aujourd'hui.
Reste à décider du comportement en cas de sortie — `exit`, ou retrait du point de la
liste. Le second est préférable pour les simulations d'écoulement, mais il faut alors
invalider les listes de voisins (voir **A7**).

---

## ~~A2~~ — `BSpline` : la branche d'erreur n'empile rien

> **CORRIGÉ le 2026-08-05**, en même temps que le passage de `BSpline` aux tableaux de pile (voir `Doc/OPTIM.md`). La branche écrit désormais des zéros et avertit une seule fois. **A3, la cause racine, a été corrigé le 2026-08-06.** Le texte ci-dessous décrit l'état d'avant correction.

**Fichier** : `ShapeFunctions/BSpline.cpp:36-59`

```cpp
if (absx >= 0.0 && absx < 1.0) {
  Phi.push_back(...);  PhiGrad.push_back(...);
} else if (absx < 2.0) {
  Phi.push_back(...);  PhiGrad.push_back(...);
} else {
  std::cerr << "... we shouldn't be landing here!" << std::endl;
  //exit(EXIT_FAILURE);       <-- désactivé
}
...
for (int i = 0; i < 16; i++) {
  int ix = 2 * i;  int iy = ix + 1;
  MPM.MP[p].N[i] = Phi[ix] * Phi[iy];     // ix va jusqu'à 30
```

Dans la troisième branche rien n'est empilé, mais `exit` est commenté. `Phi` a alors moins
de 32 éléments et la boucle suivante lit hors du `std::vector`. Ce n'est pas un cas
théorique : il se produit dès qu'un MP est dans un élément de bord (voir **A3**), où les
nœuds `I[4..15]` valent tous 0 et se trouvent donc à des dizaines de mailles du point.

**Correction proposée** — empiler des valeurs nulles pour garder la taille, et signaler
une seule fois :

```cpp
} else {
  Phi.push_back(0.0);
  PhiGrad.push_back(0.0);
  Logger::warn("@BSpline: MP {} hors du support B-spline (|x| = {})", p, absx);
}
```

Correction de fond : traiter correctement les bords (**A3**), sans quoi la partition de
l'unité n'est plus vérifiée près des bords même avec cette rustine.

---

## ~~A3~~ — Éléments de bord à 16 nœuds : `I[4..15]` restent à zéro

> **CORRIGÉ le 2026-08-06** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichiers** : `Commands/set_node_grid.cpp:91-117`, `Commands/new_set_grid.cpp:60-86`

```cpp
element E;                       // I[0..15] = 0
E.I[0] = ...; E.I[1] = ...; E.I[2] = ...; E.I[3] = ...;
if (j != 0 && j != Ny - 1 && i != 0 && i != Nx - 1) {
  E.I[4] = ...;                  // rempli uniquement à l'intérieur
  ...
}
box->Elem.push_back(E);
```

Pour toute la première et la dernière rangée d'éléments, douze indices sur seize pointent
sur le nœud 0. Le `BSpline` évalue alors ses fonctions de forme par rapport à un nœud
arbitrairement lointain : partition de l'unité fausse, et déclenchement de **A2**.

**Correction proposée** — deux options :

1. *Grille fantôme* : construire deux rangées de nœuds supplémentaires de chaque côté et
   décaler l'origine, de sorte qu'aucun élément utile ne touche le bord. C'est la solution
   classique et elle laisse `BSpline.cpp` inchangé.
2. *Repli local* : à défaut, marquer explicitement les éléments incomplets
   (`E.I[r] = SIZE_MAX`) et faire retomber `BSpline` sur les quatre nœuds internes dans ce
   cas, avec renormalisation. Plus intrusif et moins précis.

À court terme et si `BSpline` n'est pas utilisé en production, il suffit de refuser la
combinaison : `Logger::critical` si `element::nbNodes == 16` et qu'un MP se trouve dans un
élément de bord.

---

## ~~A4~~ — Déréférencement de `models.end()`

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichiers** : `Core/MPMbox.cpp:527`, `Commands/set_MP_grid.cpp:18`,
`Commands/set_MP_polygon.cpp:21`, `Commands/reset_model.cpp:11`,
`Commands/add_MP_ShallowPath.cpp:21`

Le même motif est répété cinq fois :

```cpp
auto itCM = box->models.find(modelName);
if (itCM == box->models.end()) {
  Logger::error("@set_MP_grid::exec, model {} not found", modelName);
}
ConstitutiveModel* CM = itCM->second;   // itCM == end() : comportement indéfini
```

Une simple faute de frappe sur le nom d'un modèle dans le fichier d'entrée suffit. Le
message est bien affiché, puis le programme déréférence l'itérateur de fin.

**Correction proposée** — sortir dans les cinq cas :

```cpp
auto itCM = box->models.find(modelName);
if (itCM == box->models.end()) {
  Logger::critical("@set_MP_grid::exec, le modèle '{}' n'est pas défini", modelName);
  exit(EXIT_FAILURE);
}
ConstitutiveModel* CM = itCM->second;
```

Dans `MPMbox::read` (ligne 527) le point est en cours de construction : il faut sortir de
même, ou au minimum ignorer le MP.

---

## ~~A5~~ — `DataTable::get` hors bornes dès qu'un groupe n'a pas de `set`

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichiers** : `BoundaryForceLaw/frictionalViscoElastic.cpp:18-23`,
`frictionalNormalRestitution.cpp:18-23`, `frictionalViscoElastofragile.cpp:24-29`,
`Core/MPMbox.cpp:949`

`DataTable::get` (toofus) ne fait aucun contrôle :

```cpp
double get(size_t id, size_t g1, size_t g2) const { return tables[id][g1][g2]; }
```

`ngroup` ne grandit que par les appels à `set`. Un fichier d'entrée qui crée des MP de
groupe 2 mais ne définit que `set kn 0 0 …` laisse `ngroup = 1` : `tables[id][2][0]` lit
hors du `std::vector`. De même, `g2 = Obstacles[o]->group` est un `int` : un groupe négatif
devient un `size_t` énorme.

Ce défaut est facile à déclencher — il suffit d'oublier une ligne `set` pour un couple de
groupes — et il ne se manifeste par aucun message.

**Correction proposée** — vérifier une fois pour toutes à la fin de `MPMbox::read()`, avant
`run()`, que tous les couples (groupe MP, groupe obstacle) réellement présents sont définis :

```cpp
void MPMbox::checkInteractionParameters() {
  std::set<int> gMP, gObs;
  for (auto& mp  : MP)        gMP.insert(mp.groupNb);
  for (auto* obs : Obstacles) gObs.insert(obs->group);

  const size_t ng = dataTable.get_ngroup();
  for (int g1 : gMP) {
    for (int g2 : gObs) {
      if (g1 < 0 || g2 < 0 || (size_t)g1 >= ng || (size_t)g2 >= ng) {
        Logger::critical("Aucun paramètre d'interaction pour les groupes MP {} / obstacle {}", g1, g2);
        exit(EXIT_FAILURE);
      }
      if (!dataTable.isDefined(id_kn, g1, g2)) {
        Logger::critical("'set kn {} {} ...' manquant", g1, g2);
        exit(EXIT_FAILURE);
      }
      // idem kt, mu, et en2 / viscRate selon la loi effectivement branchée
    }
  }
}
```

Cette vérification est aussi le bon endroit pour rendre `convergenceConditions()` sûre
(voir **B5**).

---

## ~~A6~~ — `BoundaryForceLaw` inconnu → pointeur nul déréférencé à chaque pas

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichier** : `Core/MPMbox.cpp:433-447`, déréférencé en `OneStep/ModifiedLagrangian.cpp:115`,
`UpdateStressFirst.cpp:113`, `UpdateStressLast.cpp:124`

```cpp
BoundaryForceLaw *bType = Factory<BoundaryForceLaw>::Instance()->Create(boundaryName);
for (size_t o = 0; o < Obstacles.size(); o++) {
  if (Obstacles[o]->group == obstacleGroup) { Obstacles[o]->boundaryForceLaw = bType; }
}
```

Aucun test sur `bType`. Un nom mal orthographié remplace la loi par défaut
(`frictionalNormalRestitution`, installée par le constructeur d'`Obstacle`) par `nullptr`,
et `Obstacles[o]->boundaryForceLaw->computeForces(MPM, o)` plante au premier pas.

**Correction proposée** :

```cpp
BoundaryForceLaw *bType = Factory<BoundaryForceLaw>::Instance()->Create(boundaryName);
if (bType == nullptr) {
  Logger::critical("BoundaryForceLaw '{}' inconnue", boundaryName);
  exit(EXIT_FAILURE);
}
bool assigned = false;
for (size_t o = 0; o < Obstacles.size(); o++) {
  if (Obstacles[o]->group == obstacleGroup) {
    delete Obstacles[o]->boundaryForceLaw;   // la loi par défaut fuit aujourd'hui (voir C1)
    Obstacles[o]->boundaryForceLaw = bType;
    assigned = true;
  }
}
if (!assigned) Logger::warn("BoundaryForceLaw {} : aucun obstacle du groupe {}", boundaryName, obstacleGroup);
```

Attention : `bType` est partagé entre tous les obstacles du groupe. Si on ajoute un `delete`
il faut soit créer une instance par obstacle, soit passer par un `std::shared_ptr`. Le plus
simple est de créer une instance par obstacle dans la boucle.

Le même contrôle manque pour `Factory<OneStep>` (ligne 339) et `Factory<ShapeFunction>`
(ligne 410) : un nom erroné y donne aussi `nullptr`, mais le repli automatique de fin de
`read()` ne s'applique pas puisque la variable n'est nulle qu'après coup — en réalité le
repli fonctionne, mais l'utilisateur ne sait pas que son mot-clé a été ignoré. Un message
suffit.

---

## ~~A7~~ — `RemoveMaterialPoint` laisse des indices périmés dans les listes de voisins

> **CORRIGÉ le 2026-08-06** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichier** : `Schedulers/RemoveMaterialPoint.cpp:13-23`

```cpp
std::vector<MaterialPoint> MP_swap;
for (size_t i = 0; i < box->MP.size(); i++) {
  if (box->MP[i].constitutiveModel->key != CMkey) MP_swap.push_back(box->MP[i]);
}
MP_swap.swap(box->MP);
```

Les `Obstacle::Neighbors` stockent des `PointNumber` qui sont des indices dans `box->MP`.
Après le compactage, ces indices désignent d'autres points, ou sortent du tableau.

L'ordre dans `MPMbox::run()` est : `checkProximity()` → `Scheduled[s]->check()` →
`advanceOneStep()`. Les points sont donc retirés **après** la reconstruction des voisins et
**avant** le pas de calcul : dès le pas courant, `computeForces` lit `MPM.MP[pn]` avec un
`pn` hors bornes. La détection `MP.size() != number_MP_before_any_split` n'intervient qu'au
pas suivant, trop tard.

Les `PBC3Dbox` des points supprimés sont par ailleurs perdus (fuite).

**Correction proposée** — invalider les listes de voisins immédiatement :

```cpp
void RemoveMaterialPoint::check() {
  if (removeTime >= box->t && removeTime <= box->t + box->dt) {
    std::vector<MaterialPoint> MP_swap;
    for (size_t i = 0; i < box->MP.size(); i++) {
      if (box->MP[i].constitutiveModel->key != CMkey) {
        MP_swap.push_back(box->MP[i]);
      } else {
        delete box->MP[i].PBC;          // sinon fuite (nullptr toléré par delete)
        box->MP[i].PBC = nullptr;
      }
    }
    MP_swap.swap(box->MP);

    for (size_t o = 0; o < box->Obstacles.size(); o++) box->Obstacles[o]->Neighbors.clear();
    box->checkProximity();              // reconstruit avec les nouveaux indices
  }
}
```

L'historique de contact est perdu au pas de suppression, ce qui est acceptable ici.
Le même raisonnement vaudra pour toute suppression de MP ajoutée par la suite (par exemple
un retrait des points sortis de la grille, cf. **A1**).

---

## ~~A8~~ — `ReactivateCHCLBonds` déréférence `PBC` sans vérification

> **CORRIGÉ le 2026-08-06** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichier** : `Schedulers/ReactivateCHCLBonds.cpp:12-17`

```cpp
for (size_t p = 0; p < box->MP.size(); p++) {
  box->MP[p].PBC->ActivateBonds(bondingDistance, bondedStateDam);
}
```

`PBC` vaut `nullptr` pour tout MP simple échelle. Une simulation mixte, ou même une
simulation purement mono-échelle où ce scheduler traîne dans le fichier, plante.

**Correction proposée** :

```cpp
for (size_t p = 0; p < box->MP.size(); p++) {
  if (box->MP[p].isDoubleScale == false || box->MP[p].PBC == nullptr) continue;
  box->MP[p].PBC->ActivateBonds(bondingDistance, bondedStateDam);
}
```

Par ailleurs `bondingDistance` et `timeBondReactivation` ne sont pas initialisés dans le
`.hpp` : leur donner `{0.0}`.

---

## ~~A9~~ — `set_BC_line` / `set_BC_column` : avertissement puis accès hors bornes

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichiers** : `Commands/set_BC_column.cpp:7-17`, `Commands/set_BC_line.cpp:7-18`

```cpp
if (box->nodes.empty()) {
  std::cerr << "@set_BC_column::exec, Cannot set BC. Grid not yet defined!" << std::endl;
}                              // ... et on continue quand même
node* N;
for (int j = line0; j <= line1; j++) {
  N = &(box->nodes[j * (box->Grid.Nx + 1) + column_num]);   // aucune borne
```

Deux problèmes cumulés : le message n'interrompt rien, et aucun indice n'est vérifié. Un
`set_BC_line 40 0 30 1 1` sur une grille de 30 lignes écrit dans la mémoire d'à côté.

**Correction proposée** :

```cpp
void set_BC_column::exec() {
  if (box->nodes.empty()) {
    Logger::critical("@set_BC_column::exec, la grille doit être définie avant (set_node_grid)");
    exit(EXIT_FAILURE);
  }
  if (column_num < 0 || (size_t)column_num > box->Grid.Nx ||
      line0 < 0 || line1 < line0 || (size_t)line1 > box->Grid.Ny) {
    Logger::critical("@set_BC_column::exec, indices hors grille "
                     "(colonne {}, lignes {}..{}, grille {}x{})",
                     column_num, line0, line1, box->Grid.Nx, box->Grid.Ny);
    exit(EXIT_FAILURE);
  }
  for (size_t j = (size_t)line0; j <= (size_t)line1; j++) {
    node& N = box->nodes[j * (box->Grid.Nx + 1) + (size_t)column_num];
    N.xfixed = Xfixed;
    N.yfixed = Yfixed;
  }
}
```

Rappel utile pour le manuel : les indices vont de 0 à `Nx` inclus (il y a `Nx+1` colonnes de
nœuds pour `Nx` éléments).

---

## ~~A10~~ — Périodes à zéro → division entière par zéro

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichier** : `Core/MPMbox.cpp:817`, `:837`, `:853-854`
**Mesuré** : sur Apple Silicon, `confPeriod 0` ne plante pas — voir la remarque
sur l'architecture ci-dessous.

```cpp
if (step % confPeriod == 0) { ... }
if (step % proxPeriod == 0 || ...) { ... }
if ((step % Spies[s]->nstep) == 0) Spies[s]->exec();
if ((step % Spies[s]->nrec)  == 0) Spies[s]->record();
```

`confPeriod 0` ou `proxPeriod 0` dans le fichier d'entrée est une division entière par
zéro. Pour les spies, `Spy::nstep` et `Spy::nrec` valent 0 par défaut
(`Spies/Spy.hpp:9-10`) : un spy dont le `read()` oublierait de les positionner tombe dans
le même cas, tout comme `Spy MeanStress 0 fichier.txt`.

**Le symptôme dépend de l'architecture**, ce qui rend le défaut d'autant plus désagréable :

- sur x86-64, l'instruction `idiv` par zéro lève `SIGFPE` : le programme meurt immédiatement ;
- sur AArch64 (Apple Silicon), `sdiv` par zéro **retourne 0 sans piéger** : `step % 0`
  vaut 0, la condition est donc toujours vraie et un conf-file est écrit **à chaque pas**.
  Vérifié sur cette machine : le test `T23` ne plante pas, il produit un conf-file par pas.

Une entrée qui fait planter un collègue sous Linux et remplit le disque sur un Mac est le
pire des deux mondes.

**Correction proposée** — valider à la fin de `read()` :

```cpp
if (confPeriod <= 0) { Logger::warn("confPeriod <= 0, remis à 1"); confPeriod = 1; }
if (proxPeriod <= 0) { Logger::warn("proxPeriod <= 0, remis à 1"); proxPeriod = 1; }
for (size_t s = 0; s < Spies.size(); s++) {
  if (Spies[s]->nstep <= 0) Spies[s]->nstep = 1;
  if (Spies[s]->nrec  <= 0) Spies[s]->nrec  = 1;
}
```

et donner à `Spy` des valeurs par défaut non nulles : `int nstep{1}; int nrec{1};`.

---

## A11 — `cut.cpp` : accès à `corner[4]`

**Fichier** : `See/cut.cpp:63`

> **Correction de priorité.** `See/cut.cpp` contient un `main()` mais **n'est compilé par
> aucune cible** de `CMakeLists.txt` : le `file(GLOB …)` ne balaie pas `See/`, et seuls
> `mpmbox` (depuis `Runners/run.cpp`) et `see` (depuis `See/see.cpp`) sont déclarés. Ce
> fichier est donc du code mort, au même titre que `VtkOutputs/` (**D12**). Les défauts
> ci-dessous sont réels mais leur priorité effective est celle de la classe D tant que
> l'outil n'est pas remis dans la compilation. Ils sont à corriger **au moment** où on le
> fera, pas avant.

```cpp
d1 = SmoothedData[i].corner[0] - SmoothedData[i].corner[2];
d2 = SmoothedData[i].corner[1] - SmoothedData[i].corner[4];   // le tableau a 4 éléments
```

`ProcessedDataMP::corner` est un `vec2r[4]` : les indices valides sont 0 à 3. Lecture hors
bornes systématique dans la branche « 2D » de `cut`.

`d2` n'est d'ailleurs jamais utilisé ensuite, puisque la ligne 68 écrit
`std::max(norm(d1), norm(d1))` — `d1` deux fois (voir **D9**).

**Correction proposée** :

```cpp
d1 = SmoothedData[i].corner[0] - SmoothedData[i].corner[2];   // diagonale 0-2
d2 = SmoothedData[i].corner[1] - SmoothedData[i].corner[3];   // diagonale 1-3
...
<< std::max(norm(d1), norm(d2)) << " "
```

`cut.cpp` ne met par ailleurs pas `computationMode` à `false` avant de lire le conf-file :
il vide donc les fichiers des spies (voir **B12**).

---

# B — Majeur (résultat faux, silencieusement)

## ~~B1~~ — `shearLimit` vaut 0 par défaut : `F` remis à l'identité à chaque pas

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichier** : `Core/MPMbox.cpp:1100-1105`, défaut en `Core/MPMbox.hpp:120`

```cpp
double shearLimit{0.0};   // max Fxy or Fyx value. After this F becomes Identity matrix
...
if (fabs(MP[p].F.xy) > shearLimit or fabs(MP[p].F.yx) > shearLimit) {
  MP[p].F.xx = 1;  MP[p].F.xy = 0;
  MP[p].F.yx = 0;  MP[p].F.yy = 1;
}
```

Avec la valeur par défaut, la condition est vraie dès que le cisaillement est non nul,
c'est-à-dire pratiquement toujours. Activer `splitting` sans préciser `shearLimit` remet
donc le gradient de transformation à l'identité à **chaque pas de temps** : les MP ne se
déforment plus, le volume ne varie plus (dans `UpdateStressFirst`/`Last` où
`vol = F.det()*vol0`), et le critère de découpage n'est jamais atteint.

**Correction proposée** — une valeur par défaut qui désactive le mécanisme, plus un test
explicite :

```cpp
// MPMbox.hpp
double shearLimit{-1.0};   // < 0 : mécanisme désactivé
```

```cpp
// MPMbox.cpp
if (shearLimit > 0.0 && (fabs(MP[p].F.xy) > shearLimit || fabs(MP[p].F.yx) > shearLimit)) {
  ...
}
```

À signaler dans le manuel : remettre `F` à l'identité est une remise à zéro de l'histoire de
déformation, pas une régularisation ; c'est à utiliser en connaissance de cause.

---

## ~~B2~~ — `velGrad` jamais remis à zéro dans `UpdateStressFirst` / `UpdateStressLast`

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichiers** : `OneStep/UpdateStressFirst.cpp:49-50`, `OneStep/UpdateStressLast.cpp:46-49`,
accumulation en `Core/MPMbox.cpp:1000-1012`

```cpp
// ==== Reset the resultant forces on MPs and velGrad
for (size_t p = 0; p < MP.size(); p++) { MP[p].f.reset(); }   // velGrad n'est pas remis à zéro
```

Le commentaire annonce la remise à zéro de `velGrad`, mais seule `f` est traitée.
`MPMbox::updateVelocityGradient()` accumule (`+=`) :

```cpp
MP[p].velGrad.xx += (MP[p].gradN[r].x * nodes[I[r]].vel.x);
```

Le gradient de vitesse est donc la somme de tous les gradients depuis le début du calcul.
`F = (I + dt·L)·F` diverge très vite. `ModifiedLagrangian` fait bien
`MP[p].velGrad.reset()` (ligne 53) — les deux autres schémas non.

Ces deux fichiers portent l'entête « NOT anymore used … AVOID TO USE IT », mais ils restent
enregistrés dans la fabrique et `oneStepType UpdateStressLast` est accepté sans réserve.

**Correction proposée** — ajouter la remise à zéro dans les deux fichiers :

```cpp
for (size_t p = 0; p < MP.size(); p++) {
  MP[p].f.reset();
  MP[p].velGrad.reset();
}
```

Plus sûr encore : déplacer la remise à zéro au début de `MPMbox::updateVelocityGradient()`,
ce qui rend l'oubli impossible pour un futur schéma :

```cpp
void MPMbox::updateVelocityGradient() {
  for (size_t p = 0; p < MP.size(); p++) MP[p].velGrad.reset();
  ...
}
```

et retirer alors le `reset()` de `ModifiedLagrangian`.

---

## ~~B3~~ — `prev_pos` jamais mis à jour par `ModifiedLagrangian`

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichier** : `OneStep/ModifiedLagrangian.cpp` (ligne manquante), utilisé en
`BoundaryForceLaw/frictionalNormalRestitution.cpp:46`, `Spies/Work.cpp:56` et `:70`,
`Spies/EnergyBalance.cpp:36` et `:47`

`UpdateStressFirst` (ligne 136) et `UpdateStressLast` (ligne 152) font
`MP[p].prev_pos = MP[p].pos;` avant la mise à jour des positions. `ModifiedLagrangian` — le
schéma par défaut, et le seul utilisable en double échelle — ne le fait nulle part.
`prev_pos` conserve donc la valeur posée une fois pour toutes par `MPMbox::init()`
(ligne 787).

Trois conséquences, toutes silencieuses :

- `frictionalNormalRestitution` (la loi de contact **par défaut**) calcule l'incrément de
  glissement tangentiel comme `pos - prev_pos`, c'est-à-dire le **déplacement total depuis
  le début du calcul**. `ft += -kt·delta_dt` sature immédiatement au seuil de Coulomb :
  le frottement n'est plus incrémental, il est toujours au maximum et de signe arbitraire.
- Les spies `Work` et `EnergyBalance` calculent le travail des forces de contact et le
  travail du poids sur le même déplacement cumulé : les bilans d'énergie sont sans
  signification.
- `VtkOutputs/totalDisplacement.cpp` en dépend aussi (famille non compilée).

**Correction proposée** — dans `ModifiedLagrangian::advanceOneStep`, juste avant la boucle
« Update positions » (ligne 240) :

```cpp
// ==== Update positions avec le q provisoire
for (size_t p = 0; p < MP.size(); p++) {
  MP[p].prev_pos = MP[p].pos;              // <-- ajout
  I = &(Elem[MP[p].e].I[0]);
  double invmass;
  ...
}
```

À vérifier après correction : le frottement obtenu avec `frictionalNormalRestitution`
devient sensiblement différent (et cohérent avec `frictionalViscoElastic`, qui utilise
`velRelative · T · dt` et ne souffre pas du problème). Un cas test simple : un bloc posé
sur un plan incliné, qui doit rester immobile tant que `tan(pente) < mu`.

---

## ~~B4~~ — `corner[]` jamais mis à jour par `ModifiedLagrangian`

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichier** : `OneStep/ModifiedLagrangian.cpp` (ligne manquante), utilisé en
`Obstacles/Polygon.cpp:148`

Même situation : les deux autres schémas appellent `MP[p].updateCornersFromF()` en fin de
pas, `ModifiedLagrangian` non. `MaterialPoint::corner[]` garde donc les valeurs posées par
`set_MP_grid::exec()`.

`Polygon::pointinPolygon` en déduit le repère de contact :

```cpp
tang = MP.corner[2] - MP.corner[3];
tang.normalize();
normal.x = tang.y;  normal.y = -tang.x;
```

La normale de contact avec un obstacle `Polygon` est donc figée à sa valeur initiale.
L'affichage n'est pas concerné : `see` recalcule les coins dans `MPMbox::postProcess`.

**Correction proposée** — ajouter en fin de `ModifiedLagrangian::advanceOneStep` :

```cpp
for (size_t p = 0; p < MP.size(); p++) { MP[p].updateCornersFromF(); }
```

Le coût est faible (quatre produits matrice-vecteur par MP). Si on ne veut pas le payer
quand aucun `Polygon` n'est présent, le conditionner à la présence d'un obstacle qui en a
besoin. Noter que `Polygon` est lui-même marqué « NOT anymore used ».

---

## ~~B5~~ — `convergenceConditions` : `std::max` au lieu de `std::min`, et `knMax` négatif

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichier** : `Core/MPMbox.cpp:915-984`

```cpp
// Choosing critical dt as the smallest
double criticalDt = std::max({passthough_crit_dt, collision_crit_dt, cfl_crit_dt});
```

Le commentaire dit « la plus petite », le code prend la plus grande. Le pas de temps est
donc comparé au **moins** contraignant des trois critères : un `dt` qui viole le critère le
plus sévère passe sans un mot. Et quand la correction se déclenche, elle ramène `dt` à
`0,5 · max(…)`, une valeur qui peut rester au-dessus du critère le plus sévère. Le garde-fou
ne garantit donc rien.

Second défaut, dans la même fonction :

```cpp
double knMax = -inf;
...
for (it = groupsMP.begin(); ...) for (it2 = groupsObs.begin(); ...) { ... }   // vide sans obstacle
double collision_crit_dt = sqrt(massMin / knMax);                            // sqrt(négatif) = NaN
```

Sans obstacle, `groupsObs` est vide, `knMax` reste à `-DBL_MAX` et `collision_crit_dt` vaut
`NaN`. `std::max` avec un `NaN` a un résultat non spécifié, et `dt > 0.5*NaN` est faux :
aujourd'hui le contrôle est simplement inopérant, par accident.

`rayMin` mérite aussi un commentaire : `sqrt(vol/π)` est le rayon du disque de même aire que
le MP, alors que `Circle::touch` et `Line::touch` utilisent `size/2`, demi-côté du carré.
Les deux diffèrent d'un facteur ≈ 1,13.

**Correction proposée** :

```cpp
// Choisir le dt critique comme le plus petit des critères applicables
std::vector<double> crits;
crits.push_back(passthough_crit_dt);
if (knMax > 0.0) crits.push_back(sqrt(massMin / knMax));       // seulement s'il y a des obstacles
if (YoungMax > 0.0 && PoissonMax >= 0.0 && PoissonMax < 0.5) {
  double Kmax = YoungMax / (1.0 - 2.0 * PoissonMax);
  crits.push_back(rayMin / sqrt(Kmax / rhoMin));
}
if (crits.empty()) return;
double criticalDt = *std::min_element(crits.begin(), crits.end());
```

Attention à `PoissonMax >= 0.5` : `Kmax` devient infini ou négatif. Et à `CHCL_DEM`, qui
retourne `-1` pour `getYoung()`/`getPoisson()` par convention — le test actuel
`YoungMax >= 0 && PoissonMax >= 0` le gère, à condition qu'il reste.

**Cette correction va probablement réduire le `dt` de simulations existantes**, donc les
ralentir. C'est le comportement correct, mais il vaut mieux le savoir avant de relancer une
campagne.

---

## ~~B6~~ — `GravityRamp` : interpolation sans le terme constant

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichier** : `Schedulers/GravityRamp.cpp:24-32`

```cpp
if (box->t <= rampStart)      box->gravity = gravityFrom;
else if (box->t >= rampStop)  box->gravity = gravityTo;
else box->gravity = (box->t - rampStart) / (rampStop - rampStart) * (gravityTo - gravityFrom);
```

Il manque `gravityFrom +`. Pendant la rampe, la gravité vaut `s·(g₁ − g₀)` au lieu de
`g₀ + s·(g₁ − g₀)` : elle saute à `−g₀` à l'instant `rampStart`, puis à `g₁` à `rampStop`.
Avec la rampe la plus courante (`gravityFrom = 0 0`), l'erreur est invisible ; avec toute
autre valeur de départ, la gravité est discontinue aux deux extrémités.

**Correction proposée** :

```cpp
} else {
  const double s = (box->t - rampStart) / (rampStop - rampStart);
  box->gravity = gravityFrom + s * (gravityTo - gravityFrom);
}
```

Ajouter aussi une garde `rampStop > rampStart` dans `read()`, sinon division par zéro.

---

## ~~B7~~ — `KelvinVoigt` : la contrainte visqueuse est cumulée

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichier** : `ConstitutiveModels/KelvinVoigt.cpp:32-42`

```cpp
mat9r Sigma(MPM.MP[p].stress.xx, ...);   // état de contrainte au pas précédent
...
Sigma += C.getStress(dstrain3x3);        // part élastique : incrément, correct
Sigma += (eta / MPM.dt) * dstrain3x3;    // part visqueuse : ajoutée au cumul
```

La contrainte visqueuse d'un modèle de Kelvin-Voigt est `η·ε̇`, une quantité **instantanée**
qui dépend de la vitesse de déformation courante et ne doit pas s'accumuler. Ici elle est
ajoutée à chaque pas au total déjà accumulé : la somme vaut `Σᵢ η·ε̇ᵢ = η·ε_total/dt`.
Le terme se comporte donc comme une **rigidité additionnelle** de module `η/dt` — d'autant
plus grande que le pas de temps est petit — et non comme un amortisseur. Le modèle n'est pas
un Kelvin-Voigt.

**Correction proposée** — retirer la contribution visqueuse du pas précédent avant d'ajouter
la nouvelle. `MaterialPoint` n'a pas de champ dédié ; le plus propre est d'en ajouter un
(`mat4r viscousStress;`), le plus économe est de réutiliser `stressCorrection`, qui n'est
lu que par `MohrCoulomb` :

```cpp
// dans KelvinVoigt::updateStrainAndStress, avec un champ MP.viscousStress
mat9r Sigma(MPM.MP[p].stress.xx - MPM.MP[p].viscousStress.xx, ...);  // retirer l'ancienne part visqueuse
Sigma += C.getStress(dstrain3x3);                                    // élasticité (cumulative)

mat9r visc = (eta / MPM.dt) * dstrain3x3;                            // nouvelle part visqueuse
Sigma += visc;

MPM.MP[p].viscousStress.xx = visc.xx;  // mémoriser pour le pas suivant
MPM.MP[p].viscousStress.xy = visc.xy;
MPM.MP[p].viscousStress.yx = visc.yx;
MPM.MP[p].viscousStress.yy = visc.yy;
```

Le nouveau champ doit être ajouté à la sauvegarde (voir **C3**), sans quoi une reprise fait
un saut de contrainte. Cas test : une éprouvette relâchée doit voir sa contrainte visqueuse
tomber à zéro dès que `ε̇ = 0`, ce qui n'est pas le cas aujourd'hui.

---

## B8 — `VonMises` : `plasticStrain` écrasée au lieu d'être cumulée

**Fichier** : `ConstitutiveModels/VonMisesElastoPlasticity.cpp:93-96`

```cpp
MPM.MP[p].plasticStrain.xx = lambdadot * gradf.xx;
MPM.MP[p].plasticStrain.yy = lambdadot * gradf.yy;
MPM.MP[p].plasticStrain.xy = lambdadot * gradf.xy;
MPM.MP[p].plasticStrain.yx = MPM.MP[p].plasticStrain.xy;
```

`plasticStrain` est déclarée « Plastic Strain » et `MohrCoulomb` l'incrémente
(`plasticStrain += deltaPlasticStrain`, ligne 120). Ici elle est **remplacée** par
l'incrément du pas courant : ce n'est donc pas la déformation plastique cumulée, mais le
dernier incrément. La grandeur est sauvegardée dans les conf-files et affichée par `see` —
elle est trompeuse dans les deux cas.

Le correcteur de contrainte, lui, est bien calculé à partir de cet incrément — donc la
contrainte est correcte, seule la variable d'histoire est fausse. Corriger l'un sans l'autre
casserait le modèle.

**Correction proposée** — travailler sur un incrément local et cumuler :

```cpp
mat4r dEp;
dEp.xx = lambdadot * gradf.xx;
dEp.yy = lambdadot * gradf.yy;
dEp.xy = lambdadot * gradf.xy;
dEp.yx = dEp.xy;

MPM.MP[p].plasticStrain += dEp;          // cumul, comme MohrCoulomb

stressCorrection.xx = f * (dEp.xx * (1.0 - Poisson) + dEp.yy * Poisson);
stressCorrection.yy = f * (dEp.xx * Poisson + dEp.yy * (1.0 - Poisson));
stressCorrection.xy = f * (dEp.xy * (1.0 - 2.0 * Poisson));
stressCorrection.yx = stressCorrection.xy;
```

Attention : la contrainte reste inchangée, mais toute reprise de calcul existante repartira
d'une `plasticStrain` interprétée différemment.

---

## ~~B9~~ — Historique de contact perdu : comparaison inversée

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichiers** : `Obstacles/Circle.cpp:80-95`, `Obstacles/Line.cpp:78-91`,
`Obstacles/Polygon.cpp:106-117`

Les trois obstacles reconstruisent leur liste de voisins puis tentent de récupérer les
forces mémorisées. Les deux listes sont triées par `PointNumber` croissant (la boucle de
construction parcourt `p` dans l'ordre). La fusion s'écrit :

```cpp
size_t istore = 0;
for (size_t inew = 0; inew < Neighbors.size(); inew++) {
  while (istore < Store.size() && Neighbors[inew].PointNumber < Store[istore].PointNumber) {
    ++istore;                                   // comparaison inversée
  }
  if (istore == Store.size()) break;
  if (Store[istore].PointNumber == Neighbors[inew].PointNumber) { ...restaurer... ; ++istore; }
}
```

Pour avancer dans `Store` jusqu'au point recherché il faut avancer **tant que le stocké est
en retard**, donc `Store[istore].PointNumber < Neighbors[inew].PointNumber`. Avec la
comparaison actuelle, `istore` n'avance jamais quand `Store` est en retard.

Conséquence : tant que la liste ne change pas, tout fonctionne (les deux listes sont
identiques, la comparaison est fausse dans les deux sens et on tombe directement sur
l'égalité). Dès qu'un point **quitte** la liste, la synchronisation est perdue et
**l'historique de tous les points suivants est jeté**. Exemple : `Store = [3, 5, 7]`,
`Neighbors = [5, 7]` → aucune restauration.

En pratique, tous les `proxPeriod` pas, les forces tangentielles accumulées (`ft`) et les
enfoncements (`dn`) repartent de zéro pour une partie des contacts. Le frottement est
sous-estimé et de l'énergie est injectée ou dissipée sans raison physique.

**Correction proposée** (identique dans les trois fichiers) :

```cpp
while (istore < Store.size() && Store[istore].PointNumber < Neighbors[inew].PointNumber) {
  ++istore;
}
```

Le reste de la boucle est correct. Une écriture plus lisible et sans piège serait une
`std::map<size_t, Neighbor>` ou un `std::set_intersection`, mais la correction d'un
caractère suffit et se vérifie facilement (`Store = [3,5,7]`, `Neighbors = [5,7]` doit
restaurer les deux).

---

## ~~B10~~ — `Circle` et `Polygon` ne restaurent que `fn` et `ft`

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichiers** : `Obstacles/Circle.cpp:90-91`, `Obstacles/Polygon.cpp:112-113`

```cpp
Neighbors[inew].fn = Store[istore].fn;
Neighbors[inew].ft = Store[istore].ft;
```

`Line` restaure la structure entière (`Neighbors[inew] = Store[istore];`, ligne 88).
`Circle` et `Polygon` perdent `dn`, `dt` et `sigma_n` à chaque reconstruction.

- `dn` est lu par `frictionalNormalRestitution` pour décider charge/décharge
  (`delta_dn = dn - Neighbors[nn].dn`) : avec `dn` remis à zéro, le pas suivant la
  reconstruction est systématiquement traité comme une décharge.
- `sigma_n` est lu par `frictionalViscoElastofragile` pour le seuil de frottement.
- `dn` est aussi le test d'activité des spies `Work` et `EnergyBalance`
  (`if (Neighbors[nn].dn >= 0.0) continue;`).

**Correction proposée** — aligner sur `Line` :

```cpp
if (Store[istore].PointNumber == Neighbors[inew].PointNumber) {
  Neighbors[inew] = Store[istore];
  ++istore;
}
```

À corriger en même temps que **B9**, dont ce défaut est le voisin immédiat.

---

## ~~B11~~ — Découpage adaptatif : quatre problèmes cumulés

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction, **et se trompe sur `vol0`/`size`** : voir le journal.

**Fichier** : `Core/MPMbox.cpp:1094-1184`

```cpp
for (size_t p = 0; p < MP.size(); p++) {
  ...
  MP[p].mass *= 0.5;
  MP[p].vol  *= 0.5;
  MaterialPoint MP2 = MP[p];
  ...
  MP.push_back(MP2);
}
```

1. **`vol0` et `size` ne sont pas divisés.** `mass` et `vol` le sont, `F` est divisé par 2
   dans la direction du découpage — la géométrie courante est donc cohérente
   (`corner = pos + F·(±size/2)`). Mais `vol0` reste la valeur d'origine, et
   `MPMbox::read` reconstruit `size = sqrt(vol0)` à la relecture : **après une reprise, tous
   les MP découpés retrouvent leur taille initiale** alors que leur `F` est resté divisé.
   `UpdateStressFirst`/`Last` recalculent aussi `vol = F.det()·vol0`, ce qui donne un volume
   moitié du bon.
   `frictionalViscoElastofragile` utilise `vol0` directement (lignes 21 et 43).

2. **`MP2.nb` n'est pas attribué.** Les deux moitiés portent le même numéro.

3. **La borne de boucle grandit pendant l'itération.** `MP.push_back` invalide toute
   référence et le nouveau point est visité dans la même passe. Il est protégé par
   `splitCount > MaxSplitNumber`, mais le comportement dépend de l'ordre — et un
   `MP[p]` gardé sous forme de référence provoquerait un accès à de la mémoire libérée.

4. **`MP2.PBC` est un pointeur copié.** En double échelle, les deux moitiés partagent la
   **même** cellule DEM. Dans `ModifiedLagrangian`, `updateStrainAndStress` est appelée dans
   une boucle `#pragma omp parallel for` : deux threads appellent `PBC->transform()` sur le
   même objet — course de données. Et la cellule est détruite deux fois si un `delete` est
   ajouté un jour.

**Correction proposée** :

```cpp
if ((critX || critY) == true) {
  MP[p].splitCount += 1;
  double halfSizeMP = 0.5 * MP[p].size;

  MP[p].mass *= 0.5;
  MP[p].vol  *= 0.5;
  MP[p].vol0 *= 0.5;                       // (1)

  MaterialPoint MP2 = MP[p];
  MP2.nb  = nextMPNumber++;                // (2) compteur membre de MPMbox
  MP2.PBC = nullptr;                       // (4)
  if (MP[p].isDoubleScale) {
    MP2.PBC  = new PBC3Dbox(*MP[p].PBC);   // copie profonde, à vérifier côté PBC3D
  }
  ...
}
```

Pour (1), noter que `size = sqrt(vol0)` n'est valable que pour un MP carré : après un
découpage dans une seule direction le point n'est plus carré, et l'invariant
`vol0 = size²` est rompu. La solution propre est de stocker `size` explicitement dans le
conf-file plutôt que de le recalculer à la lecture — voir **C3**.

Pour (3), fixer la borne avant la boucle :

```cpp
const size_t nbBefore = MP.size();
for (size_t p = 0; p < nbBefore; p++) { ... }
```

Pour (4), la duplication d'une cellule DEM demande un constructeur de copie fiable côté
`PBC3Dbox`. Tant que ce n'est pas vérifié, le plus sûr est de **refuser** le découpage des
MP double échelle :

```cpp
if (MP[p].isDoubleScale) continue;   // le découpage d'une cellule DEM n'a pas de sens
```

---

## B12 — Les spies vident leurs fichiers en mode visualisation

**Fichiers** : `Spies/MeanStress.cpp:15`, `Spies/MPTracking.cpp:26`,
`Spies/EnergyBalance.cpp:19`, `Spies/ElasticBeamDev.cpp:19` ; `See/cut.cpp:8-15`

`Work` et `ObstacleTracking` protègent l'ouverture :

```cpp
if (box->computationMode) { file.open(filename.c_str()); }
```

Les quatre autres spies ouvrent sans condition. Or `see` et `cut` lisent les conf-files avec
le même `MPMbox::read()`, qui exécute `spy->read(file)` — et les conf-files contiennent
rarement des lignes `Spy`… sauf si l'utilisateur les a ajoutées pour reprendre un calcul
(procédure décrite dans le manuel, § « Restarting a computation »). Dans ce cas, **ouvrir la
simulation dans `see` efface les fichiers de résultats**.

De plus `MPMbox::clean()` ne vide pas `Spies` (voir **C1**) : chaque conf-file relu dans
`see` ajoute une nouvelle instance de chaque spy et rouvre — donc revide — le fichier.
Parcourir 200 conf-files avec `+` crée 200 spies.

`cut.cpp` est plus exposé encore : il ne met jamais `computationMode` à `false`, donc même
`Work` et `ObstacleTracking` y écrasent leurs fichiers.

**Correction proposée** — appliquer partout la garde existante :

```cpp
// MeanStress.cpp, MPTracking.cpp, EnergyBalance.cpp, ElasticBeamDev.cpp
if (box->computationMode) { file.open(filename.c_str()); }
```

et protéger toutes les écritures :

```cpp
void MeanStress::record() {
  if (!file.is_open()) return;
  ...
}
void MeanStress::end() {
  if (file.is_open()) file.close();
}
```

(`Work::end()` écrit aujourd'hui dans `fileSlices` sans vérifier `is_open`.)

Dans `See/cut.cpp:8` :

```cpp
void try_to_readConf(int num, MPMbox& CF, std::string ca) {
  char file_name[256];
  snprintf(file_name, 256, "%s%d.txt", ca.c_str(), num);
  std::cout << "Read " << file_name << std::endl;
  CF.computationMode = false;         // <-- ajout
  CF.clean();
  CF.read(file_name);
  CF.postProcess(SmoothedData);
}
```

La solution de fond est que `computationMode` soit une donnée du constructeur plutôt qu'un
drapeau positionné après coup, pour qu'un outil de visualisation ne puisse structurellement
pas ouvrir un fichier en écriture.

---

## B13 — `add_MP_ShallowPath` : borne en x fausse, `init()` du modèle non appelée

**Fichier** : `Commands/add_MP_ShallowPath.cpp:27-42`

```cpp
vec2r vecA = pathPoints[i + 1] - pathPoints[i];
double maxx = vecA * vec2r::unit_x();                     // composante x du VECTEUR
for (double x = pathPoints[i].x + halfSizeMP; x <= maxx - halfSizeMP; x += size) {
```

`maxx` est la **longueur** du segment en x, comparée à une **abscisse absolue**. La boucle
n'est correcte que pour un chemin qui commence en `x = 0` : au-delà du premier segment,
elle s'arrête trop tôt, ou ne produit aucun point.

Deux autres problèmes dans la même fonction :

- `CM->init(P)` n'est pas appelé, contrairement à `set_MP_grid` et `set_MP_polygon`. Avec
  `CHCL_DEM`, les points créés restent `isDoubleScale = false` et `PBC = nullptr` : la loi
  double échelle est silencieusement inactive, et `CHCL_DEM::updateStrainAndStress`
  déréférence `PBC` au premier pas.
- `P.nb` n'est jamais attribué (tous les points portent `nb = 0`).
- `pathPoints.size() - 1` avec un `size_t` : si `nbPathPoints` vaut 0, la boucle part sur
  `SIZE_MAX` itérations.

**Correction proposée** :

```cpp
void add_MP_ShallowPath::exec() {
  double halfSizeMP = 0.5 * size;

  auto itCM = box->models.find(modelName);
  if (itCM == box->models.end()) {
    Logger::critical("@add_MP_ShallowPath::exec, modèle '{}' inconnu", modelName);
    exit(EXIT_FAILURE);
  }
  ConstitutiveModel* CM = itCM->second;

  if (pathPoints.size() < 2) {
    Logger::warn("@add_MP_ShallowPath::exec, il faut au moins 2 points de chemin");
    return;
  }

  for (size_t i = 0; i + 1 < pathPoints.size(); i++) {
    const double xEnd = pathPoints[i + 1].x;                  // abscisse, pas longueur
    for (double x = pathPoints[i].x + halfSizeMP; x <= xEnd - halfSizeMP; x += size) {
      double ypos = lineEquation(pathPoints[i], pathPoints[i + 1], x);
      for (double y = ypos + halfSizeMP; y <= ypos + height - halfSizeMP; y += size) {
        MaterialPoint P(groupNb, size, rho, CM);
        CM->init(P);                                          // <-- ajout
        P.pos.set(x, y);
        P.nb = box->MP.size();
        box->MP.push_back(P);
      }
    }
  }
}
```

`lineEquation` divise par `point2.x - point1.x` : un segment vertical donne `inf`. À garder
en tête, ou à documenter comme une limite de la commande.

---

## B14 — `set_MP_polygon` : allocation DEM pour chaque point rejeté

**Fichier** : `Commands/set_MP_polygon.cpp:37-47`

```cpp
for (double y = ...; ...) {
  for (double x = ...; ...) {
    vec2r point(x, y);
    MaterialPoint P(groupNb, size, rho, CM);
    CM->init(P);                        // alloue et charge une cellule DEM
    if (isInside(vertices, nbVertices, point)) {
      P.pos.set(x, y);
      box->MP.push_back(P);
    }
  }                                     // sinon P est détruit, mais P.PBC fuit
}
```

Le point est construit et initialisé **avant** le test d'appartenance. Avec `CHCL_DEM`,
`init()` fait `new PBC3Dbox` puis `loadConf(fileName)`. Pour une géométrie non convexe ou
simplement inscrite dans une grande boîte englobante, cela peut représenter des milliers de
lectures de fichier DEM inutiles — et autant de fuites, `MaterialPoint` n'ayant pas de
destructeur qui libère `PBC`.

Le résultat est correct, mais la commande peut mettre plusieurs minutes et consommer
plusieurs gigaoctets pour créer quelques centaines de points.

**Correction proposée** — tester d'abord :

```cpp
for (double y = miny + halfSizeMP; y <= maxy - halfSizeMP; y += size) {
  for (double x = minx + halfSizeMP; x <= maxx - halfSizeMP; x += size) {
    vec2r point(x, y);
    if (!isInside(vertices, nbVertices, point)) continue;   // <-- avant toute allocation
    MaterialPoint P(groupNb, size, rho, CM);
    CM->init(P);
    P.pos.set(x, y);
    P.nb = box->MP.size();
    box->MP.push_back(P);
  }
}
```

`isInside` prend son polygone **par valeur** (`std::vector<vec2r> polygon`) : une copie du
vecteur à chaque appel, soit une copie par point candidat. Passer par
`const std::vector<vec2r>&`.

---

# C — Moyen (robustesse, reprise, fuites)

## C1 — `clean()` ne libère qu'une partie des ressources

**Fichier** : `Core/MPMbox.cpp:293-304`

```cpp
void MPMbox::clean() {
  nodes.clear();  Elem.clear();  MP.clear();
  for (size_t i = 0; i < Obstacles.size(); i++) { delete (Obstacles[i]); }
  Obstacles.clear();
  for (itModel = models.begin(); itModel != models.end(); ++itModel) { delete itModel->second; }
  models.clear();
}
```

Ne sont ni libérés ni vidés :

| Ressource | Conséquence |
|---|---|
| `Spies` | fuite, **et** accumulation dans `see` (chaque conf relu ajoute une instance et rouvre le fichier — voir **B12**) |
| `Scheduled` | fuite et accumulation identiques ; les schedulers d'un conf s'ajoutent à ceux du précédent |
| `shapeFunction`, `oneStep` | fuite à chaque `read()` (ceux-là sont bien remplacés avec `delete` dans `read`, mais pas libérés par `clean`) |
| `Obstacle::boundaryForceLaw` | jamais libérée (`~Obstacle()` est vide) ; la loi par défaut est en plus perdue quand `BoundaryForceLaw` la remplace |
| `MaterialPoint::PBC` | jamais libérée — un `MP.clear()` détruit les `MaterialPoint` sans toucher aux cellules DEM |
| `liveNodeNum` | non vidé (compensé par `postProcess`, mais fragile) |
| `controlledMP`, `BFLCommandStored` | non vidés (`read()` vide le second) |

Pour `mpmbox`, l'impact est limité à la fin du processus. Pour `see`, qui appelle
`clean()` puis `read()` à chaque changement de conf-file, la mémoire croît sans borne et les
spies se dupliquent.

**Correction proposée** :

```cpp
void MPMbox::clean() {
  nodes.clear();
  Elem.clear();
  liveNodeNum.clear();

  for (size_t p = 0; p < MP.size(); p++) { delete MP[p].PBC; MP[p].PBC = nullptr; }
  MP.clear();

  for (size_t i = 0; i < Obstacles.size(); i++) delete Obstacles[i];
  Obstacles.clear();

  for (size_t s = 0; s < Spies.size(); s++) delete Spies[s];
  Spies.clear();

  for (size_t s = 0; s < Scheduled.size(); s++) delete Scheduled[s];
  Scheduled.clear();

  delete shapeFunction; shapeFunction = nullptr;
  delete oneStep;       oneStep       = nullptr;

  for (auto it = models.begin(); it != models.end(); ++it) delete it->second;
  models.clear();

  controlledMP.clear();
  BFLCommandStored.clear();
}
```

et `~Obstacle() { delete boundaryForceLaw; }` — **à condition** d'avoir d'abord réglé le
partage d'instance décrit en **A6**, sinon double libération.

`delete MP[p].PBC` suppose que le pointeur n'est pas partagé : à faire après **B11** (point 4).

---

## C2 — `Elem` non vidé (cas 16 nœuds), `liveNodeNum` empilé

**Fichiers** : `Commands/set_node_grid.cpp:72-125`, `Commands/new_set_grid.cpp:41-94`

```cpp
if (element::nbNodes == 4) {
  if (!box->Elem.empty()) box->Elem.clear();      // seulement ici
  ...
} else if (element::nbNodes == 16) {
  // pas de clear
  ...
}
...
for (size_t i = 0; i < box->nodes.size(); i++) {
  box->liveNodeNum.push_back(i);                  // jamais vidé avant
}
```

Deux appels successifs à `set_node_grid` (ou un `see` qui relit des conf-files) doublent
`Elem` en 16 nœuds, et empilent `liveNodeNum` à chaque fois.

**Correction proposée** — sortir le nettoyage des branches :

```cpp
box->Elem.clear();
box->liveNodeNum.clear();

if (element::nbNodes == 4) {
  ...
} else if (element::nbNodes == 16) {
  ...
} else {
  Logger::critical("element::nbNodes = {} : seules les valeurs 4 et 16 sont admises", element::nbNodes);
  exit(EXIT_FAILURE);
}

for (size_t i = 0; i < box->nodes.size(); i++) box->liveNodeNum.push_back(i);
```

Ajouter aussi un contrôle sur la taille de grille, aujourd'hui absent des trois variantes :

```cpp
if (nbElemX == 0 || nbElemY == 0 || lx <= 0.0 || ly <= 0.0) {
  Logger::critical("@set_node_grid : grille invalide (Nx={}, Ny={}, lx={}, ly={})",
                   nbElemX, nbElemY, lx, ly);
  exit(EXIT_FAILURE);
}
```

(`set_node_grid W.H.lx.ly` avec `lx > W` donne `nbElemX = 0`.)

Enfin, `new_set_grid` est un doublon quasi exact de `set_node_grid` avec une grille carrée.
Deux copies du même code à 16 nœuds, à maintenir en parallèle : il vaudrait mieux que
`new_set_grid` délègue à `set_node_grid`, ou disparaisse.

---

## C3 — `save()` incomplet : la reprise n'est pas exacte

**Fichier** : `Core/MPMbox.cpp:614-739`

Ne sont pas écrits dans le conf-file :

| Donnée | Conséquence de l'omission |
|---|---|
| `MaterialPoint::hardeningForce` | `SinfoniettaClassica` et `SinfoniettaCrush` : `init()` remet `-log(pc0)` si la valeur relue est 0.0 — **l'écrouissage est remis à son état initial** |
| `MaterialPoint::outOfPlaneEp` | déformation plastique hors plan perdue |
| `MaterialPoint::viscousStress` | ajouté le 2026-08-05 pour **B7** : la contribution visqueuse du dernier pas reste comme décalage constant après reprise |
| `MaterialPoint::size` | recalculé par `sqrt(vol0)`, faux après découpage (voir **B11**) |
| `MaterialPoint::isTracked` | les dossiers `DEM_MP<p>/` ne sont plus alimentés après reprise |
| `MaterialPoint::prev_F` | `CHCL_DEM` calcule `Finc = F · prev_F⁻¹` : premier pas après reprise incohérent |
| `MaterialPoint::plastic` | drapeau d'affichage |
| État DEM de chaque MP | `CHCL_DEM::init()` recharge le `fileName` **initial** pour tous les points, puis **écrase la contrainte relue** par celle de la cellule initiale : une reprise en double échelle repart d'un empilement neuf |
| `splitCriterionValue`, `MaxSplitNumber`, `shearLimit`, `splittingExtremeShearing` | le comportement du découpage change à la reprise |
| `Spy` | déjà documenté dans le manuel |
| `select_tracked_MP` / `select_controlled_MP` | sélections perdues |
| `dtInitial` | recalculé comme `dt`, donc figé à la valeur réduite par `limitTimeStepForDEM` |

À l'inverse, `stressCorrection` est écrit alors qu'il n'est qu'une variable de travail
de `MohrCoulomb` — le `FIXME` ligne 519-522 le signale déjà.

**Correction proposée** — le format de conf-file étant aussi le format d'entrée, il ne peut
pas changer sans casser la compatibilité. Deux étapes :

1. Ajouter les scalaires manquants, qui sont sans risque (mots-clés nouveaux, ignorés par
   les anciennes versions… mais qui feront un « what do you mean by ? » — donc à ajouter en
   même temps dans `read()`) :

   ```cpp
   file << "splitCriterionValue " << splitCriterionValue << '\n';
   file << "MaxSplitNumber "      << MaxSplitNumber      << '\n';
   file << "shearLimit "          << shearLimit          << '\n';
   file << "splittingExtremeShearing " << extremeShearing << ' ' << extremeShearingval << '\n';
   ```

2. Pour les champs des MP, écrire une **nouvelle** section optionnelle plutôt que d'allonger
   la ligne existante — les anciens fichiers restent lisibles, les nouveaux aussi :

   ```
   MPextra 3 hardeningForce outOfPlaneEp size
   <nb> <hardeningForce> <outOfPlaneEp> <size>
   ...
   ```

   Un en-tête nommant les colonnes rend le format extensible sans nouvelle rupture, ce qui
   est exactement le problème rencontré ici.

Pour la double échelle, sauvegarder l'état DEM de **tous** les MP (pas seulement les points
suivis) et le recharger dans `CHCL_DEM::init()` est un chantier à part entière ; à défaut,
la limitation mérite d'être écrite noir sur blanc dans le manuel : *une reprise de calcul
double échelle n'est pas une reprise, c'est un nouveau calcul avec l'état MPM du précédent.*

---

## C4 — `Polygon` : `rot` en radians à l'écriture, en degrés à la lecture

**Fichier** : `Obstacles/Polygon.cpp:14-52`, `:170-192`

```cpp
void Polygon::read(std::istream& is) {
  is >> group >> nVertices >> pos >> rot >> R;
  ...
  rot *= Mth::pi / 180.0;                       // degrés -> radians
  createPolygon(verticePos);
}
void Polygon::write(std::ostream& os) {
  os << group << ' ' << nVertices << ' ' << pos << ' ' << rot << ' ' << R << ' ';   // radians
```

L'angle est reconverti à chaque cycle sauvegarde/relecture : un polygone à 45° devient 0,785°
au premier conf-file relu.

Trois autres défauts dans le même fichier :

- `createPolygon` boucle `for (int i = 0; i <= nVertices; i++)` : elle crée `nVertices + 1`
  sommets, le dernier confondu avec le premier.
- `Area()` applique la formule du lacet **autour de l'origine**
  (`determinant(v[i], v[i+1])`) et non autour d'un point du polygone : l'aire n'est correcte
  que si le polygone est centré en (0,0). Elle sert à calculer la masse en mode `free`.
- Dans `read()`, la branche `free` appelle `Area()` et lit `verticePos[0]`/`[1]` **avant** la
  conversion degrés → radians et avant le `verticePos.clear()` de la ligne 39 ; l'ordre est
  fragile même s'il fonctionne par accident (`Area()` reconstruit la liste).

`Polygon` porte l'entête « NOT anymore used … AVOID TO USE IT ». À décider : réparer ou
retirer de la fabrique.

**Correction proposée** si on répare :

```cpp
void Polygon::write(std::ostream& os) {
  os << group << ' ' << nVertices << ' ' << pos << ' ' << rot * 180.0 / Mth::pi << ' ' << R << ' ';
```

```cpp
void Polygon::createPolygon(std::vector<vec2r>& vect) {
  vec2r P;
  double inc = Mth::_2pi / (double)nVertices;
  for (int i = 0; i < nVertices; i++) {          // < et non <=
    P.x = pos.x + R * cos(rot + i * inc);
    P.y = pos.y + R * sin(rot + i * inc);
    vect.push_back(P);
  }
}

double Polygon::Area() {
  // polygone régulier de rayon circonscrit R à nVertices côtés
  return 0.5 * nVertices * R * R * sin(Mth::_2pi / (double)nVertices);
}
```

(La forme analytique évite complètement le problème du lacet, le polygone étant régulier
par construction.)

---

## C5 — `postProcess` divise par la masse nodale sans tolérance

**Fichier** : `Core/MPMbox.cpp:1231-1237`

```cpp
for (size_t r = 0; r < element::nbNodes; r++) {
  nodes[I[r]].vel    += MP[p].N[r] * MP[p].mass * MP[p].vel    / nodes[I[r]].mass;
  nodes[I[r]].stress += MP[p].N[r] * MP[p].mass * MP[p].stress / nodes[I[r]].mass;
}
```

Aucun test `> tolmass`, contrairement à tous les schémas d'intégration. Un nœud dont la
masse rapportée est nulle ou minuscule (nœud de bord du support, MP juste sur une arête)
produit `inf` ou `NaN`, qui se propage dans `Data[p].vel` et `Data[p].stress` et donc dans
l'affichage de `see` et les sorties de `cut`.

**Correction proposée** :

```cpp
for (size_t r = 0; r < element::nbNodes; r++) {
  if (nodes[I[r]].mass <= tolmass) continue;
  const double invmass = 1.0 / nodes[I[r]].mass;
  nodes[I[r]].vel    += MP[p].N[r] * MP[p].mass * MP[p].vel    * invmass;
  nodes[I[r]].stress += MP[p].N[r] * MP[p].mass * MP[p].stress * invmass;
}
```

---

## C6 — `MohrCoulomb` : `apex` divisé par `sin(phi)`, non-convergence silencieuse

**Fichier** : `ConstitutiveModels/MohrCoulomb.cpp:90-140`

```cpp
double apex = Cohesion * cosFrictionAngle / sinFrictionAngle;
```

Avec `FrictionAngle = 0` (matériau de Tresca) : `apex = c/0 = +inf` si `c > 0` — ce qui
fonctionne par chance, `s < inf` étant toujours vrai. Mais avec `c = 0` **et** `phi = 0`,
`apex = 0/0 = NaN`, `s < NaN` est faux, on part dans la branche « apex » et la contrainte
devient `NaN` pour ce point, puis pour tout son voisinage via l'interpolation.

La boucle de retour radial ne signale rien si elle n'a pas convergé :

```cpp
while (iter < 50 && yieldF > 1e-10) { ... }
```

Sortie à 50 itérations avec `yieldF` encore positif = état de contrainte hors de la surface
de charge, sans message.

`1e-10` est par ailleurs un seuil **absolu** sur une quantité homogène à une contrainte :
inadapté pour des matériaux à modules très différents (kPa vs GPa).

**Correction proposée** :

```cpp
// dans read() et le constructeur
if (FrictionAngle <= 0.0 && Cohesion <= 0.0) {
  Logger::critical("MohrCoulomb : phi et c ne peuvent pas être nuls tous les deux");
  exit(EXIT_FAILURE);
}
```

```cpp
// seuil relatif
const double yieldTol = 1e-10 * std::max(1.0, 2.0 * Cohesion * cosFrictionAngle);
int iter = 0;
while (iter < 50 && yieldF > yieldTol) { ... }
if (iter >= 50) {
  Logger::warn("@MohrCoulomb : retour radial non convergé pour le MP {} (f = {})", p, yieldF);
}
```

Le cas `sinFrictionAngle == 0` demande un traitement séparé (la surface de Tresca n'a pas
d'apex) ; à défaut, refuser `FrictionAngle = 0` comme ci-dessus.

Voir aussi **D5** : le commentaire décrivant la matrice `De` ne correspond pas au code
(le code est correct).

---

## C7 — `adaptativeRefinement` : division par une extension nulle

**Fichier** : `Core/MPMbox.cpp:1107-1112`

```cpp
double XSquaredExtent = (MP[p].F.xx * MP[p].F.xx + MP[p].F.yx * MP[p].F.yx);
double YSquaredExtent = (MP[p].F.xy * MP[p].F.xy + MP[p].F.yy * MP[p].F.yy);
bool critX = ((XSquaredExtent / YSquaredExtent) >= SquaredCrit);
bool critY = ((YSquaredExtent / XSquaredExtent) >= SquaredCrit);
```

Si `F` devient singulier dans une direction (écrasement total), le dénominateur s'annule :
`x/0` vaut `+inf` (critère vrai, découpage systématique) ou `NaN` (comparaison fausse,
jamais de découpage) selon le numérateur. Aucun des deux n'est le comportement voulu.

**Correction proposée** :

```cpp
const double eps = 1e-12;
bool critX = (YSquaredExtent > eps) && (XSquaredExtent >= SquaredCrit * YSquaredExtent);
bool critY = (XSquaredExtent > eps) && (YSquaredExtent >= SquaredCrit * XSquaredExtent);
```

La forme multiplicative évite complètement la division.

---

## C8 — `set_K0_stress` : normalisation d'une gravité nulle

**Fichier** : `Commands/set_K0_stress.cpp:11-12`

```cpp
vec2r ug = box->gravity;
double g = ug.normalize();     // gravité nulle -> ug = (NaN, NaN)
```

Si `set_K0_stress` est placé avant `gravity` dans le fichier d'entrée (ou si la gravité est
appliquée par un `GravityRamp` partant de zéro), toutes les contraintes initiales deviennent
`NaN`.

La commande mélange par ailleurs deux repères : la profondeur `d` est mesurée le long de la
gravité, mais la contrainte est affectée à `stress.yy` / `stress.xx`. Ce n'est cohérent que
si la gravité est verticale.

**Correction proposée** :

```cpp
void set_K0_stress::exec() {
  if (box->MP.empty()) return;

  vec2r ug = box->gravity;
  double g = ug.normalize();
  if (g < 1e-12) {
    Logger::warn("@set_K0_stress : gravité nulle, commande ignorée "
                 "(placer 'gravity' avant 'set_K0_stress')");
    return;
  }
  if (fabs(ug.x) > 1e-9) {
    Logger::warn("@set_K0_stress : la gravité n'est pas verticale, "
                 "l'état K0 est appliqué sur les axes x et y du repère global");
  }
  ...
}
```

---

## C9 — `PICDissipation` lit un ratio FLIP là où `enablePIC` lit un ratio PIC

**Fichier** : `Schedulers/PICDissipation.cpp:5-12`

```cpp
void PICDissipation::read(std::istream& is) {
  is >> box->ratioFLIP >> endTime;      // ratio FLIP
  box->activePIC = true;
}
```

à comparer à `MPMbox::read` (ligne 358) :

```cpp
} else if (token == "enablePIC") {
  double ratioPIC;  file >> ratioPIC;
  ratioFLIP = 1.0 - ratioPIC;           // ratio PIC
```

et à `PICDissipationByPIC` (ligne 69), qui prend lui aussi un ratio **PIC**. Deux mots-clés
sur trois prennent un ratio PIC, le troisième un ratio FLIP, sans que le nom le laisse
deviner : `Scheduled PICDissipation 0.95 1.0` amortit très peu, alors que
`enablePIC 0.95` amortit énormément.

Trois défauts connexes :

- `read()` écrit directement dans `box->ratioFLIP`, donc **au moment de la lecture** : il
  écrase la valeur d'un `enablePIC` situé plus loin dans le fichier, et est écrasé par un
  `enablePIC` situé avant — le résultat dépend de l'ordre des lignes.
- `write()` écrit `box->ratioFLIP`, c'est-à-dire la valeur **courante**, éventuellement
  modifiée par un autre scheduler : la sauvegarde ne restitue pas le réglage d'origine.
- Une fois `activePIC` passé à `false`, `check()` ne peut plus le remettre à `true`
  (`if (box->activePIC == true)`) : un second `PICDissipation` programmé plus tard n'a
  aucun effet.

**Correction proposée** — stocker le réglage dans le scheduler, l'appliquer dans `check()`,
et aligner la convention sur le ratio PIC en gardant la compatibilité :

```cpp
struct PICDissipation : public Scheduler {
  double ratioFLIPvalue{0.95};
  double endTime{0.0};
  ...
};

void PICDissipation::read(std::istream& is) {
  is >> ratioFLIPvalue >> endTime;                 // convention conservée pour ne rien casser
  if (ratioFLIPvalue < 0.0 || ratioFLIPvalue > 1.0)
    Logger::warn("PICDissipation : le premier argument est un ratio FLIP dans [0,1] "
                 "(et non un ratio PIC comme enablePIC) ; valeur lue : {}", ratioFLIPvalue);
}

void PICDissipation::write(std::ostream& os) {
  os << "PICDissipation " << ratioFLIPvalue << ' ' << endTime << '\n';
}

void PICDissipation::check() {
  const bool active = (box->t < endTime);
  if (active) { box->ratioFLIP = ratioFLIPvalue; box->activePIC = true; }
  else        { box->activePIC = false; }
}
```

et surtout : **le documenter** dans le manuel, à côté de `enablePIC`. Renommer l'argument
serait plus clair mais casserait les fichiers existants.

---

## C10 — `MPTracking` : sélecteur exécuté à la lecture

**Fichier** : `Spies/MPTracking.cpp:19-28`

```cpp
void MPTracking::read(std::istream& is) {
  is >> nrec >> Filename >> MP_Selector;
  ...
  MP_Selector.execute(box);        // à la lecture de la ligne Spy
}
```

`MPMbox::read` traite les lignes dans l'ordre du fichier. Une ligne `Spy MPTracking …`
placée avant `set_MP_grid` sélectionne donc dans une liste vide : `MP_ids` reste vide,
`exec()` divise par `MP_ids.size() == 0` (résultat `NaN`, pas de plantage) et le fichier ne
contient que des `nan`. Aucun message.

`select_tracked_MP` et `select_controlled_MP` ont le même comportement, mais ce sont des
commandes : leur position dans le fichier est naturellement significative. Pour un spy, ça
l'est moins.

**Correction proposée** — différer la sélection au premier `exec()` :

```cpp
void MPTracking::read(std::istream& is) {
  is >> nrec >> Filename >> MP_Selector;
  nstep = nrec;
  filename = box->result_folder + fileTool::separator() + Filename;
  if (box->computationMode) file.open(filename.c_str());     // voir B12
}

void MPTracking::exec() {
  if (!selectionDone) {                       // nouveau membre bool
    MP_Selector.execute(box);
    selectionDone = true;
    if (MP_ids.empty()) Logger::warn("@MPTracking : aucun MP sélectionné");
  }
  if (MP_ids.empty()) return;
  ...
  meanStress *= (1.0 / (double)MP_ids.size());
  ...
}
```

Le manuel peut aussi simplement préciser que la ligne `Spy MPTracking` doit venir **après**
la création des points ; c'est la correction la moins intrusive.

---

## C11 — `MaterialPoint::nb` n'est pas unique

**Fichiers** : `Commands/set_MP_grid.cpp:25` et `:48`, `Commands/set_MP_polygon.cpp`,
`Commands/add_MP_ShallowPath.cpp`, `Core/MPMbox.cpp:1122`

```cpp
int counter = 0;                     // remis à zéro à chaque commande
...
P.nb = counter;  counter++;
```

Deux `set_MP_grid` successifs produisent deux séries `0, 1, 2, …`. `set_MP_polygon` et
`add_MP_ShallowPath` ne renseignent pas `nb` du tout (tous à 0). Le découpage adaptatif
duplique le numéro (voir **B11**).

`nb` est écrit dans les conf-files et relu, sans être utilisé ailleurs dans le calcul — les
indices utilisés partout (`Neighbor::PointNumber`, sélecteurs, `DEM_MP<p>/`) sont les
positions dans le tableau `MP`. Le champ n'est donc pas dangereux aujourd'hui, mais il est
inutilisable pour suivre un point dans le temps, ce à quoi son nom invite.

**Correction proposée** — un compteur porté par `MPMbox` :

```cpp
// MPMbox.hpp
size_t nextMPNumber{0};
```

```cpp
P.nb = box->nextMPNumber++;      // dans les quatre commandes de création et dans le split
```

et à la relecture, `nextMPNumber = 1 + max(MP[i].nb)`.

Alternative honnête si le suivi n'est pas nécessaire : retirer le champ et documenter que
les MP sont désignés par leur indice.

---

## ~~C12~~ — `set <nom>` accepte silencieusement n'importe quel nom

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichier** : `Core/MPMbox.cpp:388-393`

```cpp
} else if (token == "set") {
  std::string param;  size_t g1, g2;  double value;
  file >> param >> g1 >> g2 >> value;
  dataTable.set(param, g1, g2, value);
}
```

`DataTable::set(name, …)` appelle `add(name)`, qui **crée** le paramètre s'il n'existe pas.
Une faute de frappe (`set Kn 0 0 1e6`, `set viscrate …`) est donc acceptée sans un mot :
un nouveau paramètre inutilisé est créé et le vrai `kn` reste à 0. Une raideur de contact
nulle donne des forces nulles, donc des points qui traversent les obstacles — symptôme
difficile à relier à sa cause.

**Correction proposée** :

```cpp
} else if (token == "set") {
  std::string param;  size_t g1, g2;  double value;
  file >> param >> g1 >> g2 >> value;
  if (!dataTable.exists(param)) {
    Logger::critical("@MPMbox::read, paramètre d'interaction inconnu : 'set {} ...'. "
                     "Attendus : kn, kt, en2, mu, viscRate, dn0, dt0", param);
    exit(EXIT_FAILURE);
  }
  dataTable.set(param, g1, g2, value);
}
```

Les sept noms sont enregistrés par le constructeur de `MPMbox` (lignes 76-82), la
vérification est donc immédiate.

---

## ~~C13~~ — Pointeurs et membres non initialisés

> **CORRIGÉ le 2026-08-06** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichiers** : `ConstitutiveModels/ConstitutiveModel.hpp:11`, `Schedulers/Scheduler.hpp:7`,
`Core/MPMbox.hpp:140`, `Schedulers/ReactivateCHCLBonds.hpp:12-13`,
`Commands/set_node_grid.hpp`

```cpp
struct ConstitutiveModel { std::string key;  MPMbox *box; ... };   // box non initialisé
struct Scheduler         { MPMbox* box; ... };                      // idem
class  MPMbox { ... size_t number_MP_before_any_split; ... };       // non initialisé
struct ReactivateCHCLBonds { double bondingDistance; double timeBondReactivation; };
```

Tous sont affectés avant usage dans le déroulement actuel. Le cas de
`number_MP_before_any_split` mérite d'être détaillé, parce qu'il est instructif : il est
affecté en tête de `advanceOneStep` — dans les **trois** schémas d'intégration, ce qui est
déjà une duplication — donc à partir du pas 1 il vaut ce qu'il doit valoir. Au pas 0, il n'a
encore jamais été écrit, mais le `||` de `step % proxPeriod == 0` court-circuite avant de le
lire. Il n'y a donc pas de lecture indéterminée ; il n'y a qu'un invariant tenu par la
conjonction de trois fichiers et d'un ordre d'évaluation. C'est exactement le motif qui a produit le
bug `nbElemY` de `set_node_grid` corrigé récemment.

**Correction proposée** — initialiser systématiquement à la déclaration :

```cpp
MPMbox *box{nullptr};
size_t number_MP_before_any_split{0};
double bondingDistance{0.0};
double timeBondReactivation{0.0};
```

Et, tant qu'à faire, activer `-Weffc++` ou au moins relire les `.hpp` des familles de
plugins : c'est le point d'entrée de tout nouveau contributeur.

---

## ~~C14~~ — `Nodes` lu avant la grille

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

**Fichier** : `Core/MPMbox.cpp:491-501`

```cpp
} else if (token == "Nodes") {
  if (nodes.empty()) {
    Logger::warn("You need to set the nodes BEFORE reading their datasets ...");
  }                                        // et on continue
  size_t nb;  file >> nb;
  size_t in;
  for (size_t n = 0; n < nb; n++) {
    file >> in;
    file >> nodes[in].q >> ... ;           // nodes est vide, ou in >= nodes.size()
  }
}
```

Même schéma qu'en **A9** : avertissement sans interruption, puis indexation non vérifiée.
Un conf-file tronqué ou édité à la main suffit.

**Correction proposée** :

```cpp
} else if (token == "Nodes") {
  size_t nb;  file >> nb;
  if (nodes.empty()) {
    Logger::critical("@MPMbox::read, 'Nodes' avant la définition de la grille "
                     "(placer set_node_grid avant)");
    exit(EXIT_FAILURE);
  }
  for (size_t n = 0; n < nb; n++) {
    size_t in;  file >> in;
    if (in >= nodes.size()) {
      Logger::critical("@MPMbox::read, numéro de nœud {} hors grille ({} nœuds)", in, nodes.size());
      exit(EXIT_FAILURE);
    }
    file >> nodes[in].q >> nodes[in].f >> nodes[in].fb >> nodes[in].mass
         >> nodes[in].xfixed >> nodes[in].yfixed;
  }
}
```

Même remarque pour le bloc `Elem` (ligne 502), qui lit `element::nbNodes` depuis le fichier
et remplit `E.I[r]` sans vérifier que `r < 16` ni que les indices sont valides.

---

## C15 — `frictionalViscoElastofragile` : seuil non homogène, `sigma_n` toujours positif

**Fichier** : `BoundaryForceLaw/frictionalViscoElastofragile.cpp:37` et `:43`

```cpp
MPM.Obstacles[o]->Neighbors[nn].sigma_n = norm(MPM.MP[pn].stress * N);
...
double threshold = mu * MPM.Obstacles[o]->Neighbors[nn].sigma_n * MPM.MP[pn].vol0;
```

Deux réserves :

- `norm(...)` est la norme du vecteur contrainte : toujours ≥ 0. Le caractère « fragile »
  (perte de cohésion en traction) ne peut donc jamais se déclencher — il faudrait la
  **composante normale signée** `(stress · N) · N`, négative en compression avec la
  convention du code.
- `vol0` est une aire (m²) en 2D. Une force par unité d'épaisseur est le produit d'une
  contrainte par une **longueur**. Le seuil est donc homogène à `[force]·[longueur]`, pas à
  une force. La longueur attendue est vraisemblablement `sqrt(vol0)`, c'est-à-dire le côté
  du MP — qui est d'ailleurs déjà calculé deux lignes plus haut pour `gap`.

**Correction proposée** :

```cpp
gap = 0.5 * MPM.MP[pn].size;                       // = 0.5*sqrt(vol0), plus direct
...
const double sigN = (MPM.MP[pn].stress * N) * N;   // signé : < 0 en compression
MPM.Obstacles[o]->Neighbors[nn].sigma_n = sigN;
...
double threshold = std::max(0.0, -mu * sigN * MPM.MP[pn].size);
```

Cette loi n'est pas enregistrée dans `ExplicitRegistrations()` mais par un `Registrar`
statique en tête de fichier (ligne 11), contrairement aux deux autres. Le mécanisme
fonctionne (les objets sont explicitement listés sur la ligne d'édition de liens via
`$<TARGET_OBJECTS:core_obj>`), mais l'incohérence mérite d'être levée : soit tout par
`Registrar`, soit tout explicite.

---

# D — Mineur (cohérence, code mort, messages)

## D1 — Deuxième bloc `Nodes` mort

`Core/MPMbox.cpp:537-548` — un second `else if (token == "Nodes")` suit le premier (ligne
491) dans la même chaîne : il est inatteignable. Sa logique diffère pourtant (il vérifie
`nbNodes != nodes.size()` et lit les nœuds **dans l'ordre**, sans numéro). Il s'agit
vraisemblablement d'une ancienne version. **À supprimer** — ou à fusionner avec le premier
si la vérification de taille est jugée utile.

## D2 — `planeStrain` lu, sauvegardé, jamais utilisé

`Core/MPMbox.hpp:89`, `Core/MPMbox.cpp:340` et `:619`. Le drapeau existe et transite par les
conf-files, mais aucun modèle ne le consulte : l'hypothèse de déformation plane est câblée
en dur dans les modèles (`dstrain3x3.zz = 0`, `Finc3D.zz = 1`). Le mot-clé donne à croire
qu'un autre mode existe. **À ajouter à l'annexe B du manuel**, ou à retirer.

## D3 — `extremeShearing` : critère calculé, branche vide

`Core/MPMbox.cpp:1175-1181` :

```cpp
if (extremeShearing) {
  bool critExtremeShearing = (MP[p].F.xx / MP[p].F.xy < extremeShearingval || ...);
  if (critExtremeShearing) {
    // ... ???
  }
}
```

Le mot-clé `splittingExtremeShearing` est lu (ligne 370), le critère est évalué (avec une
division par `F.xy` potentiellement nul), et rien n'est fait. **À ajouter à l'annexe B**,
ou à supprimer avec le mot-clé.

## D4 — `VonMises` interpole `q/masse` là où les autres modèles utilisent `nodes[].vel`

`ConstitutiveModels/VonMisesElastoPlasticity.cpp:24-34`. Tous les autres modèles construisent
l'incrément de déformation à partir de `MPM.nodes[I[r]].vel`. `VonMises` recalcule
`vn = q/mass`. Dans `ModifiedLagrangian`, ces deux quantités sont **différentes** :
`nodes[].vel` est la vitesse re-projetée depuis les MP (le « re-mapping » du MUSL), `q/mass`
la vitesse issue du bilan de quantité de mouvement. Le modèle de Von Mises ne voit donc pas
le même champ de vitesse que les autres. Probablement involontaire.

## D5 — Commentaire de la matrice `De` faux

`ConstitutiveModels/MohrCoulomb.cpp:62-67` : le commentaire annonce
`f = Young / (1 + 2·Poisson)`, le code calcule `f = Young / ((1 + Poisson)·(1 − 2·Poisson))`.
**Le code est correct** (déformation plane), c'est le commentaire qu'il faut corriger.

## ~~D6~~ — `set_MP_grid` : message inversé, `exit(0)` sur une erreur

> **CORRIGÉ le 2026-08-05** — voir le journal en tête du document. Le texte ci-dessous décrit l'état d'avant correction.

`Commands/set_MP_grid.cpp:12-15` :

```cpp
if (box->Grid.lx / size < 2.0 || box->Grid.ly / size < 2.0) {
  Logger::warn("@set_MP_grid::exec, Check Grid size - MP size ratio (should not be more than 2)");
  exit(0);
}
```

Le test rejette un rapport **inférieur** à 2, le message dit « ne devrait pas être supérieur
à 2 ». Et `exit(0)` signale un succès au shell, ce qui trompe tout script d'enchaînement.
Même remarque pour les `exit(0)` de `set_node_grid.cpp:43`, `new_set_grid.cpp:10` et
`set_MP_grid.cpp:62`. Utiliser `Logger::critical` + `exit(EXIT_FAILURE)`.

## ~~D7~~ — `move_MP` : les coins sont traités comme des coordonnées locales

> **CORRIGÉ le 2026-08-06** : le champ `MaterialPoint::corner[4]` a été supprimé (voir `Doc/OPTIM.md`, § 4.3bis), et avec lui le bloc fautif.

`Commands/move_MP.cpp:45-54`. `corner[c]` contient des coordonnées **globales** (posées par
`updateCornersFromF`), mais la rotation leur est appliquée comme s'il s'agissait de
coordonnées relatives au centre, puis `pos` est ajouté. Les coins sont donc déplacés deux
fois. Le `FIXME` ligne 44 le signale. La commande écrase par ailleurs `F` par la rotation,
perdant toute déformation antérieure. Correction : `box->MP[p].updateCornersFromF();` après
la mise à jour de `pos` et `F`, et supprimer la boucle.

## D8 — Rayon du MP : deux conventions

`Obstacles/Circle.cpp:43` utilise `0.5 * sqrt(MP.vol)` (volume **courant**),
`Obstacles/Line.cpp:35` utilise `0.5 * MP.size` (taille **initiale**),
`Obstacles/Polygon.cpp:65` utilise `sqrt(MP.vol) / 2.0`,
`MPMbox::convergenceConditions` utilise `sqrt(vol/π)`.
Quatre définitions du « rayon » d'un point matériel dans le même code. À unifier — de
préférence sur `0.5 * size` mis à jour avec le volume, ou sur une méthode
`MaterialPoint::radius()`.

## D9 — `cut.cpp` : `max(norm(d1), norm(d1))`, `sprintf`

`See/cut.cpp:68` : `std::max(norm(d1), norm(d1))` — `d1` deux fois, `d2` est calculé mais
inutilisé (voir **A11**).
`See/cut.cpp:10`, `:33`, `:55` : `sprintf` dans des tampons de 256 octets avec un
`result_folder` de longueur arbitraire. Remplacer par `snprintf`, comme partout ailleurs
dans le code.

## D10 — `Neighbor::dt` jamais alimenté, `contactf` écrasé

`Core/Neighbor.hpp:14` : `dt` (déplacement tangentiel cumulé) est déclaré, sauvegardé dans
les conf-files, remis à zéro par les trois lois de contact — et jamais calculé. Soit
l'alimenter (`Neighbors[nn].dt += delta_dt`), soit le retirer du format.

`MPM.MP[pn].contactf = -f;` (les trois lois) : affectation et non accumulation. Un MP en
contact avec deux obstacles n'affiche que la force du dernier traité. Utiliser `+=` avec une
remise à zéro en début de pas, comme pour `MP[p].f`.

## D11 — `t += dt` : le dernier conf-file peut manquer

`Core/MPMbox.cpp:857` et `:812`. `t` est accumulé par additions successives ; après quelques
centaines de milliers de pas, l'erreur d'arrondi peut faire dépasser `finalTime` juste avant
le dernier multiple de `confPeriod`, et le conf-file final n'est pas écrit. C'est un
comportement déjà observé. Correction : `t = step * dt` quand `dt` est constant, ou
sauvegarde inconditionnelle après la boucle :

```cpp
}  // fin du while
save(iconf);   // état final, toujours
```

Attention : `dt` n'est pas constant en double échelle (`limitTimeStepForDEM`), donc
`t = step*dt` n'est pas généralisable — la sauvegarde finale inconditionnelle l'est.

## ~~D13~~ — `MaterialPoint::q` n'est utilisé nulle part

> **CORRIGÉ le 2026-08-06** : champ supprimé (voir `Doc/OPTIM.md`, § 4.3bis).

**Fichier** : `Core/MaterialPoint.hpp:36`

```cpp
vec2r q;               // Momentum (mass times velocity)
```

Aucun fichier du code ne lit ni n'écrit ce champ — vérifié sur l'ensemble des
sources. C'est un vestige : la quantité de mouvement est portée par les nœuds
(`node::q`), pas par les points matériels. Seize octets par point matériel, dans
une structure dont la taille est le facteur limitant à grande échelle
(`Doc/OPTIM.md`, § 3.4).

**Correction proposée** : le retirer. Il n'est ni sauvegardé dans les conf-files
ni affiché, la suppression est donc sans effet de bord.

---

## D14 — Un plantage sort avec le code de retour 0

**Fichier** : `Runners/run.cpp` (`mySigHandler`), `deps/toofus-src/stackTracer.hpp`

`mpmbox` installe un gestionnaire pour `SIGSEGV`, `SIGBUS`, `SIGFPE`, `SIGABRT` et
`SIGTERM`. Il imprime une trace de pile lisible — ce qui est précieux — puis rend la main
de telle sorte que le processus se termine avec le **code 0**.

Conséquence : rien de ce qui appelle `mpmbox` ne peut distinguer un calcul mené à son terme
d'un calcul qui a explosé au troisième pas. Ni un script d'enchaînement, ni un ordonnanceur
de calcul (SLURM, PBS…), ni `make`, ni la suite de tests — qui a d'ailleurs manqué un
plantage franc pour cette raison, jusqu'à ce qu'elle apprenne à repérer la trace de pile
dans la sortie plutôt que le code de retour.

**Correction proposée** — terminer sur le code conventionnel `128 + signal` :

```cpp
static void mySigHandler(int sig) {
  SimuChrono.stop();
  for (size_t s = 0; s < Simu.Spies.size(); ++s) { Simu.Spies[s]->end(); }
  StackTracer::defaultSigHandler(sig);   // imprime la trace
  std::_Exit(128 + sig);                 // ... et le fait savoir
}
```

`std::_Exit` plutôt que `exit` : on est dans un gestionnaire de signal, les destructeurs et
les `atexit` n'y sont pas sûrs. Une fois cela fait, `Tests/runtests.py` pourra revenir à un
simple test du code de retour (la détection par la trace de pile restera utile pour les
constructions sous sanitizer).

---

## D15 — Deuxième lecteur de `Nodes`, inatteignable

**Fichier** : `Core/MPMbox.cpp:594-605`

`MPMbox::read` contient **deux** branches `} else if (token == "Nodes") {` dans la même
chaîne de `if`/`else if`, sans accolade fermante entre les deux. La première (ligne 533) est
celle qui sert : elle lit un nombre d'enregistrements puis, pour chacun, un numéro de nœud
qu'elle vérifie — c'est la correction de **C14**. La seconde est **inatteignable**.

C'est aussi un vestige d'un format plus ancien : elle lit `nodes.size()` enregistrements sans
numéro de nœud, alors que `MPMbox::save` n'écrit que les nœuds non nuls, chacun précédé de son
numéro. Si elle était atteinte, elle désynchroniserait le flux.

**Correction proposée** : la supprimer. Vérifier au passage que son avertissement
(« cannot set the node-datasets if the grid has not been set ») n'apporte rien de plus que le
message de la branche vivante, qui arrête le calcul.

---

## D12 — Rappel des défauts déjà documentés

Ces trois points figurent déjà à l'annexe B du manuel utilisateur et ne sont pas repris
ci-dessus :

- `select_controlled_MP` — commande lue, liste remplie, aucun schéma d'intégration ne
  l'applique (le seul code correspondant est dans un `#if 0` de `ModifiedLagrangian.cpp:170`).
- `set dn0` / `set dt0` — présents dans la table d'interaction et dans les conf-files, lus
  par aucune loi de contact.
- `VtkOutputs/` — famille complète (17 fichiers), absente du `file(GLOB …)` de
  `CMakeLists.txt` et d'aucun mot-clé : jamais compilée.

Deux candidats à ajouter à cette annexe : `planeStrain` (**D2**) et
`splittingExtremeShearing` (**D3**).

### 2026-08-06 — A3, la couronne de nœuds fantômes

Un élément à 16 nœuds lit la couronne qui l'entoure : pour l'élément (i, j), les nœuds i-1 à
i+2 et j-1 à j+2. Sur la première et la dernière rangée d'éléments, cette couronne tombait
hors d'une grille qui n'a que Nx+1 par Ny+1 nœuds, et douze indices sur seize restaient à
zéro — toutes les fonctions de forme d'un point de bord étaient empilées sur le nœud 0.

**`grid::pad`** vaut désormais 1 dès que les éléments portent 16 nœuds, et 0 pour les
interpolations linéaires. Les nœuds portent des indices logiques allant de `-pad` à `Nx+pad`,
et `grid::nodeNumber(i, j)` est le seul endroit qui les traduit en numéros. **Les éléments ne
changent pas** : il y en a toujours Nx par Ny, numérotés `e = i + j*Nx`, couvrant le même
domaine. Rien en dehors de la numérotation des nœuds n'a à connaître les fantômes —
`locateElement` en particulier est inchangé.

Trois conséquences sur le reste du code :

- La construction de la grille, que `set_node_grid` et `new_set_grid` dupliquaient **mot pour
  mot**, cas particulier de bord compris, est factorisée dans `MPMbox::buildGrid()`. Les deux
  commandes se réduisent à poser `Grid.Nx/Ny/lx/ly` et à l'appeler.
- La numérotation des 16 nœuds n'est plus écrite qu'une fois, dans `element::dxOff` /
  `element::dyOff`. `buildGrid` remplit `element::I` avec, et `BSpline` lit ses fonctions de
  forme dans le même ordre : les deux **doivent** s'accorder, donc ils ne doivent pas être
  deux tables. Les quatre premières entrées sont exactement la disposition QUA4, si bien que
  la même table sert aux deux sortes d'élément.
- **Une condition limite qui atteint le bord de la grille est prolongée dans la couronne
  fantôme.** La ligne fixée est le bord physique du domaine, et une B-spline lit un nœud
  au-delà : le laisser libre reviendrait à laisser la matière traverser le mur qui la retient.
  Avec `pad = 0`, les boucles de `set_BC_line` et `set_BC_column` redeviennent exactement
  celles d'avant.

`BSpline::computeInterpolationValues` ne refuse donc plus les éléments de bord — ce refus,
introduit lors de la réécriture tensorielle, avait rendu A3 visible au lieu de silencieux. Il
ne reste qu'un garde-fou sur `Grid.pad`, pour le cas où la grille aurait été bâtie avant que
la fonction de forme ne soit connue (ce que les deux commandes refusent déjà).

**Test T30** : la grille étant régulière, un problème entier translaté d'un nombre entier de
mailles doit donner exactement le même résultat. Le même bloc est donc posé deux fois sur le
même obstacle, une fois contre le coin de la grille, une fois deux mailles plus loin ; les
déplacements coïncident à moins de 1e-12. Sans la couronne, le premier cas est refusé net.

**Vérifié sur `Examples/CantileverBeam`**, seul exemple livré qui pose des conditions limites,
et dont les deux `set_BC_column 3 0 6` et `4 0 6` touchent le haut et le bas de la grille :
résultat **identique au bit près** après 215 fichiers de configuration, pour une flèche de
3,5 mm en bout de poutre. Le prolongement dans la couronne ne change rien tant qu'aucune
matière n'atteint les nœuds concernés — il n'agit que là où il est nécessaire.

**Incompatibilité à connaître** : le nombre et la numérotation des nœuds changent pour les
calculs en B-splines. Un conf-file écrit avant cette correction reste lisible — la grille est
reconstruite depuis la commande qu'il contient — mais son bloc `Nodes` porte des numéros de
l'ancienne numérotation, et ses données (masse, quantité de mouvement, `xfixed`/`yfixed`)
tomberaient sur les mauvais nœuds. Cela n'affecte que la reprise et l'affichage des repères
de nœuds bloqués dans `see`, pas la position des points matériels. La reprise exacte est de
toute façon le chantier **C3**.

---

### 2026-08-06 — A7, A8, C13

**A7** — `RemoveMaterialPoint` compacte le tableau des points matériels, ce qui décale tous
les indices situés après ceux qu'il retire. Les listes de voisins des obstacles sont faites
de tels indices, et l'ordre dans `MPMbox::run` est `checkProximity()`, puis les
ordonnanceurs, puis `advanceOneStep()` : les listes sont construites avant le retrait et
utilisées après, **dans le même pas**. Le garde-fou du haut de `run`
(`MP.size() != number_MP_before_any_split`) ne peut rien y faire : `advanceOneStep` réaffecte
ce compteur à son entrée, donc après le retrait — il ne détecte que les *découpages*, qui ont
lieu plus tard dans le pas. Les listes sont désormais vidées et reconstruites par
l'ordonnanceur lui-même, juste après le compactage. L'échantillon DEM des points double
échelle retirés est libéré au passage (il n'était pas désalloué).

**A8** — `ReactivateCHCLBonds::check` appelait `PBC->ActivateBonds` sur **tous** les points ;
`PBC` est nul pour un point simple échelle. Un calcul sans loi homogénéisée où cet
ordonnanceur traîne plantait à `timeBondReactivation`. Les points sans échantillon DEM sont
maintenant ignorés.

**C13** — 65 membres de données initialisés à la déclaration dans 24 en-têtes : les `box`
des familles `Command` et `Scheduler`, les paramètres de tous les modèles de comportement,
des ordonnanceurs, des commandes, des obstacles et des espions. Aucun n'était lu avant
affectation dans le déroulement actuel, mais c'est exactement le motif qui a produit le bug
`nbElemY` de `set_node_grid`.

**Point de méthode.** Le dépassement de tableau d'A7 ne dure **qu'un seul pas** — les listes
sont reconstruites au pas suivant — et n'est pas observable de façon déterministe : il lit et
écrit dans le tas au-delà du tableau, ce qui ne plante pas à tout coup. Il a été mis en
évidence, puis vérifié corrigé, avec une construction sous **AddressSanitizer** :

```sh
cmake -S . -B BUILD-asan -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DCMAKE_CXX_FLAGS="-fsanitize=address -fno-omit-frame-pointer" \
      -DCMAKE_EXE_LINKER_FLAGS="-fsanitize=address"
cmake --build BUILD-asan --target mpmbox see -j8   # 'see' aussi : T19 le lance
python3 Tests/runtests.py --exe BUILD-asan/mpmbox
```

Avant correction, le cas **T28** produisait `AddressSanitizer: BUS ... READ memory access` dans
`Line::touch`, appelé depuis `advanceOneStep`. Après, la suite est propre. **Toute correction
portant sur des indices ou des durées de vie devrait être validée ainsi** : la suite ordinaire
ne voit pas ces défauts.

**Défaut découvert en écrivant les tests** — voir **D14** : `mpmbox` sort avec le **code 0**
quand il plante, son gestionnaire de signal masquant le code d'erreur. Le lanceur de tests
détecte donc désormais la trace de pile dans la sortie, et non le code de retour.

---

### 2026-08-05 — Optimisation : pistes 1 et 2 (et **A2**)

Deux corrections purement séquentielles, décrites et chiffrées dans
[`OPTIM.md`](OPTIM.md) : `BSpline` sur tableaux de pile plutôt que trois
`std::vector` par point et par pas, et liste des nœuds vivants construite par
marquage plutôt que par `std::set`. **×2,77 sur le cas de référence**, résultat
inchangé au bit près.

**A2** a été corrigé au passage, parce que la piste 1 l'imposait : la branche
d'erreur de `BSpline` n'écrivait rien, ce qui laissait les `std::vector` trop
courts — et aurait laissé, avec des tableaux fixes, des valeurs non
initialisées. Elle écrit maintenant des zéros et avertit une seule fois.
**A3**, la cause racine — les éléments de bord à 16 nœuds gardent leur anneau
extérieur à l'indice 0 — reste ouvert.

Le motif de la liste des nœuds vivants était écrit **quatre fois** : les trois
schémas d'intégration et `MPMbox::postProcess`. Il est désormais dans
`MPMbox::updateLiveNodeList()`.

---

# Tests de non-régression

Une suite de tests couvrant une partie de ces défauts est en place dans
[`../Tests/`](../Tests/README.md) et tourne en quelques secondes :

```bash
python3 Tests/runtests.py        # 26 PASS, 0 FAIL, 0 XFAIL au 2026-08-05
python3 Tests/runtests.py -v     # + la raison de chaque XFAIL
```

Elle distingue trois familles :

- **30 invariants** (`PASS`) — propriétés physiques et numériques qui doivent tenir
  **avant comme après** les corrections. Un `FAIL` est une régression.
- **plus aucun `XFAIL`** : tous les défauts couverts par la suite sont corrigés. Les entrées ci-dessous — chacun étiqueté avec son identifiant ci-dessous. Ils
  doivent basculer en `XPASS` au fur et à mesure des corrections, puis être reclassés en
  invariants.
- aucun test ne fige de valeur numérique de référence : chacun exhibe une propriété que le
  code **devrait** vérifier, de sorte qu'aucun bug ne se retrouve enregistré comme
  comportement attendu.

| Test | Défaut | État |
|---|---|---|
| T25 | ~~A1~~ | **corrigé** : `SIGBUS` → arrêt propre avec message |
| T20 | ~~A4~~ | **corrigé** : le nom du modèle est vérifié |
| T22 | ~~A5~~ | **corrigé** : les couples de groupes sont vérifiés avant le démarrage |
| T21 | ~~A6~~ | **corrigé** : le nom de la loi de contact est vérifié |
| T24 | ~~A9~~ | **corrigé** : les indices des conditions limites sont vérifiés |
| T23 | ~~A10~~ | **corrigé** : les périodes nulles sont refusées |
| T26 | ~~C12~~ | **corrigé** : un nom de paramètre `set` inconnu est refusé |
| T27 | ~~C14~~ | **corrigé** : `Nodes` avant la grille est refusé |
| T13 | ~~B1~~ | **corrigé** : le matériau se déforme à nouveau |
| T12 | ~~B2~~ | **corrigé** : `det F` reste à 4e-5 de 1 avec `UpdateStressFirst` |
| T11 | ~~B3~~ | **corrigé** : le travail du poids retrouve sa valeur à 0,1 % près |
| T09 | ~~B5~~ | **corrigé** : `dt` est ramené à la valeur analytique du critère CFL |
| T10 | ~~B6~~ | **corrigé** : une rampe plate laisse la gravité constante |
| T15 | ~~B7~~ | **corrigé** : l'écart entre `dt` et `dt/2` tombe à 0,06 % |
| T16 | ~~B9~~, ~~B10~~ | **corrigé** : la dérive ne dépend plus de `proxPeriod` (identique à 6 décimales) |
| T14 | ~~B11~~ | **corrigé** : numéros uniques, un seul découpage par pas |
| T17 | ~~B15~~ | **corrigé** : les trois schémas restent à moins de 2,3e-4 de det F = 1 |
| T19 | — | garde-fou : `see` doit pouvoir ouvrir un conf-file (ajouté après un `SIGSEGV`) |
| T28 | ~~A7~~ | **corrigé** : plus de dépassement sous AddressSanitizer après un retrait de points |
| T29 | ~~A8~~ | **corrigé** : un calcul simple échelle survit à `ReactivateCHCLBonds` |
| T30 | ~~A3~~ | **corrigé** : un bloc au coin de la grille se comporte comme le même bloc au milieu |

**T28 ne prouve son défaut que sous sanitizer.** Le dépassement d'A7 ne durait qu'un pas et
n'était pas observable autrement ; en construction ordinaire, T28 ne vérifie que le
comportement visible du retrait (les bons points partent, les bons restent). La marche à
suivre pour une construction ASan est dans le journal, à l'entrée du 2026-08-06.

Non couverts : **A3** (B-splines, éléments de bord), **B8**, **B12** à **B14**, toute la
classe C et tout le double échelle. `Tests/README.md` détaille ce qui
manque et pourquoi.

Deux invariants méritent d'être signalés parce qu'ils surveillent des corrections à venir :

- **T06** (reprise identique) attrapera toute variable d'état ajoutée et oubliée dans
  `save()` — c'est le garde-fou du chantier **C3**, et il se déclenchera notamment quand
  **B7** introduira un champ `viscousStress`.
- **T07** (convergence en `dt`) et **T05** (colonne élastique) encadrent les corrections
  **B5** et **B8**, qui vont changer des résultats.

---

# Suggestions d'ordre de traitement

Si l'objectif est de sécuriser le code avec un effort limité, l'ordre suivant maximise le
rapport bénéfice/risque :

1. ~~**A4, A6, A9, A10, C12, C14**~~ — validation des entrées. **Fait le 2026-08-05**, avec
   **A5** et **D6** dans la même passe.
2. ~~**A1**~~ — localisation de l'élément. **Fait le 2026-08-05.**
3. ~~**B9 + B10**~~ — historique de contact. **Fait le 2026-08-05.**
4. ~~**B3, B5, B6**~~ — **Fait le 2026-08-05.** B5 a bien réduit le pas de temps de trois
   exemples sur cinq (facteur 4 à 11) : voir le journal.
5. ~~**B1, B2, B11**~~ — **Fait le 2026-08-05.** A fait apparaître **B15**.
6. **B12 + C1** — mode visualisation et libération des ressources. Sans risque pour le
   calcul, corrige une perte de données. Non couvert (voir `Tests/README.md`).
7. **B8, C6** — modèles de comportement. Chaque correction change des résultats
   publiables : à valider sur un cas de référence avant d'être adoptée.
   ~~**B7**~~ **fait le 2026-08-05**, et **B15** avec lui.
8. **A2, A3** — B-splines. Chantier à part, à n'ouvrir que si la famille est utilisée.
9. **C3** — format de conf-file. Chantier à part, à traiter en une fois pour ne casser la
   compatibilité qu'une seule fois.

Les points **A7, A8, B13, B14, C2** sont indépendants et peuvent être pris à tout moment.

Deux tâches héritées de la passe du 2026-08-05 :

- corriger `Examples/BoulderImpact_NRJ/input.txt`, dont la loi de contact est assignée à un
  groupe d'obstacles inexistant (voir le journal en tête) ;
- **C1** est maintenant à portée de main pour les lois de contact : chaque obstacle
  possédant sa propre instance, `Obstacle::~Obstacle()` peut la libérer sans risque de
  double libération.
