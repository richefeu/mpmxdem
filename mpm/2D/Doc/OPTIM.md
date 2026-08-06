# MPMbox — où passe le temps, et par quoi commencer

Document de travail pour l'optimisation de `mpm/2D`. Il part de mesures, pas
d'intuitions : chaque piste a été **prototypée et chronométrée** avant d'être
proposée. Les gains annoncés sont donc des mesures, pas des estimations.

Le banc de mesure est [`../Tests/benchmark.py`](../Tests/benchmark.py) ; son mode
d'emploi est en fin de document.

> **Machine de référence** : Apple Silicon, 10 cœurs (4 performance),
> `g++-16 -O3 -fopenmp`. Les rapports entre phases se transposent ; les temps
> absolus, non.
>
> **État au 2026-08-05** : cinq pistes en place. **×3,2** sur le cas de
> référence `bspline`, **×1,13 de plus** sur le cas réaliste `bigspline`. Le § 1 décrit le profil de départ ; le § 3.2 celui du cas
> réellement utilisé (`BSpline` à l'échelle) ; le § 3.4 le facteur limitant qui
> reste, mesuré. **Le § 5 dit quoi faire ensuite.**
>
> `BSpline` est ce qui sert en pratique : les interpolations linéaires ne sont
> là que pour la forme et la pédagogie. Toute conclusion tirée d'un cas
> `Linear` est donc à reprendre sur `BSpline` — c'est ce qui s'est passé entre
> le § 3.1 et le § 3.2.

---

## 1. Le point de départ : la mesure

Cas `bspline` du banc — c'est `Examples/helloWorld` : 384 points matériels,
Mohr-Coulomb, B-splines (16 nœuds par élément), 11 000 pas.

| Phase du pas de temps | temps | part |
|---|---:|---:|
| **`shape functions`** | 1,97 s | **58,2 %** |
| **`live node list`** | 0,43 s | **12,7 %** |
| `updateStrainAndStress` | 0,54 s | 16,0 % |
| `G2P position` | 0,08 s | 2,4 % |
| `P2G mass momentum` | 0,08 s | 2,4 % |
| `updateMPVelocity` | 0,055 s | 1,6 % |
| `P2G velocity remap` | 0,055 s | 1,6 % |
| `P2G internal forces` | 0,054 s | 1,6 % |
| `contact forces` | 0,044 s | 1,3 % |
| `updateTransformationGradient` | 0,044 s | 1,3 % |
| `MP volume corners` | 0,016 s | 0,5 % |
| `grid reset` | 0,005 s | 0,1 % |
| `nodal update` | 0,002 s | 0,1 % |

**Deux phases pèsent 71 % du calcul, et aucune des deux n'est un problème de
parallélisme.** Ce sont des problèmes d'allocation mémoire.

La comparaison entre fonctions de forme le montre sans détour. Même cas, seule
la ligne `ShapeFunction` change :

| | pas complet | dont `shape functions` |
|---|---:|---:|
| `BSpline` (16 nœuds) | 15,95 s | 9,89 s |
| `Linear` (4 nœuds) | 2,86 s | 0,13 s |
| rapport | **5,6×** | **76×** |

B-spline fait 4 fois plus de travail arithmétique que Linear. Il coûte 76 fois
plus cher. Le facteur 19 qui manque, ce sont les allocations.

---

## 2. Les pistes

Les pistes 1 et 2 sont **en place depuis le 2026-08-05**. La piste 3 a été
prototypée, mesurée, puis retirée : elle attend la vérification demandée au § 3.

### Piste 1 — `BSpline` sans allocation — ✅ **faite** — gain mesuré **×8,8 sur la phase**

`BSpline::computeInterpolationValues` construit **trois `std::vector<double>`
par point matériel et par pas de temps**, remplis par `push_back` :

```cpp
std::vector<double> localCoord;
std::vector<double> Phi;
std::vector<double> PhiGrad;

for (int i = 0; i < 16; i++) {
  localCoord.push_back(...);   // 32 push_back, donc plusieurs reallocations
  localCoord.push_back(...);
}
```

Sur le cas `bspline` : 384 points × 11 000 pas × 3 vecteurs = **12,7 millions
d'allocations**, chacune suivie de réallocations au fil des `push_back`. Les
tailles sont pourtant connues à la compilation : 32 dans tous les cas.

```cpp
double localCoord[32];
double Phi[32];
double PhiGrad[32];

for (int i = 0; i < 16; i++) {
  localCoord[2 * i]     = (MPM.MP[p].pos.x - MPM.nodes[I[i]].pos.x) * invL[0];
  localCoord[2 * i + 1] = (MPM.MP[p].pos.y - MPM.nodes[I[i]].pos.y) * invL[1];
}
```

Une quinzaine de lignes touchées, aucun changement d'algorithme, résultat
numérique identique au bit près. `shape functions` passe de 1,97 s à 0,22 s.

**A2 a été corrigé en même temps** (`Doc/BUGS.md`) : la branche d'erreur
n'écrivait rien, ce qui laissait les vecteurs trop courts — et, avec des
tableaux fixes, aurait laissé des valeurs non initialisées. Elle écrit
maintenant des zéros et émet un avertissement, une seule fois. La cause
racine, **A3**, reste ouverte : les éléments de bord à 16 nœuds gardent leur
anneau extérieur à l'indice 0, et la partition de l'unité n'y est pas vérifiée.

### Piste 2 — liste des nœuds vivants par marquage — ✅ **faite** — gain mesuré **×11 à ×13 sur la phase**

À chaque pas, les trois schémas d'intégration reconstruisent la liste des nœuds
concernés par un point matériel :

```cpp
std::set<size_t> sortedLive;
for (p) for (r) sortedLive.insert(I[r]);
liveNodeNum.clear();
std::copy(sortedLive.begin(), sortedLive.end(), std::back_inserter(liveNodeNum));
```

Soit 384 × 16 = 6 144 insertions par pas dans un arbre rouge-noir, avec une
allocation par nœud inséré.

**Ce qui ne marche pas** : remplacer par `push_back` + `std::sort` + `std::unique`
ne gagne rien (mesuré : 0,93× à 1,17×, donc du bruit). Le tri coûte aussi cher
que l'arbre.

**Ce qui marche** : un tableau de marquage, indexé par numéro de nœud, avec un
compteur de pas comme marque. Ni tri, ni allocation, ni recherche.

Le motif était écrit **quatre fois** — les trois schémas d'intégration et
`MPMbox::postProcess`. Il est maintenant dans `MPMbox::updateLiveNodeList()`,
appelée par les quatre.

```cpp
// membres de MPMbox
std::vector<uint32_t> nodeStamp;   // dimensionné avec la grille
uint32_t stampTag{0};

// à chaque reconstruction
++stampTag;
liveNodeNum.clear();
for (p) for (r) {
  if (nodeStamp[I[r]] != stampTag) { nodeStamp[I[r]] = stampTag; liveNodeNum.push_back(I[r]); }
}
```

`live node list` passe de 0,43 s à 0,033 s sur `bspline`, de 0,11 s à 0,010 s sur
`linear`.

Deux points d'attention, tous deux traités :

- **La liste n'est plus triée.** Vérifié : aucune boucle n'en dépend pour le
  résultat — les nœuds y sont traités indépendamment, sans sommation croisée.
  L'ordre reste déterministe (celui des points matériels). Seule la localité des
  accès mémoire dans les boucles nodales en pâtit, et ces boucles pèsent 0,2 %.
- `nodeStamp` est redimensionné quand la grille change, et le compteur remis à
  zéro avec lui. Le débordement de `uint32_t` après 4 milliards de
  reconstructions réinitialise le tableau.

### Piste 3 — OpenMP sur la loi de comportement — ✅ **faite**, gain modeste et dépendant de la taille

Les pistes 1 et 2 étant en place, c'est `updateStrainAndStress` qui domine :
**44 % du pas** sur `bspline`, **62 %** sur `linear` et `dense`. La boucle est
déjà parallélisée pour les modèles double échelle ; il suffit de l'étendre au
cas simple échelle, une ligne dans `OneStep::updateStrainAndStress`.

Chaque itération n'écrit que dans `MP[p]` et ne lit que `nodes[]` : **aucune
course de données**.

**Vérification préalable, obligatoire** : les modèles de comportement sont
*partagés* entre tous les points qui s'y réfèrent. Chacun a été relu, y compris
les fonctions auxiliaires de `SinfoniettaClassica` et `Rigidity::getStress` :
**aucun n'écrit dans ses propres membres** pendant `updateStrainAndStress`. Tout
ce qui est affecté est local. La parallélisation sur `p` est donc sûre.

Gain mesuré sur 4 cœurs, en fonction du nombre de points :

| cas | points | 1 fil | 4 fils | gain |
|---|---:|---:|---:|---:|
| `linear` | 384 | 0,56 s | 0,62 s | **×0,90** |
| `dense` | 1 536 | 1,24 s | 0,92 s | **×1,35** |
| `big` | 32 000 | 2,19 s | 2,16 s | ×1,05 |

**Ce n'est pas la courbe attendue.** On espérait une efficacité qui monte avec la
taille ; on obtient un optimum au milieu. En dessous de quelques centaines de
points la boucle est trop courte et l'équipe de threads, créée à chaque pas,
coûte plus qu'elle ne rapporte. Au-delà de quelques milliers, le gain
s'effondre — et le § 3.1 explique pourquoi.

La directive est laissée sans seuil : avec un seul fil, qui est le défaut de la
ligne de commande, elle ne coûte rien de mesurable. Les 10 % perdus ne le sont
que si l'on demande explicitement des fils sur un cas de quelques centaines de
points, lequel tourne en une seconde de toute façon.

---

## 3. Bilan — état au 2026-08-05

Pistes 1 et 2 en place, mesuré sur la référence prise avant :

| cas | avant | après | gain |
|---|---:|---:|---:|
| `bspline` | 3,43 s | 1,24 s | **×2,77** |
| `linear` | 0,66 s | 0,56 s | ×1,18 |
| `dense` | 1,44 s | 1,23 s | ×1,18 |
| `usf` | 0,65 s | 0,54 s | ×1,20 |

**Le résultat n'a pas changé d'un bit** : le banc compare huit scalaires de
l'état final et ne signale rien. Les 26 tests de `runtests.py` passent, et les
cinq exemples tournent sans message.

**Le message principal : la question posée était « où mettre OpenMP ? », la
réponse mesurée est « nulle part, pour l'instant ».** Deux corrections
séquentielles, sans le moindre risque de course de données, donnent ×2,77 sur le
cas de référence — davantage que ce qu'on peut espérer d'OpenMP sur 4 cœurs.

Le profil est maintenant celui-ci sur `bspline` :

| Phase | part |
|---|---:|
| `updateStrainAndStress` | **44 %** |
| `shape functions` | 18 % |
| `G2P position` | 7 % |
| `P2G mass momentum` | 7 % |
| `live node list` | 3 % |
| le reste | 21 % |

Sur `linear` et `dense`, la loi de comportement pèse **62 %**. C'est désormais
la seule phase qui vaille la peine d'être parallélisée, et c'est la plus simple
à traiter : aucune course de données.

### 3.1 À grande échelle, le facteur limitant est la mémoire, pas le calcul

C'est le résultat le plus important de cette campagne, et il n'était pas prévu.

Le cas `big` — 32 000 points, 4 par maille — donne un profil **complètement
plat** : aucune phase ne dépasse 15 %.

| Phase | part | | Phase | part |
|---|---:|---|---|---:|
| `updateStrainAndStress` | 14,5 % | | `P2G internal forces` | 7,8 % |
| `P2G mass momentum` | 13,7 % | | `P2G velocity remap` | 7,0 % |
| `updateTransformationGradient` | 12,8 % | | `contact forces` | 6,5 % |
| `shape functions` | 11,2 % | | `updateMPVelocity` | 5,7 % |
| `MP volume corners` | 8,5 % | | `live node list` | 3,2 % |
| `G2P position` | 7,9 % | | `grid reset` | 1,0 % |

Toutes les phases parcourent le même tableau de points matériels, et coûtent en
proportion. Le coût par point, mesuré à densité constante (4 points par maille,
`Linear`), le confirme :

| points | tableau des MP | µs/pas/point |
|---:|---:|---:|
| 1 600 | 1,4 Mo | 0,179 |
| 6 400 | 5,8 Mo | 0,145 |
| 18 496 | 16,6 Mo | 0,156 |
| **32 000** | **28,8 Mo** | **0,247** |

Plat jusqu'à ~17 Mo, puis **+58 %**. C'est la falaise de cache : le L2 partagé
de la machine fait environ 16 Mo. Passé ce point le calcul est limité par la
bande passante mémoire, et ajouter des cœurs n'y change rien — d'où le ×1,05 de
la piste 3 sur ce cas.

**Le levier, à cette échelle, est donc la taille de `MaterialPoint`, pas le
parallélisme.** Et il y a de quoi faire : `double N[16]` et `vec2r gradN[16]`
occupent 384 octets, soit **43 % de la structure** — alors que `Linear` et
`RegularQuadLinear` n'utilisent que 4 emplacements sur 16, c'est-à-dire 96
octets. Les 288 octets gaspillés par point représentent **9,2 Mo sur les 28,8 du
cas `big`** : les récupérer ramènerait le tableau à 19,6 Mo, sous la falaise.

Ce n'est pas gratuit : 86 sites accèdent à `.N[` ou `.gradN[`. Deux formes
possibles, décrites au § 4.3.

---

### 3.2 Deuxième campagne : le cas réellement utilisé

Toute l'étude d'échelle du § 3.1 avait été menée avec `Linear`. Or **`BSpline`
est ce qui sert en pratique** — les interpolations linéaires ne sont là que pour
la forme et la pédagogie. Refaite sur `bigspline` (32 000 points *et*
B-splines), la répartition est tout autre :

| Phase | part | | Phase | part |
|---|---:|---|---|---:|
| **`shape functions`** | **18,5 %** | | `updateTransformationGradient` | 7,1 % |
| `G2P position` | 12,0 % | | `P2G mass momentum` | 7,1 % |
| `P2G velocity remap` | 11,2 % | | `live node list` | 5,5 % |
| `contact forces` | 11,0 % | | **`updateStrainAndStress`** | **5,7 %** |
| `updateMPVelocity` | 10,5 % | | `P2G internal forces` | 5,7 % |
| | | | `MP volume corners` | 4,9 % |

**La loi de comportement ne pèse plus que 5,7 %.** La piste 3, qui la
parallélise, ne peut donc rien apporter sur le cas réel : c'est une correction
de propreté, pas une optimisation. Les six boucles qui parcourent les 16 nœuds
de chaque point pèsent ensemble **54 %**.

### Piste 4 — B-spline tensoriel — ✅ **faite** — ×2,9 sur la phase

Le B-spline est un produit tensoriel, $N_k = \Phi(x_k)\,\Phi(y_k)$, et les 16
nœuds du support ne prennent que **quatre positions distinctes dans chaque
direction** : les décalages valent tous $-1$, $0$, $1$ ou $2$. Il n'y a donc que
$4+4 = 8$ facteurs distincts, là où le code en évaluait 32 — une fois par nœud
et par direction, avec un branchement à chaque fois.

Deux conséquences :

- 8 évaluations du polynôme cubique au lieu de 32 ;
- le décalage décide de la branche ($|x| \in [1,2]$ pour $-1$ et $2$,
  $[0,1]$ pour $0$ et $1$), donc plus aucun branchement imprévisible. Les deux
  expressions coïncidant en $|x| = 1$, le cas limite est sans danger.

Les coordonnées réduites sont en outre calculées à partir des indices
d'élément plutôt que lues dans les positions nodales — la grille est régulière
par construction — ce qui économise 16 lectures dispersées d'un nœud de
156 octets par point et par pas.

| | avant | après |
|---|---:|---:|
| `shape functions` (cas `bigspline`) | 0,279 s | **0,095 s** |
| part du pas | 18,5 % | 6,9 % |
| cas `bspline` (384 points) | 1,24 s | **1,08 s** (×1,15) |
| cas `bigspline` (32 000 points) | 1,99 s | 1,86 s (×1,07) |

Résultat inchangé au bit près.

**Effet de bord assumé** : les coordonnées réduites étant désormais calculées et
non lues, la branche « hors du support » n'existe plus, et un élément de bord
— dont l'anneau extérieur de nœuds n'existe pas, défaut **A3** — produirait
seize contributions correctes empilées sur le nœud 0, silencieusement. Un
contrôle explicite a donc été ajouté : `BSpline` **refuse** de traiter un point
situé dans un élément de bord, avec un message qui dit quoi faire. Les huit
exemples livrés utilisent tous `BSpline` et aucun ne déclenche ce contrôle.

### 3.3 Ce qui a été testé et écarté

Deux hypothèses ont été mesurées puis abandonnées. Elles valent d'être
consignées : elles évitent de refaire le chemin.

- **Lire les positions nodales coûte cher.** Faux. Les supprimer du calcul des
  coordonnées locales, seules, n'a rien changé (0,280 s → 0,279 s). Le gain de
  la piste 4 vient de la structure tensorielle, pas de là.
- **La taille de `node` (156 octets) est limitante.** Faux. Ajouter 40 octets de
  bourrage à `node` coûte 0,5 %, c'est-à-dire rien. Le tableau des nœuds fait
  3 Mo et tient confortablement en cache. Retirer `stress` et
  `outOfPlaneStress`, que le pas de temps n'utilise pas, ne rapporterait donc
  rien — c'est une question de propreté, pas de performance.

### 3.4 Le facteur limitant, mesuré : la taille de `MaterialPoint`

L'expérience symétrique, elle, est sans appel. En ajoutant du bourrage à
`MaterialPoint` sur le cas `bigspline` :

| `MaterialPoint` | tableau | temps | µs/pas/point |
|---:|---:|---:|---:|
| ~900 o | 28,8 Mo | 1,92 s | 0,400 |
| +96 o | 31,9 Mo | 2,45 s | **0,510** |
| +200 o | 35,2 Mo | 2,53 s | 0,526 |

**+11 % de taille coûtent +28 % de temps.** La pente est brutale, et elle
confirme le § 3.1 : au-delà de ~17 Mo, le tableau des points matériels ne tient
plus en cache et c'est lui qui commande tout.

Par symétrie, **en retirer** doit rapporter au moins autant. Et il y a de quoi :
`double N[16]` et `vec2r gradN[16]` occupent 384 octets, soit **43 % de la
structure**. Les sortir ramènerait `MaterialPoint` à ~516 octets et le tableau
de 28,8 à **16,5 Mo — sous la falaise**.

Contrairement à ce que supposait la première version de ce document, l'intérêt
n'est pas de récupérer des emplacements inutilisés : avec `BSpline`, les seize
servent. Il est de **séparer une donnée parcourue en flux d'une structure
parcourue de façon éparse**. `N` et `gradN` deviennent deux tableaux contigus,
lus séquentiellement dans chaque boucle, tandis que le reste du point matériel
tient enfin en cache.

C'est la prochaine chose à faire, et de loin la plus rentable. Voir § 4.3.

---

### 3.5 Ne pas calculer ce que personne ne lit

Une fois les allocations et la redondance traitées, le relevé champ par champ de
`MaterialPoint` montre autre chose : **le pas de temps entretient des données que
la configuration courante n'utilise pas.**

| Champ | taille | lu par |
|---|---:|---|
| `N[16]` + `gradN[16]` | 384 o | toutes les boucles |
| `corner[4]` | 64 o | `Polygon` seulement — `see` les recalcule |
| `plasticStrain` | 32 o | lois plastiques |
| `stressCorrection` | 32 o | `MohrCoulomb`, `VonMises` |
| `prev_F` | 32 o | `CHCL_DEM` seulement |
| `viscousStress` (+ hors plan) | 40 o | `KelvinVoigt` seulement |
| `hardeningForce`, `outOfPlaneEp` | 16 o | `SinfoniettaClassica`/`Crush` |
| `q` | 16 o | **personne** |

Deux de ces entretiens étaient inconditionnels et ont été supprimés :

- **`prev_F`** était recopié à chaque pas — 32 octets écrits par point — alors que
  seul `CHCL_DEM` le lit. Il n'est plus mis à jour qu'en double échelle.
- **`corner[4]`** était rafraîchi à chaque pas par les trois schémas : quatre
  produits matrice-vecteur et 64 octets écrits par point. Seul
  `Polygon::getContactFrame` les lit pendant le pas ; `MPMbox::postProcess` les
  recalcule pour l'affichage. `MPMbox::checkSettings` détecte maintenant la
  présence d'un obstacle `Polygon` et positionne `needMPCorners` en conséquence.

Mesuré sur `bigspline` : la phase `MP volume corners` passe de 0,074 à 0,041 s,
`updateTransformationGradient` de 0,107 à 0,101 s, l'ensemble de 1,86 à 1,83 s
(**×1,02**). C'est modeste, mais gratuit et correct : on ne calcule plus ce que
personne ne lit.

`MaterialPoint::q`, la quantité de mouvement du point, **n'est utilisée nulle
part** : champ mort, à retirer (voir D13 dans `Doc/BUGS.md`).

**Ce que ce tableau dit surtout** : 616 octets sur ~976, soit **63 % de
`MaterialPoint`**, ne servent qu'à une configuration particulière. Un calcul
`HookeElasticity` sans obstacle `Polygon` et sans double échelle traîne 184
octets par point qu'il n'ouvrira jamais — et, d'après le § 3.4, chaque tranche
de 96 octets coûte 28 %.

---

### 3.6 Localité des données : ce que la mesure dit

Trois questions de proximité mémoire ont été posées et tranchées par
l'expérience, chacune par un A/B entrelacé sur le cas `bigspline`.

**L'ordre des points matériels compte beaucoup — et il est déjà optimal.**
Mélanger le tableau des points au démarrage coûte **+27 %** (1,93 s → 2,46 s) :
la localité de l'ordre est donc bien un facteur de premier plan. Mais il ne
reste rien à y gagner, car cet ordre **ne se dégrade pas** :

| conf-file | temps | saut moyen d'indice d'élément | paires voisines |
|---|---:|---:|---:|
| `conf0` | 0,000 s | 2,0 | 100 % |
| `conf11` | 0,051 s | 2,0 | 100 % |
| `conf21` | 0,097 s | 2,2 | 100 % |

Les points sont créés rangée par rangée, dans l'ordre même de la numérotation
des nœuds, et ils se déplacent lentement devant la taille d'une maille. Sur
`helloWorld`, qui s'écoule pourtant franchement, deux points consécutifs restent
dans des éléments voisins **dans 100 % des cas**. Un tri spatial périodique — la
parade classique en MPM — n'aurait donc rien à trier.

*Réserve* : mesuré sur 0,1 s d'écoulement. Un effondrement long et violent
mériterait de refaire ce relevé ; la métrique ci-dessus, calculée depuis
n'importe quel conf-file, suffit à le vérifier.

**La fusion des deux passes P2G ne rapporte que 2 à 3 %.** `P2G mass momentum`
et `P2G internal forces` parcourent les mêmes points, les mêmes `I[16]` et les
mêmes nœuds ; les fusionner évite un parcours du tableau des points, soit
19 Mo. Mesuré : ×1,02 sur le meilleur tour, ×1,13 sur un tour bruité — donc
**dans le bruit**. L'ordre de grandeur attendu est cohérent : un parcours de
19 Mo coûte environ 3 % du pas. Non retenu : le gain ne paie pas la lisibilité
perdue, et il faudrait une machine au repos pour trancher.

**La taille des nœuds ne compte toujours pas.** L'expérience du bourrage
(+40 octets sur `node`) avait déjà donné 0,5 %, c'est-à-dire rien. Le tableau
des nœuds fait 3 Mo et tient confortablement en cache, quel que soit le nombre
de points matériels.

**Ce qui reste, donc, est la taille de la structure, pas sa disposition.** Le
§ 3.4 le mesure : chaque tranche de 96 octets retirée à `MaterialPoint` vaut
plusieurs pourcent. C'est le § 4.3bis, et il reste 232 octets conditionnels à
sortir.

---

## 4. Ce qui reste à explorer

Ces pistes n'ont **pas** été mesurées. Elles sont classées par rapport
bénéfice/risque estimé.

### 4.1 Parallélisme — ce qui est sûr et ce qui ne l'est pas

En MPM, les boucles se rangent en deux familles, et la distinction décide de
tout :

**Rassemblement (nœuds → points), sans course de données.** Chaque itération
n'écrit que dans `MP[p]`. Parallélisation directe :

- `computeInterpolationValues` (la boucle appelante, pas la fonction)
- `updateStrainAndStress` (piste 3)
- `updateMPVelocity`
- `G2P position`
- `updateVelocityGradient`, `updateTransformationGradient`
- `MP volume corners`

Ces phases pèsent ensemble environ 25 % du pas après les pistes 1 et 2.

**Dispersion (points → nœuds), avec courses de données.** Plusieurs points
écrivent dans le même nœud : `nodes[I[r]].mass += …`. Trois solutions
classiques, par ordre de complexité :

1. **`#pragma omp atomic`** sur chaque accumulation. Simple, mais le coût des
   opérations atomiques sur 4 à 8 accumulations par nœud et par point risque
   d'annuler le gain. À mesurer avant d'y croire.
2. **Tampons nodaux par thread**, puis réduction. Coût mémoire :
   `nbThreads × nbNodes × taille_du_nœud`. Pour une grille de 1 271 nœuds c'est
   négligeable ; pour une grille fine, moins.
3. **Coloriage** : partitionner les points matériels en groupes tels que deux
   points d'un même groupe ne partagent aucun nœud. Le plus efficace, le plus
   lourd à écrire, et à refaire quand les points bougent.

Ces phases pèsent environ 10 % du pas. **Le jeu n'en vaut probablement pas la
chandelle** tant que la loi de comportement n'est pas parallélisée.

### 4.2 Une seule région parallèle par pas

Si plusieurs boucles sont parallélisées, il vaut mieux ouvrir **une** région
`#pragma omp parallel` pour tout le pas et y placer des `#pragma omp for
nowait` que d'ouvrir et fermer une équipe de threads pour chaque boucle. C'est
précisément le surcoût suspecté au § 2, piste 3.

### 4.3 Sortir `N` et `gradN` de `MaterialPoint` — ✅ **fait le 2026-08-05**, ×1,13

**Mesuré** (§ 3.4) : +11 % sur la taille de `MaterialPoint` coûtent +28 % de
temps. C'était le facteur limitant à l'échelle.

**Résultat obtenu** : `sizeof(MaterialPoint)` passe de **984 à 600 octets**
(−39 %), pour un gain de **×1,13** sur le cas `bigspline`. Les petits cas sont
inchangés (1,00 à 1,02×) : ils tenaient déjà en cache. Résultat identique au bit
près, 27 tests au vert.

> **Correction.** Ce gain avait d'abord été annoncé à ×1,24, à partir de deux
> mesures séparées dans le temps. Reprise en **A/B entrelacé** — trois tours
> alternant la version courante et une version portant 384 octets de bourrage
> pour retrouver l'empreinte d'avant — la valeur est ×1,13 (2,14 s contre
> 1,89 s). Entre les deux mesures, la ligne de base du même binaire avait dérivé
> de 1,48 à 1,90 s. **Sur cette machine, toute comparaison doit être entrelacée**,
> les répétitions du banc ne suffisent pas à absorber une dérive de cette
> ampleur.

Deux points de mise en œuvre valent d'être notés :

- Les accès passent par `MPMbox::N(p)` et `MPMbox::gradN(p)`, qui rendent un
  pointeur sur les `nbNodes` valeurs du point. **Il faut les sortir de la boucle
  interne** : laissés dedans, ils recalculent `shapeN.data() + p * nbNodes` à
  chaque accès, et le compilateur ne peut pas les hisser puisque l'écriture dans
  `nodes[...]` pourrait, formellement, aliaser `shapeN`. Mesuré : sans le
  hissage, `P2G internal forces` était 74 % plus lent qu'avant le changement.
  Les 26 boucles concernées commencent donc par
  `const vec2r *gNp = MPM.gradN(p);`.
- `MPMbox::resizeShapeArrays()` est appelée au début de chaque pas et de
  `postProcess`. Le test ne coûte rien et les tableaux ne grandissent que
  lorsque des points sont créés.

Le tableau total occupe toujours 31,5 Mo, mais il est désormais **coupé en
deux** : 19,2 Mo de structures parcourues de façon éparse, et 12,3 Mo de
fonctions de forme lues en flux séquentiel. C'est ce découpage qui paie, pas une
réduction du volume global.

`double N[16]` (128 o) et `vec2r gradN[16]` (256 o) font 384 octets, soit 43 %
de la structure. Les déplacer dans deux tableaux portés par `MPMbox` :

```cpp
std::vector<double> shapeN;      // taille MP.size() * element::nbNodes
std::vector<vec2r>  shapeGradN;
```

`MP[p].N[r]` devient `MPM.shapeN[p * nbNodes + r]`.

| | avant | après (mesuré) |
|---|---:|---:|
| `sizeof(MaterialPoint)` | 984 o | **600 o** |
| tableau des points (32 000) | 31,5 Mo | **19,2 Mo** |
| `N`/`gradN` | dispersés dans la structure | 12,3 Mo contigus, lus en flux |
| cas `bigspline` | — | **×1,13** (A/B entrelacé) |

Deux effets se cumulent : le tableau des points repasse sous la falaise du L2,
et les fonctions de forme deviennent un flux séquentiel parfait pour le
préchargeur.

93 accès répartis sur 15 fichiers, tous de la forme `MP[p].N[r]` : la
transformation a été mécanique. Ni le format des conf-files ni `see` n'étaient
concernés — ni `N` ni `gradN` n'y figurent.

### 4.3bis Aller plus loin : structure de tableaux

Si le § 4.3 ne suffit pas, l'étape d'après est de traiter de la même façon les
autres champs chauds (`pos`, `vel`, `mass`, `stress`), c'est-à-dire de passer
d'un tableau de structures à une structure de tableaux. Gain potentiel
supérieur, vectorisation possible, mais refonte de tout le code. À ne pas
entreprendre avant d'avoir mesuré ce que le § 4.3 donne.

### 4.4 `element::nbNodes` est une variable statique

Elle est lue dans **chaque boucle interne** du code. Le compilateur ne peut ni
la mettre en registre de façon sûre ni dérouler les boucles, puisqu'il doit
supposer qu'un appel de fonction peut la modifier. La copier dans une variable
locale `const size_t nbNodes = element::nbNodes;` en tête de chaque fonction est
gratuit et peut débloquer le déroulage. À mesurer.

### 4.5 Points divers

- `Obstacle::checkProximity` copie tout le vecteur des voisins (`std::vector<Neighbor> Store = Neighbors;`)
  à chaque reconstruction. Un `std::swap` suffirait. La phase pèse 0,07 % : à ne
  faire que par propreté.
- `convergenceConditions` construit deux `std::set<int>` par pas de temps pour
  des groupes qui ne changent jamais. Ils peuvent être calculés une fois pour
  toutes. 0,4 % du temps.
- `MPMbox::run` appelle `convergenceConditions()` à **chaque pas** alors que le
  résultat ne bouge quasiment pas ; une évaluation tous les `proxPeriod` pas
  suffirait largement.

---

## 5. Ordre de traitement

**Fait le 2026-08-05**, dans cet ordre :

1. ~~Piste 1~~ — `BSpline` sans allocation. ×8,8 sur la phase, avec **A2**.
2. ~~Piste 2~~ — nœuds vivants par marquage, dans `MPMbox::updateLiveNodeList`.
   ×11 à ×13 sur la phase.
3. ~~Piste 3~~ — OpenMP sur la loi de comportement. ×1,35 au mieux, mais **5,7 %
   du pas seulement sur le cas réel** : correction de propreté plus
   qu'optimisation.
4. ~~Piste 4~~ — B-spline tensoriel. ×2,9 sur la phase.

Cumul sur `bspline` : **3,43 s → 1,08 s, soit ×3,2**.

**À faire ensuite**, par ordre de rentabilité mesurée :

5. ~~**Sortir `N` et `gradN` de `MaterialPoint`**~~ — **fait le 2026-08-05**,
   ×1,24 sur le cas réaliste, structure ramenée de 984 à 600 octets.
6. **Sortir les 232 octets conditionnels du § 3.5** — `corner[4]`, `prev_F`,
   `viscousStress`, `stressCorrection`, `hardeningForce` — et retirer `q`, qui
   est mort (D13). La structure tomberait à ~360 octets. Même geste que le
   § 4.3, mais chaque champ demande de décider où le loger : un tableau annexe
   alloué seulement si le modèle ou l'obstacle concerné est présent.
6. Re-mesurer. Si le tableau des points repasse sous la falaise, le profil
   changera encore et il faudra refaire le § 3.2.
7. Fusionner les boucles qui se suivent et parcourent les mêmes données —
   `P2G mass momentum` + `P2G internal forces`, `G2P position` + `MP volume
   corners`. Économie estimée de 5 %, à confirmer.
8. Structure de tableaux (§ 4.3bis) seulement si le reste ne suffit pas.

**Ce qui n'est pas à faire** : chercher du parallélisme ailleurs. Le § 3.2
montre qu'aucune phase ne dépasse 18 % sur le cas réel, et le § 3.4 que la
limite est la bande passante mémoire. Les boucles de dispersion (§ 4.1), avec
leurs courses de données et leur coloriage, ne rapporteraient rien.

## 6. Le banc de mesure

```bash
python3 Tests/benchmark.py --save          # référence, AVANT de modifier le code
python3 Tests/benchmark.py                 # mesure et compare
python3 Tests/benchmark.py --phases        # détail par phase du pas de temps
python3 Tests/benchmark.py --threads 1,2,4 # passage à l'échelle OpenMP
python3 Tests/benchmark.py -k bspline -n 5 # un cas, cinq répétitions
python3 Tests/benchmark.py -k bigspline    # le cas realiste, exclu par défaut
```

Quatre cas courants, dérivés de `Examples/helloWorld`, environ 18 s au total,
plus deux cas lourds qu'il faut demander nommément :

| cas | ce qu'il expose |
|---|---|
| `bspline` | le cas d'origine ; les fonctions de forme dominent |
| `linear` | 4 nœuds par élément ; le coût bascule sur la loi de comportement |
| `dense` | 1536 points sur la même grille ; suit le coût en nombre de points |
| `usf` | un autre schéma d'intégration ; vérifie qu'une optimisation faite dans les parties communes profite bien aux trois |
| `bigspline` | **le cas réaliste** : 32 000 points *et* B-splines. C'est celui sur lequel juger une optimisation destinée à la production |
| `big` | le même avec `Linear`, pour isoler ce qui vient du nombre de nœuds par élément |

Trois précautions y sont câblées, chacune apprise à ses dépens :

- **Le pas de temps est fixé** sous la limite CFL, pour que
  `convergenceConditions` ne l'ajuste pas. Sans cela le nombre de pas dépend de
  la version du code et les temps ne sont plus comparables.
- **Trois répétitions par défaut**, la plus rapide retenue. Une mesure unique
  s'est trompée de 40 % pendant la rédaction de ce document, et a bien failli y
  faire figurer une accélération inexistante.
- **Comparer en A/B entrelacé, jamais deux mesures séparées dans le temps.**
  Les répétitions absorbent le bruit court ; elles n'absorbent pas la dérive de
  la machine. La ligne de base d'un même binaire est passée de 1,48 à 1,90 s en
  une demi-heure, ce qui a suffi à faire annoncer un ×1,24 là où l'A/B donne
  ×1,13. La bonne méthode : alterner les deux versions plusieurs fois et
  comparer les meilleurs temps de chaque.
- **Le résultat est comparé, pas seulement le temps.** Le banc résume l'état
  final par huit scalaires (centre de masse, énergie cinétique, contraintes
  moyennes…) et signale tout écart relatif supérieur à 1e-9. *Une optimisation
  qui change le résultat n'est pas une optimisation.* Le code de retour est 1
  dans ce cas.

Le banc ne remplace pas [`Tests/runtests.py`](../Tests/README.md), qui vérifie
la physique. Les deux sont à passer après chaque modification.
