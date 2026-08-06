# MPMbox (mpm/2D) — bilan des gains

Synthèse de la campagne menée les 5 et 6 août 2026 sur `mpm/2D`. Le détail est ailleurs :
`Doc/BUGS.md` pour la revue de défauts et le journal des corrections, `Doc/OPTIM.md` pour le
profil du pas de temps et les pistes d'optimisation mesurées. Ce document ne dit que ce qui a
été gagné, et à quel prix.

Point de départ : commit `78157f8a5`. Point d'arrivée : l'arbre de travail au 6 août 2026.

---

## 1. Vitesse

Mesure **A/B entrelacée** — les deux binaires tournent alternativement, dans le même tour de
boucle, trois tours, médiane retenue. C'est une précaution qui a sa raison d'être : sur cette
machine, la ligne de base d'un même binaire a dérivé de 1,48 à 1,90 s en une demi-heure, ce
qui avait fait annoncer un ×1,24 là où l'A/B donnait ×1,13.

Les temps sont normalisés **par pas de calcul**, et non par exécution. La raison est au § 2.

| Cas | Ce qu'il exerce | µs/pas — origine | µs/pas — actuel | Gain |
|---|---|---:|---:|---:|
| `bspline` | B-splines, 384 points | 318,2 | 100,4 | **×3,17** |
| `linear` | interpolation linéaire, 384 points | 60,8 | 50,5 | ×1,20 |
| `dense` | 1536 points sur la même grille | 264,6 | 205,8 | ×1,29 |
| `usf` | schéma `UpdateStressFirst` | 69,6 | 48,2 | ×1,44 |

**Moyenne géométrique : ×1,63.** Le chiffre qui compte en pratique est celui de `bspline` :
c'est l'interpolation réellement utilisée, les linéaires n'étant là que pour la forme et la
pédagogie. Sur le cas de production, le pas de calcul est **trois fois plus court**.

`sizeof(MaterialPoint)` est passé de **944 à 432 octets** (−54 %), mesuré avec le même
compilateur des deux côtés. La structure est même passée par 984 octets en cours de route :
la correction de **B7** avait dû y ajouter un champ `viscousStress` de 40 octets, sans quoi
le terme visqueux s'accumulait au lieu d'amortir. La justesse a coûté du poids avant que
l'optimisation n'en retire. Ce n'est pas un chiffre décoratif : au-delà d'environ 17 Mo de
points matériels, le calcul est limité par la bande passante mémoire et non par le calcul,
et le coût par point saute de 0,15 à 0,247 µs (la falaise du L2, ~16 Mo). Diviser la
structure par deux, c'est doubler la taille du problème qui tient du bon côté de la falaise.

---

## 2. Le cas `dense`, ou pourquoi il ne faut pas comparer des exécutions

En temps de mur, `dense` est **plus lent qu'avant : ×0,67**. Il serait malhonnête de le
cacher, et instructif de l'expliquer.

Le cas a des points matériels quatre fois plus petits que les autres, donc un critère CFL
quatre fois plus strict. Le code d'origine ne le voyait pas — c'était le défaut **B5** — et
tournait avec `dt = 9,0e-7`, au-dessus de la limite de stabilité. Le code actuel le voit et
ramène le pas à `4,6e-7`. Il lui faut donc 5826 pas là où l'ancien en faisait 3001 pour
couvrir la même durée physique.

**La version la plus rapide était celle qui était fausse.** Par pas de calcul — c'est-à-dire à
travail égal — le code actuel est ×1,29 plus rapide sur ce cas.

C'est la raison pour laquelle le tableau du § 1 est normalisé par pas : dès qu'une correction
touche au pas de temps, au nombre de points ou aux critères de convergence, comparer deux
exécutions ne compare plus rien.

---

## 3. D'où viennent les gains — et d'où ils ne viennent pas

Quatre leviers ont payé, dans cet ordre :

| Levier | Ce que c'était | Effet |
|---|---|---|
| **Allocation** | trois `std::vector` construits par point **et par pas** dans `BSpline`, soit 12,7 M d'allocations sur le cas de référence ; la liste des nœuds vivants bâtie dans un `std::set` | ×8,8 et ×11 à ×13 sur ces phases |
| **Redondance de calcul** | les 16 nœuds d'une B-spline ne prennent que 4 positions distinctes par direction : 8 évaluations suffisent au lieu de 32 | ×2,9 sur la phase |
| **Travail inutile** | `prev_F` recopié à chaque pas pour un seul lecteur ; les quatre coins rafraîchis pour personne (`see` les recalcule) | ×1,02 |
| **Taille du jeu de données** | `N[16]` et `gradN[16]` (384 o, 43 % de la structure) sortis dans deux tableaux contigus, puis l'état conditionnel des modèles | ×1,13 puis ×1,08 |

Et ce qui a été essayé puis **écarté**, mesures à l'appui — c'est la partie du bilan qui fait
gagner du temps la prochaine fois :

- **Le parallélisme au-delà de la loi de comportement.** C'était l'hypothèse de départ. Le
  profil dit le contraire : sur le cas réel, `updateStrainAndStress` ne pèse que 5,7 %, et
  le gain OpenMP y est non monotone (×0,90 à 384 points, ×1,35 à 1536, ×1,05 à 32000). Les
  deux premiers gains étaient **séquentiels**. À 32000 points, aucune phase ne dépasse 15 %
  du pas : le profil est plat, ajouter des cœurs n'y peut rien.
- **Le tri spatial des points matériels.** Mélanger leur ordre coûte bien +27 %, mais l'ordre
  ne se dégrade pas : même après écoulement, 100 % des paires consécutives sont dans des
  éléments voisins. Il n'y a rien à trier.
- **La taille des nœuds** (+40 o = +0,5 %) et **la lecture des positions nodales** (0,280 →
  0,279) : sans effet. **La fusion des deux passes P2G** : 2 à 3 %, dans le bruit.

**Un mot de vocabulaire, parce qu'il a fallu le corriger en cours de route** : aucun gain
n'est venu de *l'alignement* des données. Le seul effet mémoire démontré est celui de la
**taille du jeu de données**. Allocation, redondance et travail inutile ont fait le reste.

---

## 4. Justesse et robustesse

La revue a recensé **54 défauts** (plus une entrée de rappel), classés A à D. **26 sont
corrigés.**

| Classe | Ce que c'est | Corrigés |
|---|---|---|
| **A — critique** | comportement indéfini : hors bornes, pointeur nul, division par zéro | **9 / 11** |
| **B — majeur** | le calcul tourne et produit un résultat faux, sans message | **11 / 15** |
| **C — moyen** | robustesse, reprise, fuites mémoire, cas limites | 3 / 15 |
| **D — mineur** | incohérences, code mort, messages trompeurs | 3 / 13 |

Des deux défauts de classe A qui restent, **A11 est sans objet** (`See/cut.cpp` n'est compilé
par aucune cible). Il n'en reste donc qu'un seul de réellement vivant, **A3**.

Ce qui a changé de nature, plus que de chiffre :

- **Les erreurs d'entrée s'arrêtent au démarrage.** Un nom de modèle inconnu, une loi de
  contact inexistante, des paramètres d'interaction manquants, une période nulle, des indices
  de conditions limites hors grille : tout cela produisait auparavant une lecture hors bornes
  ou un résultat silencieusement faux. C'est désormais un message qui nomme le fichier, la
  ligne et ce qu'il faut corriger.
- **Trois résultats faux ont été redressés** : le mélange FLIP/PIC était sans effet dans USF
  et USL (facteur 1 sur l'énergie cinétique), la masse volumique n'y était jamais mise à
  jour (dérive de 6,3e-5), et le matériau ne se déformait plus après un découpage adaptatif.
- **Le critère de pas de temps fait son travail** (§ 2), ce qui est la correction la plus
  lourde de conséquences de toute la campagne : elle change les résultats de tout cas dont
  les points sont petits devant la maille.

Un effet de bord à connaître : la réécriture de `BSpline` fait qu'un point matériel dans un
élément **de bord** est maintenant refusé net, au lieu d'être interpolé faux. C'est le défaut
**A3**, qui était silencieux et qui est devenu visible. Aucun exemple livré ne le déclenche,
mais c'est la première chose à traiter.

---

## 5. Le filet

Rien de ce qui précède n'aurait dû être tenté sans lui, et il a servi à chaque lot.

- **`python3 Tests/runtests.py`** — 29 tests, 8 secondes, stdlib Python seule. Aucun ne fige
  de valeur numérique de référence : chacun exhibe une propriété que le code *devrait*
  vérifier (partition de l'unité, conservation de la masse, travail du poids, indépendance
  au pas de temps, indépendance à `proxPeriod`…). Une empreinte numérique prise avant
  correction aurait enregistré les bugs comme comportement attendu.
- **`python3 Tests/benchmark.py`** — 18 secondes. Il compare **le résultat autant que le
  temps** : huit scalaires au seuil 1e-9, et sortie en code 1 si la physique a bougé. Une
  optimisation qui change le résultat est une régression, pas une optimisation.
- **Une construction sous AddressSanitizer**, pour ce que les deux précédents ne voient pas.
  Le dépassement de tableau du défaut A7 ne durait qu'un seul pas et ne plantait pas de façon
  fiable : le test écrit pour lui passait alors que le bug était là. La recette est dans le
  journal de `Doc/BUGS.md`, à l'entrée du 6 août.

Volume : **1257 lignes ajoutées et 550 supprimées sur 63 fichiers** de code, auxquelles
s'ajoutent environ 7300 lignes de documentation et de tests — les trois quarts de l'écrit
sont le filet et le compte rendu, pas le solveur.

---

## 6. Ce qui reste, dans l'ordre

1. **A3** — la couronne de nœuds fantômes, pour que les éléments de bord cessent d'être un
   cas particulier. C'est le seul défaut critique vivant, et il est devenu visible.
2. **B12 + C1** — quatre espions sur six ouvrent leur fichier sans vérifier
   `computationMode` : **ouvrir un conf-file dans `see` efface les résultats**. Et `clean()`
   ne vide pas `Spies`, si bien que chaque relecture ajoute une instance et rouvre — donc
   revide — le fichier. Les deux se traitent d'une seule passe.
3. **C3** — le format des conf-files. La reprise n'est pas exacte, et la liste s'est allongée
   du fait même de l'optimisation : `hardeningForce`, `outOfPlaneEp` et `viscousStress` ne
   sont toujours pas sauvegardés. C3 débloque aussi les 96 derniers octets de
   `MaterialPoint` (`strain`, `plasticStrain`, `stressCorrection`), qui ne peuvent en sortir
   sans toucher à la lecture et à l'écriture.
4. **B8** et **C6** — `VonMises` écrase la déformation plastique au lieu de la cumuler ;
   l'apex de `MohrCoulomb` est divisé par `sin φ` et ne converge pas silencieusement. Les
   deux changent des résultats publiables : à valider sur un cas de référence.
5. **D14** — `mpmbox` sort avec le code 0 quand il plante. Une ligne, et tout ce qui appelle
   le solveur (script, `make`, ordonnanceur SLURM) redevient capable de distinguer un calcul
   terminé d'un calcul explosé.
6. **D11** — `t += dt` fait manquer le dernier conf-file. Cosmétique sur le papier ; c'est
   arrivé deux fois pendant l'écriture des tests.

Et un point qui n'est pas un défaut du code : `Examples/BoulderImpact_NRJ/input.txt` déclare
`BoundaryForceLaw frictionalViscoElastic 1` alors que tous ses obstacles sont en groupe 0. Ce
cas tourne donc depuis toujours avec la loi par défaut. Le nouvel avertissement du défaut A6
l'a révélé.
