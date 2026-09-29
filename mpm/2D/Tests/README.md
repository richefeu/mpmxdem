# Tests de MPMbox

Deux outils, à passer tous les deux après une modification :

| | ce qu'il vérifie | durée |
|---|---|---|
| `python3 Tests/runtests.py` | **la physique** — invariants et défauts connus | ~3 s |
| `python3 Tests/benchmark.py` | **le temps**, et que le résultat n'a pas bougé | ~18 s |

Le banc de mesure est décrit dans [`../Doc/OPTIM.md`](../Doc/OPTIM.md), avec le
profil détaillé du pas de temps et les pistes d'optimisation chiffrées. Le reste
de ce fichier porte sur la suite de non-régression.

---

## Tests de non-régression

Filet de sécurité destiné à être mis en place **avant** de corriger les défauts
listés dans [`../Doc/BUGS.md`](../Doc/BUGS.md). Toute la suite tourne en une
poignée de secondes ; il n'y a aucune dépendance hors de la bibliothèque
standard de Python 3.

```bash
python3 Tests/runtests.py                        # tout
python3 Tests/runtests.py T10 T11                # une sélection
python3 Tests/runtests.py -k splitting           # par motif sur le nom
python3 Tests/runtests.py -v                     # + le détail des XFAIL
python3 Tests/runtests.py -l -v                  # lister sans exécuter
python3 Tests/runtests.py --exe BUILD/mpmbox     # binaire explicite
```

L'exécutable est cherché dans `BUILD/`, `build/`, `bld/`, puis dans le `PATH` ;
la variable d'environnement `MPMBOX` ou l'option `--exe` prennent le dessus.
Les répertoires de travail sont conservés dans `Tests/_work/<cas>/` pour
pouvoir rejouer un cas à la main après un échec :

```bash
cd Tests/_work/T11_travail && ../../../BUILD/mpmbox input.txt -v 5
```

## Les trois familles

| Statut | Signification |
|---|---|
| **PASS** | invariant vérifié — c'est ce qu'on attend |
| **FAIL** | **régression** : une propriété vraie avant la correction ne l'est plus |
| **XFAIL** | défaut connu, toujours présent — attendu tant qu'il n'est pas corrigé |
| **XPASS** | le défaut ne se manifeste plus : il est corrigé, **basculer le test en `INVARIANT`** |

Le code de retour du script est 1 s'il y a au moins un **FAIL**, 0 sinon. Un
XPASS n'est pas un échec, mais il faut y donner suite : c'est le signal qu'un
test doit changer de famille.

### La règle d'usage

1. Lancer la suite **avant** de toucher au code, et noter l'état de référence
   (voir ci-dessous).
2. Corriger un défaut.
3. Relancer. Le test du défaut corrigé doit passer XFAIL → XPASS, et **aucun
   PASS ne doit devenir FAIL**.
4. Éditer `runtests.py` : passer le test de `XFAIL` à `INVARIANT` et retirer
   l'argument `bug=`. Il devient alors un vrai test de non-régression.

## État courant

```
  T01   freefall_linear                    PASS
  T02   freefall_regularquadlinear         PASS
  T03   shapefunctions_equivalentes        PASS
  T04   conservation_masse                 PASS
  T05   colonne_elastique_stable           PASS
  T06   reprise_identique                  PASS
  T07   convergence_en_dt                  PASS
  T08   contact_pas_de_traversee           PASS
  T09   critere_de_pas_de_temps            PASS      <- B5  corrigé le 2026-08-05
  T10   gravite_constante_sous_rampe       PASS      <- B6  corrigé le 2026-08-05
  T11   travail_du_poids                   PASS      <- B3  corrigé le 2026-08-05
  T12   usl_gradient_de_vitesse            PASS      <- B2  corrigé le 2026-08-05
  T13   splitting_ne_gele_pas_F            PASS      <- B1  corrigé le 2026-08-05
  T14   decoupage_adaptatif                PASS      <- B11 corrigé le 2026-08-05
  T15   kelvinvoigt_independant_de_dt      PASS      <- B7  corrigé le 2026-08-05
  T16   frottement_pas_de_reptation        PASS      <- B9+B10 corrigés le 2026-08-05
  T17   schemas_dintegration_equivalents   PASS      <- B15 corrigé le 2026-08-05
  T18   parite_des_schemas                 PASS      <- FLIP/PIC + densité portés dans USF/USL
  T20   modele_inconnu                     PASS      <- A4  corrigé le 2026-08-05
  T21   loi_de_contact_inconnue            PASS      <- A6  corrigé le 2026-08-05
  T22   parametres_dinteraction_manquants  PASS      <- A5  corrigé le 2026-08-05
  T23   periode_nulle                      PASS      <- A10 corrigé le 2026-08-05
  T24   conditions_limites_hors_grille     PASS      <- A9  corrigé le 2026-08-05
  T25   point_sorti_de_la_grille           PASS      <- A1  corrigé le 2026-08-05
  T26   parametre_set_inconnu              PASS      <- C12 corrigé le 2026-08-05
  T27   noeuds_avant_la_grille             PASS      <- C14 corrigé le 2026-08-05

  26 PASS   0 FAIL   0 XFAIL   0 XPASS
```

### Journal

| Date | Défaut corrigé | Effet sur la suite |
|---|---|---|
| 2026-08-05 | **A1** — indice d'élément non borné | T25 : `SIGBUS` → `PASS`, reclassé en invariant. Les 8 autres invariants inchangés, les 12 autres XFAIL inchangés **à la valeur numérique près**. |
| 2026-08-05 | **A4, A5, A6, A9, A10, C12, C14** — validation des entrées | T20 à T24 : `XFAIL` → `PASS`, reclassés en invariants. T26 et T27 ajoutés pour C12 et C14, qui n'étaient pas couverts. Les 9 invariants inchangés, les 7 XFAIL restants inchangés à la valeur numérique près. |
| 2026-08-05 | **B9, B10** — historique de contact | T16 : `XFAIL` → `PASS`. Le test a été **refait** : il mesurait la fin d'un transitoire et pas un fluage, et son assertion principale est maintenant l'indépendance à `proxPeriod`. Les 16 autres invariants inchangés, les 6 XFAIL restants inchangés à la valeur numérique près. |
| 2026-08-05 | **B3, B5, B6** | T10 et T11 : `XFAIL` → `PASS`. T09 ajouté pour B5, qui n'était pas couvert. T11 a été **recalibré** : il comparait le cumul du spy à un instant et le déplacement à un autre, d'où un résidu de 44 % qui n'était pas un défaut du code. Les 17 autres invariants inchangés, les 4 XFAIL restants inchangés à la valeur numérique près. |
| 2026-08-05 | **B1, B2, B11** | T12, T13 et T14 : `XFAIL` → `PASS`. **T14 a été réécrit sur une prémisse corrigée** : la somme des `vol0` n'est pas une grandeur conservée, c'est la somme des `vol` qui l'est. T17 ajouté pour **B15**, défaut d'`UpdateStressLast` découvert en vérifiant B2. Les 20 autres invariants inchangés. |
| 2026-08-05 | **Mise à niveau des schémas** + **B4** | T18 ajouté : vérifie sur les **trois** schémas que la masse est conservée et que l'amortissement PIC agit. Avant, `UpdateStressFirst` donnait une énergie cinétique **identique** avec et sans PIC (facteur 1) et une dérive de masse de 6,3e-5. Les 25 autres tests inchangés. |
| 2026-08-05 | **B7, B15** | T15 et T17 : `XFAIL` → `PASS`. **Plus aucun XFAIL** : tous les défauts couverts par la suite sont corrigés. B7 : l'écart entre `dt` et `dt/2` passe de 47 % à 0,06 %. B15 : `UpdateStressLast` passe de « éjecte un point » à 7,2e-5 sur det F, meilleur que `ModifiedLagrangian`. |

En plus de la suite, les sept exemples de `Examples/` ont été relancés à chaque étape : tous
démarrent et tournent, y compris le cas double échelle `SmallOedo-MPMxDEM`. C'est ce qui a
révélé que `BoulderImpact_NRJ` assigne sa loi de contact à un groupe d'obstacles inexistant
(voir le journal de `Doc/BUGS.md`). Ce contrôle-là vaut la peine d'être refait après chaque
correction — la suite ne couvre que des cas synthétiques.

**Tous les défauts couverts par cette suite sont corrigés.** Ceux qui restent dans
`Doc/BUGS.md` ne sont pas testés : voir « Ce qui n'est pas couvert » plus bas.

## Ce que chaque test protège

### Invariants (doivent tenir avant comme après)

- **T01 / T02** — chute libre d'un bloc libre, fonctions de forme `Linear` et
  `RegularQuadLinear`, sur des mailles **non carrées** (`lx = 2·ly`). Vérifie
  d'un coup : $\sum N = 1$ (sinon l'accélération n'est pas $g$), $\sum \nabla N = 0$
  (sinon des contraintes parasites apparaissent), et l'appariement
  nœud/fonction. C'est exactement ce jeu de conditions qui a permis de valider
  la réécriture de `Linear`.
- **T03** — les deux fonctions de forme implémentent la même interpolation
  bilinéaire : elles doivent coïncider à l'arrondi près. Les tolérances sont
  données champ par champ, à l'échelle de la grandeur — comparer une contrainte
  nulle à $10^{-14}$ près n'aurait aucun sens.
- **T04** — masse totale et volume initial total conservés, **sans découpage**
  (avec découpage, $\sum \mathrm{vol}_0$ n'est plus une grandeur conservée : voir T14).
- **T05** — une colonne élastique amortie converge vers $\sigma_{yy} \simeq -\rho g h/2$,
  ne décolle pas, ne traverse pas le plancher.
- **T06** — une reprise depuis un conf-file redonne exactement le même état
  qu'un calcul direct. **C'est le test qui détectera toute variable d'état
  ajoutée et oubliée dans `save()`** — c'est-à-dire le chantier C3.
- **T07** — convergence en pas de temps : diviser $dt$ par deux ne change les
  déplacements que de quelques pourcent.
- **T08** — un bloc lâché ne traverse jamais le plancher et son énergie
  mécanique ne croît pas.
- **T14** — un découpage adaptatif conserve la matière, produit des numéros
  uniques et ne coupe chaque point qu'une fois par pas. Attention à la grandeur
  conservée : c'est $\sum \mathrm{vol}$, l'aire réellement occupée, et non
  $\sum \mathrm{vol}_0$ — `vol0` est l'empreinte de référence, laissée intacte,
  et c'est $\det F$ qui porte la division. La première version de ce test se
  trompait là-dessus.
- **T18** — les fonctionnalités ne sont pas réservées à un schéma : masse
  conservée et amortissement PIC effectif, pour `ModifiedLagrangian`,
  `UpdateStressFirst` et `UpdateStressLast`.
- **T09** — le pas de temps retenu est bien le plus contraignant des critères,
  et il vaut la valeur analytique du CFL. Vérifie aussi qu'un cas sans obstacle
  ne produit ni NaN ni ajustement intempestif.
- **T16** — un bloc posé sur une pente à 18° avec $\mu = 1{,}0$ s'immobilise, et
  la fréquence de reconstruction des listes de voisins n'y change rien.

### Le principe de construction

Il n'y a plus aucun `XFAIL`, mais le principe reste le même pour tout nouveau
test : **exhiber une propriété que le code devrait vérifier**, plutôt que de
figer une valeur numérique de référence — laquelle enregistrerait le bug au
lieu de le révéler.

Trois exemples valent d'être lus avant d'en écrire un nouveau :

- **T15** — on n'y teste pas la valeur de la contrainte visqueuse, qui
  demanderait une solution analytique, mais son **indépendance au pas de
  temps**. Une contrainte visqueuse cumulée équivaut à une rigidité $\eta/dt$,
  qui double quand $dt$ est divisé par deux : c'est exactement ce que le test
  mesure, et l'écart est passé de 47 % à 0,06 %.

- **T10** — la rampe de gravité est réglée avec des extrémités **identiques**.
  Une interpolation correcte laisse alors la gravité constante ; le terme
  constant manquant la faisait tomber à zéro pendant toute la rampe. Le test est
  donc insensible à la formule exacte d'interpolation.
- **T16** — on n'y mesure pas une valeur de fluage, mais le fait que la
  **fréquence de reconstruction des listes de voisins**, réglage purement
  numérique, ne doit pas changer la physique. C'est ce qui a permis de conclure
  sur B9/B10 : les valeurs mesurées pour `proxPeriod` 5, 10, 100 et 1000
  coïncident maintenant à la sixième décimale, contre un facteur 47 avant.
  Sa première version mesurait sur la seconde moitié d'un calcul de 0,6 s, donc
  encore en plein transitoire de mise en place ; elle a été refaite sur le
  dernier tiers d'un calcul de 1,5 s.

### Robustesse

T20 à T27 vérifient qu'une entrée invalide est **refusée proprement** :
code de retour non nul, pas de signal. Tous sont au vert depuis la passe de
validation des entrées du 2026-08-05. Ce sont les tests les moins chers de la
suite : chaque cas s'arrête presque immédiatement.

Chacun porte un `expectMsg` : il ne suffit pas que le programme s'arrête, il
faut qu'il s'arrête pour la bonne raison. T25 exige ainsi la présence de
`has left the grid` dans la sortie, faute de quoi un arrêt dû à tout autre
motif ferait passer le test à tort.

## Ce qui n'est pas couvert

- **B12** (les spies vident leurs fichiers en mode visualisation) — demande de
  lancer `see`, donc un contexte OpenGL. Vérification manuelle :

  ```bash
  cd Tests/_work/T11_travail
  wc -l energy.txt        # non vide
  ../../../BUILD/see conf1.txt      # ouvrir puis quitter
  wc -l energy.txt        # aujourd'hui : vidé
  ```

  Le fichier `conf1.txt` doit d'abord recevoir une ligne `Spy EnergyBalance
  200 energy.txt`, comme le fait la procédure de reprise décrite dans le manuel.

- **A2 / A3** (B-splines aux bords) — la famille `BSpline` demande un jeu de
  tests à part, la construction des éléments de bord étant à revoir avant que
  quoi que ce soit soit mesurable.

- **Le double échelle (MPM×DEM)** — aucun test : il faudrait `libPBC3D.a` et un
  fichier de configuration DEM. Les défauts B11 (cellule DEM partagée après
  découpage), A8 et C3 (état DEM non sauvegardé) restent donc non couverts.

- Les modèles `MohrCoulomb`, `VonMisesElastoPlasticity`, `SinfoniettaClassica`
  et `SinfoniettaCrush` ne sont pas testés. Un test d'écrouissage sur
  `SinfoniettaClassica` serait le complément naturel pour couvrir C3
  (`hardeningForce` non sauvegardé).

## Ajouter un test

```python
@test("T30", "nom_court", INVARIANT, doc="""
Ce que la propriete signifie, et pourquoi elle doit tenir.""")
def t30(ctx):
    c = ctx.case("T30_nom")
    c.write("input.txt", timedHeader(1e-5, 2000) + HOOKE + GRID + "\n" + BLOCK)
    c.run()
    c1 = c.conf(1)
    ctx.close(c1.meanStress()[3], -1234.0, 0.01, "sigma_yy moyen")
```

Quelques règles apprises en écrivant celle-ci :

- Utiliser `timedHeader(dt, nsteps)` plutôt que `header(tmax=..., confPeriod=...)`
  dès qu'on compare deux calculs : il place `conf1` **exactement** au pas
  demandé et évite que le cumul `t += dt` fasse manquer le dernier conf-file
  (défaut D11 — c'est arrivé en écrivant T15).
- Toujours vérifier que la comparaison a bien lieu à $t > 0$ et que la
  grandeur observée est assez grande pour conclure. Un test qui compare
  `conf0` à `conf0` passe, et ne prouve rien (c'est arrivé en écrivant T07).
- Choisir des tolérances à l'échelle de la grandeur comparée, pas une tolérance
  unique (c'est arrivé en écrivant T03).
- `ctx.close(got, want, tol, quoi)` en relatif, `ctx.below(got, tol, quoi)` en
  absolu, `ctx.expect(cond, message)` sinon. Le message apparaît tel quel dans
  le rapport : y mettre les valeurs.
