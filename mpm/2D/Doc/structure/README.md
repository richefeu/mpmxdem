# Diagrammes de structure de MPMbox

Trois formats du même contenu : une vue d'ensemble puis un diagramme par famille de classes.

| Fichier | Usage |
|---|---|
| `mpmbox_structure.md` | sources **mermaid** (français) — GitHub, ou doc Sphinx via `sphinxcontrib-mermaid` |
| `mpmbox_structure.html` | page autonome en français (publiée aussi comme artifact) |
| `tikz/fig*.tex` → `tikz/fig*.pdf` | figures **PDF en français** (présentation, cours) |
| `tikz/en/fig*.tex` → `tikz/en/fig*.pdf` | figures **PDF en anglais**, celles utilisées par `Doc/mpmbox_doc.tex` |

Les deux jeux TikZ ont le même contenu ; seules les figures 1, 3 et 7 ont une mise
en page différente en anglais, pour tenir dans la largeur d'une page A4 sans être
réduites (le texte resterait illisible).

## Les figures

| Figure | Contenu |
|---|---|
| `fig1_overview` | vue d'ensemble : `MPMbox`, l'état simulé, les familles enfichables |
| `fig2_constitutive` | `ConstitutiveModel` et ses lois, dont `CHCL_DEM` (couplage MPM×DEM) |
| `fig3_obstacles` | `Obstacle` et `BoundaryForceLaw` |
| `fig4_shapefunctions` | `ShapeFunction` |
| `fig5_onestep` | `OneStep` (schémas d'intégration) |
| `fig5b_cycle` | le cycle d'un pas de temps |
| `fig6_commands_schedulers` | `Command` et `Scheduler` |
| `fig7_spies` | `Spy` (les sondes) |

## Recompiler les PDF

```sh
cd tikz    && ./build.sh   # version française
cd tikz/en && ./build.sh   # version anglaise (celle de la doc)
```

Il faut `pdflatex` (TeX Live) ; le style commun est dans `tikz/mpmbox-style.tex`
(couleurs, boîtes, connecteurs). Les figures sont des `standalone`, donc chaque
PDF est recadré sur son contenu.

## Dans la doc LaTeX

C'est déjà fait : la section « Code structure » de `Doc/mpmbox_doc.tex` (juste après
la table des matières, `\label{sec:structure}`) inclut les huit figures anglaises.
Le préambule a reçu `\usepackage{float}` pour que les figures restent à l'endroit
où on en parle (`[H]`).

Pour en ajouter une :

```latex
\includegraphics[width=0.70\linewidth]{structure/tikz/en/fig4_shapefunctions.pdf}
```

Les largeurs sont choisies pour que chaque figure soit à sa taille naturelle
(la colonne fait 17 cm) ; une figure trop réduite devient illisible.

## Insertion dans Sphinx

Ajouter `sphinxcontrib.mermaid` aux `extensions` de `sphinxdoc/source/conf.py`,
puis inclure les blocs de `mpmbox_structure.md` (ou les convertir en `.rst` avec
la directive `.. mermaid::`).

## Mise à jour

La liste des variantes est celle enregistrée dans
`MPMbox::ExplicitRegistrations()` (`Core/MPMbox.cpp`). Après l'ajout d'une classe,
c'est là qu'il faut aller vérifier ce qui manque dans les diagrammes.
