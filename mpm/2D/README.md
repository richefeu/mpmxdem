### Notes pour le developpement du couplage MPMxDEM (29/04/2021, Olivier & Vincent)

On va mettre en place l'outil en couplant `PBC3D_sandstone` avec `MPMbox` (2D)

Olivier a fait un peu de ménage dans les deux codes avant de les "coller".
Il a été décidé, avant, d'inclure "en dur" `PBC` and `MPM`. En particulier, les fichiers hpp qui était dans le répertoire de PBC3D et qui avaient leur double dans le répertoire `common` ont été remplacés.

Olivier a fait une première tentative d'implémentation qu'on a revu. Voici la stratégie qui en ressort.

Une simulation DEM à petite échelle sera un modèle constitutif comme il en existe déjà dans le code MPM. Une proposition de nom pourait être  `NHLDEM` pour Numerically Homogenised Law avec DEM_sandstone. C'est au sein de ce modèle constitutif particulier qu'est stocké une ou plusieurs version d'échantillon initial. Une possibilité d'entré dans le fichier de commande :

```
#     modelID  modelName  NbInputFiles  File(s)...
model NHLDEM   DEM        2             sampleA.txt  sampleB.txt
```

Chaque point materiel a un pointeur vers une instance de simulation DEM, et ce pointeur est par défaut `null`. Au moment où on crée le point matériel, on précise le numéro d'échantillon voulue (0 pour `sampleA.txt` et 1 pour `sampleB.txt`), et on **duplique** l'échantillon selectionné. Dans le fichier de commande, ça pourait se traduire comme ça :

```
select_sample 0
#               grpID  modelName SampleID x0   y0   x1   y1   size
set_NHLMP_grid  0      DEM       0        2.0  0.5  3.0  0.6  0.005
set_NHLMP_grid  0      DEM       1        2.0  0.6  3.0  0.7  0.005
```


