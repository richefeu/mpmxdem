# Input File Syntax Reference

This document describes the syntax and structure of input files (shared with configuration files). It also includes implementation details and key physics concepts relevant to the simulations. 

---

## Simulation time flow

* **`t`** <(double) value>

 This is the current time. Normally it is 0 for input-file (*e.g.*, `input.txt`) or a value smaller than `tmax` in a conf-file (*e.g.*, `conf102`).

* **`tmax`** <(double) value>
 
 End time of the simulation.

* **`dt`** <(double) value>
 
 Time step. Remember that it has to be a lot less than $\pi \sqrt{m/k}$.

* **`iconf`** <(int) value>
 
 The configuration number (should be the number of the file)
 

## Generated files

* **`interOut`** <(double) value>
 
 Elapsed time between the data storage in text file during the computation (`cell.out.txt`, `stress.out.txt`, `strain.out.txt` et `resultant.out.txt`)
 
 `cell.out.txt` contains
 
 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |  
 |---|---|---|---|---|---|---|---|---|----|
 | time | $h_{xx}$ | $h_{xy}$ | $h_{xz}$ | $h_{yx}$ | $h_{yy}$ | $h_{yz}$ |$h_{zx}$ | $h_{zy}$ | $h_{zz}$ |
 
 | 11 | 12 | 13 | 14 | 15 | 16 | 17 | 18 | 19 |
 |----|----|----|----|----|----|----|----|----|
 | $\dot{h}_{xx}$ | $\dot{h}_{xy}$ | $\dot{h}_{xz}$ | $\dot{h}_{yx}$ | $\dot{h}_{yy}$ | $\dot{h}_{yz}$ |$\dot{h}_{zx}$ | $\dot{h}_{zy}$ | $\dot{h}_{zz}$ |
 
 `stress.out.txt` contains
 
 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |  
 |---|---|---|---|---|---|---|---|---|----|
 | time | $\sigma_{xx}$ | $\sigma_{xy}$ | $\sigma_{xz}$ | $\sigma_{yx}$ | $\sigma_{yy}$ | $\sigma_{yz}$ |$\sigma_{zx}$ | $\sigma_{zy}$ | $\sigma_{zz}$ |

 `strain.out.txt` contains
 
 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |  
 |---|---|---|---|---|---|---|---|---|----|
 | time | $\varepsilon_{xx}$ | $\varepsilon_{xy}$ | $\varepsilon_{xz}$ | $\varepsilon_{yx}$ | $\varepsilon_{yy}$ | $\varepsilon_{yz}$ |$\varepsilon_{zx}$ | $\varepsilon_{zy}$ | $\varepsilon_{zz}$ |

 `resultant.out.txt` contains

 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |  
 |---|---|---|---|---|---|---|---|---|----|
 | time | $\langle \vert \vec{F} \vert \rangle$ | $\langle \vert \vec{F} \vert_2 \rangle$ | $\text{min}(f_n)$ | $\langle f_n \rangle$| $N_\text{bonds}$ | $N_\text{break}^\text{tens}$ | $N_\text{break}^\text{fric}$ |$V_\text{cell}$ | $v_\text{max}$ |
 
 | 11 | 12 | 13 | 14 | 15 |
 |----|----|----|----|----|
 | $v_\text{min}$  | $\langle v \rangle$ | $\Delta v$ | ReducedPartDistMean | $E_k$ |
 
 where $\langle \vert \vec{F} \vert \rangle$ is the average resultant force over all bodies, $\langle \vert \vec{F} \vert_2 \rangle$ is same but with only the bodies holding more than 2 contacts, $\text{min}(f_n)$ is the minimum normal force strictly superior to zero, $\langle f_n \rangle$ is the mean nonzero normal force ...
 
 > The name of this file is not ok. I will change it later.
 
 
 With gnuplot, to plot data from different files, you can do that way:
 
 ```
 plot "< paste strain.out.txt stress.out.txt" u ($2+$6+$10):($12+$16+$20) w lp pt 7
 ```

* **`interConf`** <(double) value>
 
 Elapsed time between the conf-file dumps (the format is described in this document)


## Periodic cell


* **`hmass`** < (*double*) **value** >
 
 Mass of the collective degrees of freedom.
 

> ℹ️
> This parameter represents a mass that must be defined 
> when applying a pressure load to the system. While 
> its physical interpretation may not be immediately obvious, assigning 
> it a value equivalent to the mass of a single particle typically 
> yields satisfactory results based on empirical observations.

 
* **`h`** < (*mat9r*) **values** >
 
 This is a 3-by-3 matrix where the columns are the vectors that form the periodic cell:
 
 $$
 \mathbf{h} = 
 \begin{pmatrix}
   h_{xx} & h_{xy} & h_{xz} \\
   h_{yx} & h_{yy} & h_{yz} \\
   h_{zx} & h_{zy} & h_{zz} 
 \end{pmatrix}
 \text{ means that the vectors are }
 \begin{pmatrix}
   h_{xx} \\ h_{yx} \\ h_{zx} 
 \end{pmatrix}
 \text{, }
  \begin{pmatrix}
   h_{xy} \\ h_{yy} \\ h_{zy} 
 \end{pmatrix}
 \text{, and }
 \begin{pmatrix}
   h_{xz} \\ h_{yz} \\ h_{zz} 
 \end{pmatrix}
 $$


> ℹ️
> The components are written line by line in the conf-file. For example, a $1 \times 2 \times 3$ box,
> the line will be `h  1 0 0  0 2 0  0 0 3`


* **`vh`** <(mat9r) values>
 
 This is the cell velocities $\dot{\mathbf{h}}$ (collective velocities).
 
* **`ah`** <(mat9r) values>
 
 This is the cell accelerations $\ddot{\mathbf{h}}$
 
* **`hvelGrad`** <(mat9r) values>
 
 This is computed as $\nabla \mathbf{v} = \dot{\mathbf{h}} \cdot \mathbf{h}$. It is here only for processing purpose.

* **`hstrain`** <(mat9r) values>
 
 xxx $\mathbf{\varepsilon} = \frac{1}{2}\left( \nabla \mathbf{v} + \nabla \mathbf{v}^T\right)$
 
* **`Sig`** <(mat9r) values>
 
 This is the averall mean stress tensor. 


## Sample


### Particles

* **`Particles`** <(int) nbParticles>

[...]


### Interactions

* **`Interactions`** <(int) nbInteractions> 

[...]


## Loading

[...]

## Neighbor list

 
* **`interVerlet`** <(double) value>
 
 Elapsed time between the updates of neighbor list

* **`dVerlet`** <(double) value>
 
 Distance above which two spheres are considered as being close (and so part of the neighbor list)
 
* **`NLStrategy`** <(int) number>
 
 Strategy used to build the Verlet list. it can be `0` for the brute force solution with complexity O(N^2), or `1`for the link-cells solution (WARNING: probably not working fully correctly for now)
 
  
## Particle and interation parameters

* **`density`** < (*double*) **value** >
 
 Mass per unit volume (e.g. kg/m$^3$) for all paraticles. 
 😅 There is actually only one particle parameter which is there density 
 
* **`kn`** < (*double*) **value** >
 
 xxx
 
* **`kt`** < (*double*) **value** >
 
 xxx

* **`kr`** < (*double*) **value** >
 
 xxx
 
* **`dampRate`** < (*double*) **value** >
 
 xxx 

* **`mu`** < (*double*) **value** >
 
 xxx

* **`mur`** < (*double*) **value** >
 
 xxx

* **`fcoh`** < (*double*) **value** >
 
 xxx

* **`fn0`** < (*double*) **value** >
 
 xxx

* **`ft0`** < (*double*) **value** >
 
 xxx

* **`mom0`** < (*double*) **value** >
 
 xxx
 
* **`dn0`** < (*double*) **value** >
 
 xxx

* **`dt0`** < (*double*) **value** >
 
 xxx

* **`drot0`** < (*double*) **value** >
 
 xxx

* **`powSurf`** < (*double*) **value** >
 
 xxx

* **`zetaMax`** < (*double*) **value** >
 
 xxx
 
* **`zetaInter`** < (*double*) **value** >
 
 xxx
 
* **`softening`** < (*string*) **model** >
 
 xxx can be `linear`, `gate` or `trainee` 

* **`permamentGluer`** <0 or 1>

 xxx
 
* `continuumContact` <(double) value>
 
 xxx
 
* `Kratio` <(double) value>
 
 xxx
 
* `xxxx` <(double) value>
 
 xxx
 
* `xxxx` <(double) value>
 
 xxx
 
* `xxxx` <(double) value>
 
 xxx 
 
* `xxxx` <(double) value>
 
 xxx
 
* `xxxx` <(double) value>
 
 xxx
 
* `xxxx` <(double) value>
 
 xxx
 
* `xxxx` <(double) value>
 
 xxx

## velocity ramp

* **`rampDuration`** < (*double*) **value** >
 
 xxx 

* **`limitHboxvelocity`** < (*double*) **value** >
 
 xxx Bon ce truc n'est pas clair, il faut régler ce problème

## Cundall damping

* `numericalDampingCoeff` <(double) value>
 
 xxx
 
 
### Some flags

* `enableSwitch` <0 or 1>
	
### Preprocessing

* `ActivateBonds` <(double) distance>
 
 xxx
 
* `ActivateDamageableBonds` <(double) distance>
 
 xxx
 
* `nodamage`
 
 xxx

* `RemoveBondsRandomly` <(double) percent>
 
 xxx

* `RemoveBondsSmallestPairs` <(double) percent>
 
 xxx 
 
* `RecomputeMassProperties` 
 
 xxx

* `RandomVelocities` <(double) maxVelocityComponent>
 
 xxx
 
 