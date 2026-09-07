# mpmpost — post-processing of an MPMbox computation

`mpmpost` is built the same way as `see`: it links against `libMPMbox.a` and
reloads the `conf<N>.txt` files that a computation has written. Instead of
drawing them, it applies to each of them a list of actions declared in a
command file.

```sh
mpmpost [commandFile]      # default: post.txt
```

The tool never writes into the computation folder unless asked to: give a
`result_folder` and every output lands there.

## Command file

One command per line. `#`, `!` and `/` start a comment.

### Global settings

**`source_folder`** (_string_)`path`

> Where the `conf<N>.txt` files are read. Default `.`.

**`result_folder`** (_string_)`path`

> Where the output files are written; created if missing. Default `.`.

**`confs`** (_int_)`first` (_int_)`last` (_int_)`stride`

> Range of configurations to walk through, bounds included. A configuration
> that does not exist is reported and skipped, so an over-large `last` is
> harmless.

**`verbose`** (_bool_)`onOff`

> Print one line per loaded configuration. Default `1`.

**`smoothing`** (_bool_)`onOff`

> `1` (default) projects the fields on the grid and brings them back, which is
> what `see` displays. `0` uses the values the Material Points carry in the
> conf-file, with no projection.
>
> Of everything `MPMbox::postProcess` fills, only three fields are actually
> smoothed: the velocity, the stress and its out-of-plane component. The
> position, the deformation gradient, the density and the corners of a point
> are copied straight from it and are the same either way.
>
> The velocity gradient is the exception, and it stays on the grid in both
> modes: a Material Point carries no velocity gradient of its own — `MPMbox::save`
> does not write `MaterialPoint::velGrad` to the conf-file — and in the MPM a
> velocity gradient is a grid quantity by construction, `sum_r gradN_r v_r`.
> The shear rate is therefore identical in both modes. The inertial number is
> not, since `I` also depends on the pressure.
>
> Which to choose depends on what is being measured. For a `mu(I)` rheology,
> `smoothing 0` is the honest option: the raw stress of a Material Point sits
> exactly on the yield surface after the return mapping, whereas the
> projection averages neighbouring points and manufactures scatter around it.
> The projection also lends a pressure to points that have none — a free
> surface point ends up with the pressure of its neighbours — which is why
> `Pmin` discards far fewer points with smoothing on.

### Actions

**`Post`** (_string_)`actionName` (...parameters that depend on `actionName`...)

Several `Post` lines may be given; they are all applied to every
configuration, in the order they are declared.

#### `Runout` (_string_)`fileName` (_double_)`xWall` (_double_)`yFloor` (_double_)`L0` (_double_)`H0`

Front position and deposit height of a column collapse, one line per
configuration. `xWall` and `yFloor` locate the corner the column rests
against; `L0` and `H0` are its initial width and height.

The front and the height are taken from the four corners of the **deformed**
Material Points (`ProcessedDataMP::corner`, built from `F`), not from their
centres, so a point contributes its own size to the runout.

Columns: `iconf t xf (xf-xWall)/L0 hf (hf-yFloor)/H0 xG yG Ekin vmax nbMP`,
where `xG, yG` is the centre of mass, `Ekin` the total kinetic energy and
`vmax` the largest Material Point speed. `Ekin` falling to zero is the
practical way of telling that the flow has stopped.

#### `MPFields` (_string_)`baseName` (_double_)`xmin` (_double_)`ymin` (_double_)`xmax` (_double_)`ymax`

Per-Material-Point fields, one file `<baseName><confNum>.txt` per
configuration, one line per point whose centre falls in the box. A box larger
than the grid takes them all. This is the command-file version of the old
`See/cut.cpp`.

Columns: `nb group x y vx vy` then the smoothed stress
(`sig_xx sig_xy sig_yx sig_yy sig_zz sig_xz sig_yz`), the deformation gradient
(`F_xx F_xy F_yx F_yy`), the total strain (`eps_xx eps_xy eps_yx eps_yy`),
the smoothed velocity gradient (`L_xx L_xy L_yx L_yy`), then
`rho vol mass plastic`.

> Careful when reading the MPMbox sources: the field named `strain` in
> `ProcessedDataMP` actually holds the deformation gradient `F`
> (`MPMbox::postProcess` does `Data[p].strain = MP[p].F`). The columns above
> are named after what they contain, not after that field.

#### `Scalars` (_string_)`fileName` (_double_)`d` (_double_)`rho_s` (_double_)`Pmin` (_double_)`gammaMin` (_string_)`zzMode` [(_double_)`Poisson`]

Macroscopic `mu(I)` scalars, taken from the Material Points themselves. This
is the single-scale counterpart of `DEMScalars`: it works for any
constitutive model — `MohrCoulomb` in particular — and reports the same
`P`, `tau`, `mu`, `gdot` and `I`, so a classical run and a double-scale run
can be drawn on the same axes.

- `d`, `rho_s` — grain diameter and grain density. A continuum has no grains,
  so they are given by hand; take those of the DEM sample the run is to be
  compared with. They only scale `I`, not `mu`.
- `Pmin` — points with `P <= Pmin` get `mu = I = 0` and `valid = 0`. At a free
  surface, or for an airborne point, the pressure vanishes and `mu = tau/P`
  is meaningless noise.
- `gammaMin` — likewise for points whose accumulated shear strain `gamma`
  (last column) is below it. `0` keeps everything. `gamma` is the Hencky
  equivalent shear read off the deformation gradient, normalised so that a
  simple shear of amplitude `g` gives `g`; see
  `ScalarTools::equivalentShearStrain`, and read the caveat below before
  relying on it.
- `zzMode` — how `sigma_zz` is obtained: `stored` takes the value carried by
  the Material Point; `elastic` uses the plane-strain elastic expression
  `nu (sigma_xx + sigma_yy)`, and then `Poisson` must follow.

> **What `gamma` can and cannot do.** `mu(I)` is a steady-flow law, reached
> once the material has been sheared by an amount of order one, so filtering
> on `gamma` looks like the way to keep only the points that have got there.
> It is a necessary condition, not a sufficient one: `gamma` records what a
> point has undergone, not what it is doing now. In a column collapse most of
> the deposit has a large `gamma` and is at rest, unloaded and well below the
> yield surface. Measured on the Mohr-Coulomb run, raising `gammaMin` from 0
> to 1 moves the mean `mu` from 0.6206 to 0.6314 and leaves the minimum at
> 0.267 — it barely cleans anything.
>
> What discriminates is the shear rate: keeping `gdot >= 1 s^-1` gives
> `mu = 0.659345` with a standard deviation of 1.3e-7, the yield surface
> exactly. So filter on `gdot` (or on `I`) to select what is flowing, and use
> `gamma` on top of it to make sure the steady state was reached.
>
> For Mohr-Coulomb, `gamma` is in fact irrelevant by construction: the model
> has no hardening, so a point sits on the yield surface as soon as it
> yields, whatever its history. It will matter for `CHCL_DEM`, where a cell
> genuinely needs a shear of order one to reach its critical state.
>
> `gamma` is read off `F`, which knows the current shape of a point but not
> the path that led to it: a shear followed by its reverse gives zero. It is
> not the path length `int gdot dt`. The two agree while the loading is
> monotonic. Getting the exact path length means accumulating it during the
> computation and writing it to the conf-files, which changes their format.

**The full 3x3 stress is rebuilt** from the six components a Material Point
stores: the in-plane block, `sigma_zz`, and the two out-of-plane shears
`sigma_xz` and `sigma_yz`. The invariants of the 2D block alone are not those
of the real state, and the `mu` they give is wrong.

For every classical model the two shears are zero — the out-of-plane
direction is principal, which is what plane strain means for the stress. They
are non-zero only for `CHCL_DEM`, whose periodic cell is genuinely 3D.

Plane strain is still assumed on the velocity gradient, `L_zz = 0`, for the
shear rate.

> Prefer `stored`. Every model of `ConstitutiveModels/` now integrates its
> out-of-plane component, `MohrCoulomb` included — it used to be the one that
> did not, and a run made before that fix gets `sigma_zz = 0`, which is not
> plane strain and biases both `P` and `tau`. The action warns when every
> `sigma_zz` it sees is zero, which is how such an old run announces itself.
> `elastic` remains as a fallback for those, but it only holds while the point
> stays elastic.

> The `plastic` flag of a Material Point is reported in the output, but it is
> not written to the conf-files by `MPMbox::save`, so it always reads back as
> 0. Do not filter on it.

Columns: `iconf t p x y vx vy P tau mu gdot I sig_xx sig_yy sig_zz sig_xy
sig_xz sig_yz rho vol mass plastic valid gamma sinPhi tanPhi`, with

- `P = -tr(Sigma)/3`. The MPM counts a compression as negative; the sign is
  flipped here so that `Scalars` and `DEMScalars` agree and can be plotted
  together.
- `tau`, `gdot` and `I` are defined exactly as in `DEMScalars` below.
- `sinPhi = (S1 - S3)/(S1 + S3)` is the mobilised friction read off the Mohr
  circle of the extreme principal stresses, and `tanPhi` follows from it.

> **Which `mu`?** Three different ratios deserve the name, and they do not
> agree. Measured on the flowing points of the Mohr-Coulomb run with
> `phi = 0.55 rad`, each is exact to six digits:
>
> | column | value | equals |
> |---|---|---|
> | `sinPhi` | 0.522687 | `sin(phi)` |
> | `tanPhi` | 0.613105 | `tan(phi)` |
> | `mu` | 0.659345 | `sqrt(J2)/P` |
>
> `tanPhi` is the shear-to-normal traction ratio on the failure plane — the
> friction coefficient in the everyday sense. `mu` is the tensorial
> definition the mu(I) rheology uses (Jop, Forterre & Pouliquen 2006 write
> `mu = |tau|/P` with `|tau| = sqrt(J2)`); it weighs all three principal
> stresses, so it depends on the intermediate one. For a Mohr-Coulomb
> material in plane strain that intermediate stress is
> `sigma_zz = nu (sigma_xx + sigma_yy)`, which makes `mu` depend on Poisson's
> ratio — an artefact of the model, which does not control `sigma_zz`.
> `tanPhi` is free of it.
>
> Both are reported. What matters is using the same one for the classical run
> and for the double-scale run; `mu` is the column both actions fill, and the
> one the plots use.

#### `DEMScalars` (_string_)`fileName` (_double_)`gammaMin`

Micro-scale scalars of the DEM cells of a double-scale computation — the
entry point of the mu(I) analysis. One line per (configuration, tracked
Material Point).

For every Material Point `p`, the DEM configuration that `MPMbox::run()`
wrote in `DEM_MP<p>/conf<N>` is loaded and `computeSampleData()` is run on it.
Points that were not tracked have no such file and are simply skipped; if no
file at all is found the action says so rather than writing an empty result.

Columns:

| # | name | meaning |
|---|------|---------|
| 1-2 | `iconf t` | configuration number and time |
| 3-5 | `p x y` | Material Point number and position |
| 6-8 | `nbParticles nbActiveInteractions nbBonds` | contact network of the cell |
| 9-13 | `Vcell Vsolid solidFraction Rmean rho_s` | geometry of the cell |
| 14-16 | `P tau mu` | see below |
| 17-19 | `gdot_cell gdot_MP I` | see below |
| 20-25 | `Sig_xx Sig_yy Sig_zz Sig_xy Sig_xz Sig_yz` | raw cell stress |
| 26-27 | `VelMean VelVar` | grain velocity statistics |
| 28-29 | `gamma valid` | accumulated shear of the carrying Material Point, and whether the line is a measurement |
| 30-31 | `sinPhi tanPhi` | mobilised friction of the cell |
| 32-33 | `tau_ps mu_ps` | the same cell, projected on plane strain — see below |

with

- `P = tr(Sig)/3`. PBC3D counts a compression as **positive**, so `P > 0` in a
  flowing granular column — the opposite of the MPM sign convention.
- `tau = sqrt(J2) = sqrt(0.5 dev(Sig):dev(Sig))`
- `mu = tau / P`, left at 0 when `P <= 0`.
- `gdot = sqrt(2 dev(D):dev(D))`, with `D` the symmetric part of a velocity
  gradient. For a simple shear of rate `g` this returns `g`. Two are given:
  `gdot_cell` from the periodic cell's own `Cell.velGrad` (genuinely 3D) and
  `gdot_MP` from the smoothed MPM velocity gradient at the point, under the
  plane-strain assumption `L_zz = 0` that `CHCL_DEM` uses. **They should
  agree**; a discrepancy means the cell did not undergo the transformation the
  MPM asked for.
- `I = gdot_cell d / sqrt(P / rho_s)`, with `d = 2 Rmean`.

> **`mu` or `mu_ps`?** A 2D plane-strain continuum cannot develop `sigma_xz`
> or `sigma_yz` at all — the out-of-plane direction is principal, and on the
> Mohr-Coulomb run those two components are zero on every single line. A
> periodic DEM cell is genuinely 3D and does develop them: on the trial sample,
> `sigma_xz = -128 Pa` and `sigma_yz = 105 Pa` for `P = 883 Pa`.
>
> Since `tau = sqrt(J2)` carries `2(sigma_xy^2 + sigma_xz^2 + sigma_yz^2)`,
> part of any gap between a micro `mu` and a macro one is structural rather
> than physical. `tau_ps` and `mu_ps` are the same cell with its out-of-plane
> shears removed — `P` is untouched, only the deviatoric invariant drops. On
> the trial run, `tau` = 221.7 against `tau_ps` = 145.9, hence `mu` = 0.251
> against `mu_ps` = 0.165.
>
> Use `mu` to describe what the grains actually do. Use `mu_ps` when comparing
> against a plane-strain continuum, so that both sides answer the same
> question. Say which one a figure shows.
>
> To plot it: `gnuplot -e "what='micro'; col_mu=33" ../plot_muI.gp`. `I` is
> unaffected, since it only involves `P`.

> At the very first configuration `gdot_MP` is zero (nothing moves yet) while
> `gdot_cell` is not: the periodic cell carries the residual `hvelGrad` of its
> preparation, which is stored in the sample file.

> **`Scalars` now gets close, but this action still earns its place.** Until
> September 2026 `CHCL_DEM` copied only five of the six stress components onto
> the Material Point — `sigma_xz` and `sigma_yz` had nowhere to go — so `tau`
> and `mu` read from a conf-file were badly wrong (`mu = 0.163` against the
> cell's `0.251` on the trial run). `MaterialPoint` now carries them, and the
> two actions agree on `mu` to about 0.4 %.
>
> The residual gap is not a bug: `CHCL_DEM` stores the stress averaged over
> the tail of the MPM step (`CHCL.rateAverage`), while the DEM conf-file holds
> the instantaneous one. The averaged value is arguably the better measure.
>
> What only this action can give: the contact network
> (`nbActiveInteractions`, `nbBonds`), the geometry of the cell (`Rmean`,
> `Vsolid`, `Vcell`, `solidFraction`), the grain velocity statistics
> (`VelMean`, `VelVar`), and the `gdot_cell` / `gdot_MP` cross-check. For a
> paper whose point is that the micro scale drives the macro one, those are
> the evidence.

> `DEMScalars` needs the Material Points to have been tracked during the
> computation, through `select_tracked_MP` in the input file. Beware that
> `select_tracked_MP ALL` **does not work**: `operator>>` of
> `ElementSelector` (in toofus) has no branch for `"ALL"`, so it falls in the
> final `else` and is silently downgraded to `"NONE"`. Use a box covering the
> whole grid instead, e.g. `select_tracked_MP BOX 0 0 1 1`.

## Adding an action

1. Write `Post/MyAction.hpp` and `Post/MyAction.cpp`, deriving from
   `PostProcessor`: `read()` takes the parameters that follow the name on the
   `Post` line, `begin()` / `exec()` / `end()` are called before the loop, once
   per configuration, and after the loop.
2. Register it in `PostSession::ExplicitRegistrations()`, the same way
   `MPMbox::ExplicitRegistrations()` registers the Spies.

The CMake glob picks up `Post/*.cpp` on its own, so nothing else is needed.
Inside `exec()`, the configuration is `session->Conf`, its smoothed fields are
`session->Smoothed`, its number and time are `session->confNum` and
`session->confTime`, and `session->loadDEMConf(p, dem)` gives the DEM cell of a
Material Point.
