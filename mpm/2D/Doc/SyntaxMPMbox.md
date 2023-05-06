## Core settings

**`dt`** (_double_)time_step

**`t`** (_double_)current_time


## Commands

**`set_node_grid`**   ...

**`set_MP_grid`**  (_int_)`group` (_string_)`modelName` (_double_)`massDensity` (_double_)`x0`  (_double_)`y0`  (_double_)`x1`  (_double_)`y1` (_double_)`sizeOfBoxes`

> Place Material Points on a grid.

**`set_MP_polygon`** (_int_)`group`  (_int_)`nbVertices`  (_string_)`modelName`  (_double_)`massDensity` (_double_)`sizeOfBoxes`  (... (_double_)`x` (_double_)`y` ...)

> The part (...) is repeated `nbVertices` times. 

**`add_MP_ShallowPath`** (_int_)group  (_int_)nbPathPoints (_double_)height (_string_)modelName (_double_)rho (_double_)size (...list of nbPathPoints (_vec2r_)point...)

**`move_MP`**   (_double_)x0  (_double_)y0  (_double_)dx  (_double_)dy  (_double_)theta

> Note that a negative value of theta rotates clockwise.

**`reset_model`**  (_int_)group  (_string__)modelName  (_double_)rho  (_double_)x0 (_double_)y0 (_double_)x1 (_double_)y1

**`set_BC_column`** (_int_)column_num (_int_)line0  (_int_)line1  (0/1)Xfixed  (0/1)Yfixed

**`set_BC_line`** (_int_)line_num (_int_)column0  (_int_)column1  (0/1)Xfixed  (0/1)Yfixed

**`set_K0_stress`** (_double_)`Poisson`  (_double_)`MassDensity`



## Shape Functions

**`ShapeFunction`** (_string_)shapeFunction

shapeFunction can be either `Linear`, `RegularQuadLinear` or `BSpline`

## Splitting

**`splitting`**  (_bool_)Active (_bool_)splitTouchingMP (_double_)SplitValue

## Constitutive Model
**`model`** (_string_)modelName (_string_)modelRef (...list of parameters that depend on modelName...)

List of parameters for each modelName:

  - `HookeElasticity`: (_double_)Young (_double_)Poisson
  - `VonMisesElastoPlasticity`: (_double_)Young (_double_)Poisson (_double_)PlasticYieldStress
  - `MohrCoulomb`: (_double_)Young (_double_)Poisson (_double_)FrictionAngle (_double_)Cohesion (_double_)DilatancyAngle
  - `CHCL_DEM`:  (_string_)conf-filename




## Obstacles (Rigid Bodies)

**`Obstacle`** (_string_)type (...list of parameters that depend on type...)

List of parameters for each type:

  - `Line`: (_double_)x0 (_double_)y0 (_double_)x1 (_double_)y1
  - `Circle`: (_vec2r_)center (_double_)R

## Boundary Force Laws



## Spies (Processing)

**`Spy`** (_string_)`spyName` (...list of parameters that depend on spyName...)

List of parameters for each spyName:

  - `Work`:  (_int_)`recordPeriod`  (_string_)`fileName` (_double_)`Xmin` (_double_)`Xmax` (_int_)`nbBins`


## Schedulers

**`GravityRampe`** (_vec2r_)`gravityFrom`  (_double_)`rampStart` (_vec2r_)`gravityTo` (_double_)`rampStop`

**`PICDissipation`** (*double*)`rateOfFLIP` (*double*)`endTime`

**`ReactivateCHCLBonds`**

**`RemoveMaterialPoint`**

**`RemoveObstacle`**



  