Quick-start guid
================

What is ``MPMbox``?
-------------------

``MPMbox`` is a Material Point Method (MPM) code developed in C++. Compared to more established methods like the Finite Element Method (FEM), MPM is relatively new. Consequently, there are fewer MPM codes available, and those that exist vary significantly in their design and specialization.

The code has two key features:

(i) It incorporates interactions with rigid bodies, similar to the Discrete Element Method (DEM). This enables the definition and control of rigid boundary conditions and allows rigid bodies to interact with deformable masses.

(ii) It can be integrated with DEM simulations to perform multi-scale simulations. For that, it includes a DEM code that implements periodic boundary conditions.


Compilation with cmake
----------------------

The preferred method for building the project is by using CMake, a widely-used build system. Follow these steps to compile the project:

1. Create a new directory called build (you can choose any name) in the project's root directory:

.. code-block:: sh

   mkdir build

2. Navigate into the build directory:

.. code-block:: sh
 
	 cd build
	 
	 
3. Run CMake configuration command from within the build directory. This command sets up the build environment and generates the necessary build files based on the CMake configuration files present in the project:
		
.. code-block:: sh
 
    cmake ..

4. Then simply use the Makefile to build everything:

.. code-block:: sh

   make -j


Running a simulation
--------------------


To run a simulation, a configuration file (or an input file) has to be written. The format of such a file is described in the section Syntax for conf-files. We show here a simple example simulating cantilever elastic beam.

.. code-block:: text
   :caption: input.txt
   
   Rockable 20-02-2017
   t 0
   tmax 0.06
   dt 1e-6
   interVerlet 0.01
   interConf 0.01
   
   DVerlet 0.08
   dVerlet 0.02
   density 0 2700
   density 1 2700
   
   forceLaw Avalanches
   knContact 0 1 1e6
   en2Contact 0 1 0.05
   ktContact 0 1 1e7
   muContact 0 1 0.4
   krContact 0 1 1e7
   murContact 0 1 0.0
   
   iconf 0
   nDriven 1
   shapeFile SphereAndPlan.shp
   Particles 2
   Plan 0 0 1 0 -0.05 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0
   Sphere 1 0 1 -0.5 0.5 0 3.69 -3.29 0 0 0 0 0.707 0 0.707 0 0 0 -50.52 0 0 0
   
The shape-file as described in the section Syntax for shape-files is a file named SphereAndPlan.shp with the following content:

.. code-block:: text
   :caption: SphereAndPlan.sph
   
   <
   name Plan
   radius 0.05
   preCompDone y
   nv 4
   2 0 0.5
   2 0 -0.5
   -2 0 -0.5
   -2 0 0.5
   ne 4
   0 1
   1 2
   2 3
   3 0
   nf 1
   4 0 1 2 3
   obb.extent 2.0 0.05 0.5
   obb.e1 1 0 0
   obb.e2 0 1 0
   obb.e3 0 0 1
   obb.center 0 0 0
   volume 1
   I/m 1 1 1
   >
   
   <
   name Sphere
   radius 0.08
   preCompDone y
   nv 1
   0 0 0
   ne 0
   nf 0
   obb.extent 1 1 1
   obb.e1 1 0 0
   obb.e2 0 1 0
   obb.e3 0 0 1
   obb.center 0 0 0
   volume 0.004021
   I/m 0.00493333 0.00493333 0.0032
   >

Supposing that the executable named ``rockable`` stands in the same folder as the configuration and shape files, the simulation is launched that way:

.. code-block:: sh
   
   ./rockable bouncingSphere.txt

If the executable has been compiled with openMP abilities, the number of threads can be set with the option ``-j``, for example:

.. code-block:: sh

   ./rockable bouncingSphere.txt -j 24

In this particular example, it is clearly not a good idea to use so much threads because the number of particles is to small and the computation duration will be worst.

The verbosity of logs is set with a number that way:

.. code-block:: sh

  ./rockable bouncingSphere.txt -v 6

Highest number corresponds highest verbosity: ``trace`` = 6, ``debug`` = 5, ``info`` = 4, ``warn`` = 3, ``err`` = 2, ``critical`` = 1, ``off`` = 0

If the files produced by a computation (``conf*``, ``kineticEnergy.txt``, ``perf.txt``, and ``staticBalance.txt``) have to be deleted, ``rockable`` can do the job.

.. code-block:: sh

  ./rockable -c


Visualising the simulations
---------------------------

Normally, the application ``see`` has been built as the same time than ``rockable``. 
The application ``see`` needs ``freeglut``, the simplest way to use openMP and display 3D things.

