TOOFUS_PATH = ~/toofus
SPDLOG_PATH = /usr/local/include
CHCL_LIB_PATH = ../../dem/3D_sandstone
CHCL_LIB = libPBC3D
LIB = libMPMbox

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)

CXX = g++-12
CXXFLAGS = -fopenmp -O3 -std=c++17 -Wall -Wextra -Wshadow -Wconversion -pedantic \
-I $(SPDLOG_PATH) -I $(CHCL_LIB_PATH) -I $(TOOFUS_PATH) -I.

OPENMP_LIB = -l gomp

# Link flags for OpenGL and glut
#GLUTFLAGS = `pkg-config --cflags glut`
#GLUTLIBS = `pkg-config --libs glut` -framework OpenGL
GLUTFLAGS = -I/usr/include
GLUTLIBS = -L/usr/X11R6/lib -lX11 -lXi -lglut -framework OpenGL

else

CXX = g++
CXXFLAGS = -fopenmp -O3 -std=c++17 \
-I $(SPDLOG_PATH) -I $(CHCL_LIB_PATH) -I $(TOOFUS_PATH) -I.

OPENMP_LIB = -l gomp

# Link flags for OpenGL and glut
GLUTFLAGS = -I/usr/include
GLUTLIBS = -L/usr/X11R6/lib -lX11 -lXi -lglut -lGL -lGLU -lm -lpthread

endif

#========================================================================

SRCS = \
$(wildcard Core/*.cpp) \
$(wildcard BoundaryForceLaw/*.cpp) \
$(wildcard Commands/*.cpp) \
$(wildcard ConstitutiveModels/*.cpp) \
$(wildcard Obstacles/*.cpp) \
$(wildcard OneStep/*.cpp) \
$(wildcard ShapeFunctions/*.cpp) \
$(wildcard Spies/*.cpp)

OBJS = $(SRCS:.cpp=.o)

#========================================================================

%.o:%.cpp
	@echo "\033[0;32m-> COMPILING" $@ "\033[0m"
	$(CXX) $(CXXFLAGS) -c $< -o $@
	
all: lib mpmbox see

lib: lib/$(LIB).a
	
lib/$(LIB).a: $(OBJS)
	@echo "\033[0;32m-> BUILDING" $@ "\033[0m"
	@mkdir -p lib
	ar rs $@ $^

mpmbox: bin/mpmbox
bin/mpmbox: Runners/run.cpp $(OBJS)
	@echo "\033[0;32m-> BUILDING" $@ "\033[0m"
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) -c $< -o $(patsubst %.cpp, %.o, $<)
	$(CXX) $(OPENMP_LIB) -o $@ Runners/run.o lib/$(LIB).a $(CHCL_LIB_PATH)/$(CHCL_LIB).a

see: bin/see
	
bin/see: See/see.cpp $(OBJS)
	@echo "\033[0;32m-> BUILDING" $@ "\033[0m"
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(GLUTFLAGS) -c $< -o $(patsubst %.cpp, %.o, $<)
	$(CXX) $(OPENMP_LIB) $(GLUTLIBS) -o $@ See/see.o lib/$(LIB).a $(CHCL_LIB_PATH)/$(CHCL_LIB).a

clean:
	rm -f $(OBJS)

clean+: clean
	rm -rf lib
	rm -rf bin
