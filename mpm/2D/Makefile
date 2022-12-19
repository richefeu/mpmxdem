TOOFUS_PATH = ~/toofus
SPDLOG_PATH = /usr/local/include
CHCL_LIB_PATH = ../../dem/3D_sandstone
CHCL_LIB = libPBC3D
LIB = libMPMbox

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)

CXX = g++-12
CXXFLAGS = -O3 -std=c++17 -Wall -Wextra -Wshadow -Wconversion \
-I $(SPDLOG_PATH) -I $(CHCL_LIB_PATH) -I $(TOOFUS_PATH) -I.

# Link flags for OpenGL and glut
GLUTFLAGS = `pkg-config --cflags glut`
GLUTLIBS = `pkg-config --libs glut` -framework OpenGL
	
else

CXX = g++
CXXFLAGS = -O3 -std=c++17 -Wall -Wextra \
-I $(SPDLOG_PATH) -I $(CHCL_LIB_PATH) -I $(TOOFUS_PATH) -I.

# Link flags for OpenGL and glut
#GLUTFLAGS = 
GLUTLIBS = -lGLU -lGL -L/usr/X11R6/lib -lglut -lXmu -lXext -lX11 -lXi

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
	$(CXX) $(CXXFLAGS) -c $< -o $@
	
all: $(LIB).a	bin/mpmbox bin/see

$(LIB).a: $(OBJS)
	@mkdir -p lib
	ar rvs lib/$@ $^

bin/mpmbox: Runners/run.cpp $(OBJS)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) -c $< -o $(patsubst %.cpp, %.o, $<)
	$(CXX) -o $@ Runners/run.o lib/$(LIB).a $(CHCL_LIB_PATH)/$(CHCL_LIB).a
	
bin/see: See/see.cpp $(OBJS)
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $(GLUTFLAGS) -c $< -o $(patsubst %.cpp, %.o, $<)
	$(CXX) $(GLUTLIBS) -o $@ See/see.o lib/$(LIB).a $(CHCL_LIB_PATH)/$(CHCL_LIB).a

clean:
	rm -f $(OBJS)

clean+: clean
	rm -rf lib
	rm -rf bin