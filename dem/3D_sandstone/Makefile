# For the compilation, toofus is required. 
# It can be installed with the following command: 
#  cd ~ && rm -rf toofus && git clone https://github.com/richefeu/toofus.git && cd -


UNAME_S := $(shell uname -s)
#vpath %.hpp ../../common/
ifeq ($(UNAME_S),Darwin) # ==========================================

# The compiler to be used
CXX = g++-12
# The list of flags passed to the compiler
CXXFLAGS = -O3 -Wall -std=c++17 -I ~/toofus -I /usr/include -DENABLE_PROFILING
# Link flags for OpenGL and glut
GLFLAGS = `pkg-config --cflags glut`
GLLINK = `pkg-config --libs glut` -framework OpenGL

else # ==============================================================

# The compiler to be used
CXX = g++
# The list of flags passed to the compiler
CXXFLAGS = -O3 -Wall -std=c++17 -I ~/toofus -I /usr/include -fopenmp
# Link flags for OpenGL and glut
#LDFLAGS = -lGL -lGLU -lglut
GLLINK = -L/usr/X11R6/lib -lX11 -lXi -lglut -lGL -lGLU -lm -lpthread
#GLFLAGS = `pkg-config --cflags glut`
#GLLINK = `pkg-config --libs glut` -framework OpenGL

endif # =============================================================

# The list of source files
SOURCES = PBC3D.cpp Particle.cpp Interaction.cpp Loading.cpp PeriodicCell.cpp

# Each cpp file listed below corresponds to an object file
OBJECTS = $(SOURCES:%.cpp=%.o)

# Each cpp file listed below corresponds to a header file
HEADERS = $(SOURCES:%.cpp=%.hpp)

# All source files (listed in SOURCES) will be compiled into an object file 
# with the following command
%.o:%.cpp
	$(CXX) $(CXXFLAGS) -c $<

.PHONY: all lib clean

all: run see # mpstest

clean:
	rm -f *.o libPBC3D.a *.gch run see

lib: $(SOURCES) $(HEADERS) 
	$(CXX) $(CXXFLAGS) -c $(SOURCES) 
	ar rvs libPBC3D.a $(OBJECTS)

# The application that runs a simulation
run: run.cpp lib
	$(CXX) $(CXXFLAGS) -c run.cpp 
	$(CXX) $(CXXFLAGS) -o $@ $@.o $(OBJECTS)

# The application that visualizes the conf files
see: see.cpp see.hpp graphGL.hpp $(HEADERS) $(OBJECTS)
	$(CXX) $(CXXFLAGS)  -c see.cpp -Wno-deprecated $(GLFLAGS)
	$(CXX) $(CXXFLAGS)  -o $@ $@.o $(OBJECTS) $(GLLINK) 

# The application that stops fluctuations
stop: stop.cpp lib
	$(CXX) $(CXXFLAGS)  -c stop.cpp
	$(CXX) $(CXXFLAGS)  -o $@ $@.o $(OBJECTS)


mpstest : mpstest.cpp lib 
	$(CXX) $(CXXFLAGS)  -c mpstest.cpp
	$(CXX) $(CXXFLAGS)  -o $@ $@.o $(OBJECTS)
