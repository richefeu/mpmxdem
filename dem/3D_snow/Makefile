COMMON_FOLDER = ../../common


UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin) # ==========================================

# The compiler to be used
CXX = clang++
# The list of flags passed to the compiler
CXXFLAGS = -O3 -Wall -std=c++11 -I $(COMMON_FOLDER)
# Link flags for OpenGL and glut
GLLINK = -framework OpenGL -framework GLUT

else # ==============================================================

# The compiler to be used
CXX = g++
# The list of flags passed to the compiler
CXXFLAGS = -O3 -Wall -std=c++11
# Link flags for OpenGL and glut
GLLINK = -lGLU -lGL -L/usr/X11R6/lib -lglut -lXmu -lXext -lX11 -lXi

endif # =============================================================

# The list of source files
SOURCES = PBC3D.cpp Particle.cpp Sphere.cpp Interaction.cpp Loading.cpp PeriodicCell.cpp

# Each cpp file listed below corresponds to an object file
OBJECTS = $(SOURCES:%.cpp=%.o)

# Each cpp file listed below corresponds to a header file
HEADERS = $(SOURCES:%.cpp=%.hpp)

# All source files (listed in SOURCES) will be compiled into an object file 
# with the following command
%.o:%.cpp
	$(CXX) $(CXXFLAGS) -c $<

.PHONY: all lib clean

all: run

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
see: see.cpp see.hpp $(HEADERS) $(OBJECTS)
	$(CXX) $(CXXFLAGS) -c see.cpp -Wno-deprecated
	$(CXX) $(CXXFLAGS) -o $@ $@.o $(OBJECTS) $(GLLINK)

