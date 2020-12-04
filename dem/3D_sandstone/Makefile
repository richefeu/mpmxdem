
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin) # ==========================================

# The compiler to be used
CXX = clang++
# The list of flags passed to the compiler
CXXFLAGS = -O3 -Wall -std=c++11
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
SOURCES = PBC3D.cpp Particle.cpp Interaction.cpp Loading.cpp PeriodicCell.cpp \
quat.cpp vec3.cpp mat9.cpp ColorTable.cpp

# Each cpp file listed below corresponds to an object file
OBJECTS = $(SOURCES:%.cpp=%.o)

# Each cpp file listed below corresponds to a header file
HEADERS = $(SOURCES:%.cpp=%.hpp)

# All source files (listed in SOURCES) will be compiled into an object file 
# with the following command
%.o:%.cpp
	$(CXX) $(CXXFLAGS) -c $<

.PHONY: all lib clean

all: run see

clean:
	rm -f *.o libPBC3D.a *.gch run see

lib: $(SOURCES) $(HEADERS) 
	$(CXX) $(CXXFLAGS) -c $(SOURCES)
	ar rvs libPBC3D.a $(OBJECTS)

# The application that runs a simulation
run: run.cpp lib
	$(CXX) $(CXXFLAGS) -c run.cpp
	$(CXX) $(CXXFLAGS) -o $@ $@.o $(OBJECTS)

stressProcess: stressProcess.cpp mat9.cpp vec3.cpp
	$(CXX) $(CXXFLAGS) -c stressProcess.cpp mat9.cpp vec3.cpp
	$(CXX) $(CXXFLAGS) -o stressProcess stressProcess.o mat9.o vec3.o

confInfo: confInfo.cpp $(HEADERS) $(OBJECTS)
	$(CXX) $(CXXFLAGS) -c confInfo.cpp
	$(CXX) $(CXXFLAGS) -o $@ $@.o $(OBJECTS)
	
confOrientations: confOrientations.cpp $(HEADERS) $(OBJECTS)
	$(CXX) $(CXXFLAGS) -c confOrientations.cpp
	$(CXX) $(CXXFLAGS) -o $@ $@.o $(OBJECTS)

# The application that converts a conf to a list of position + radius
convertListDisks: convertListDisks.cpp $(HEADERS) $(OBJECTS)
	$(CXX) $(CXXFLAGS) -c convertListDisks.cpp
	$(CXX) $(CXXFLAGS) -o $@ $@.o $(OBJECTS)

conf2revini: conf2revini.cpp $(HEADERS) $(OBJECTS)
	$(CXX) $(CXXFLAGS) -c conf2revini.cpp
	$(CXX) $(CXXFLAGS) -o $@ $@.o $(OBJECTS)

# The application that visualizes the conf files
see: see.cpp see.hpp graphGL.hpp $(HEADERS) $(OBJECTS)
	$(CXX) $(CXXFLAGS) -c see.cpp -Wno-deprecated
	$(CXX) $(CXXFLAGS) -o $@ $@.o $(OBJECTS) $(GLLINK)

