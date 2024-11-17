# For the compilation, toofus is required. 
# It can be installed with the following command: 
#  cd ~ && rm -rf toofus && git clone https://github.com/richefeu/toofus.git && cd -


UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Darwin) # ==========================================

# The compiler to be used
CXX = g++-14
# The list of flags passed to the compiler
CXXFLAGS = -O3 -Wall -Wextra -Wshadow -Wconversion -pedantic -std=c++17 -I ~/toofus -I /usr/include
# Link flags for OpenGL and glut
#GLFLAGS = `pkg-config --cflags glut`
#GLLINK = `pkg-config --libs glut` -framework OpenGL
GLFLAGS = -I /usr/include
GLLINK = -L/usr/X11R6/lib -lX11 -lXi -lglut -framework OpenGL

else # ==============================================================

# The compiler to be used
CXX = g++
# The list of flags passed to the compiler
CXXFLAGS = -O3 -Wall -std=c++17 -I ~/toofus -I /usr/include -fopenmp
# Link flags for OpenGL and glut
GLLINK = -L/usr/X11R6/lib -lX11 -lXi -lglut -lGL -lGLU -lm -lpthread
#GLFLAGS = `pkg-config --cflags glut`
#GLLINK = `pkg-config --libs glut` -framework OpenGL

endif # =============================================================

# The list of source files (only files for the library)
SOURCES = PBC3D.cpp Particle.cpp Interaction.cpp Loading.cpp PeriodicCell.cpp

# Each cpp file listed below corresponds to an object file
OBJECTS = $(SOURCES:%.cpp=%.o)

# Each cpp file listed below corresponds to a header file
HEADERS = $(SOURCES:%.cpp=%.hpp)

# All source files (listed in SOURCES) will be compiled into an object file 
# with the following command
%.o:%.cpp
	@echo "\033[0;32m-> COMPILING" $@ "\033[0m"
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: clean

all: lib run see

lib: $(OBJECTS)
	@echo "\033[0;32m-> BUILDING/UPDATING" $@ "\033[0m"
	ar rs libPBC3D.a $(OBJECTS)

testLaw: testLaw.cpp lib
	@echo "\033[0;32m-> BUILDING" $@ "\033[0m"
	$(CXX) $(CXXFLAGS) -c testLaw.cpp
	$(CXX) $(CXXFLAGS) -o $@ $@.o libPBC3D.a
	
analyse_cluster_breaks: analyse_cluster_breaks.cpp lib
	@echo "\033[0;32m-> BUILDING" $@ "\033[0m"
	$(CXX) $(CXXFLAGS) -c analyse_cluster_breaks.cpp
	$(CXX) $(CXXFLAGS) -o $@ $@.o libPBC3D.a	
	
# The application that runs a simulation
run: run.cpp lib
	@echo "\033[0;32m-> BUILDING" $@ "\033[0m"
	$(CXX) $(CXXFLAGS) -c run.cpp
	$(CXX) $(CXXFLAGS) -o $@ $@.o libPBC3D.a

# The application that visualizes the conf files
see: see.cpp lib
	@echo "\033[0;32m-> BUILDING" $@ "\033[0m"
	$(CXX) $(CXXFLAGS) -c see.cpp -Wno-deprecated $(GLFLAGS)
	$(CXX) $(CXXFLAGS) -o $@ $@.o libPBC3D.a $(GLLINK) 

clean:
	rm -f *.o libPBC3D.a *.gch run see
