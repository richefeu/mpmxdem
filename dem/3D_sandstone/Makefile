#For the compilation, toofus is required.
#It can be installed with the following command:
#cd ~&&rm - rf toofus&& git clone https:  // github.com/richefeu/toofus.git && cd -

UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Darwin) # ==========================================

  #The compiler to be used
  CXX = g++-15
  CXXFLAGS = -O3 -Wall -Wextra -Wshadow -std=c++17 -I ./toofus

	# on apple, compile freeglut from source with this configuration:
	# mkdir build
	# cd build
	# cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local -DFREEGLUT_COCOA=ON -DFREEGLUT_BUILD_DEMOS=OFF -DCMAKE_BUILD_TYPE=Release
  # make
	# sudo make install
	
  GLLINK = /usr/local/lib/libglut.a -framework Cocoa -framework CoreVideo -framework OpenGL -framework IOKit
	GLFLAGS = -I/usr/local/include

else # ==============================================================

  CXX = g++  
  CXXFLAGS = -O3 -Wall -std=c++17 -I ./toofus -I /usr/include -fopenmp
  GLLINK = -L/usr/X11R6/lib -lX11 -lXi -lglut -lGL -lGLU -lm -lpthread

endif # =============================================================

#Ensure the toofus directory exists
ifeq ($(wildcard ./toofus),)
$(info Cloning ToOfUs repository)
$(shell git clone https://github.com/richefeu/toofus.git > /dev/null 2>&1)
endif

#The list of source files(only files for the library)
SOURCES = PBC3D.cpp Particle.cpp Interaction.cpp Loading.cpp PeriodicCell.cpp

#Each cpp file listed below corresponds to an object file
OBJECTS = $(SOURCES:%.cpp=%.o)

#Each cpp file listed below corresponds to a header file
HEADERS = $(SOURCES:%.cpp=%.hpp)

#All source files(listed in SOURCES) will be compiled into an object file
#with the following command
%.o:%.cpp
	@echo "\033[0;32m-> COMPILING" $@ "\033[0m"
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: clean

all: run see

libPBC3D.a: $(OBJECTS)
	@echo "\033[0;32m-> BUILDING/UPDATING" $@ "\033[0m"
	ar rs libPBC3D.a $(OBJECTS)

testLaw: testLaw.cpp libPBC3D.a
	@echo "\033[0;32m-> BUILDING" $@ "\033[0m"
	$(CXX) $(CXXFLAGS) -c testLaw.cpp
	$(CXX) $(CXXFLAGS) -o $@ $@.o libPBC3D.a
	
analyse_cluster_breaks: analyse_cluster_breaks.cpp libPBC3D.a
	@echo "\033[0;32m-> BUILDING" $@ "\033[0m"
	$(CXX) $(CXXFLAGS) -c analyse_cluster_breaks.cpp
	$(CXX) $(CXXFLAGS) -o $@ $@.o libPBC3D.a

#The application that runs a simulation
run: run.cpp libPBC3D.a
	@echo "\033[0;32m-> BUILDING" $@ "\033[0m"
	$(CXX) $(CXXFLAGS) -c run.cpp
	$(CXX) $(CXXFLAGS) -o $@ $@.o libPBC3D.a

#The application that visualizes the conf files
see: see.cpp libPBC3D.a graphGL.hpp
	@echo "\033[0;32m-> BUILDING" $@ "\033[0m"
	$(CXX) $(CXXFLAGS) -c $< -o see.o $(GLFLAGS)
	$(CXX) -o $@ see.o libPBC3D.a $(GLLINK) 

clean:
	rm -f *.o libPBC3D.a *.gch run see
