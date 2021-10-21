#include <ctime>

#include "Core/MPMbox.hpp"
#include "ExecChrono.hpp"
#include "Spies/Spy.hpp"
#include "stackTracer.hpp"

ExecChrono SimuChrono;
MPMbox* SimuHandler;

void mySigHandler(int sig) {
  std::cerr << std::endl << std::endl;
  SimuChrono.stop();

  for (size_t s = 0; s < SimuHandler->Spies.size(); ++s) {
    SimuHandler->Spies[s]->end();
  }
  // SimuHandler->save_state("data", SimuHandler->step);
  StackTracer::defaultSigHandler(sig);
}

int main(int argc, char** argv) {

  time_t rawtime;
  time(&rawtime);
  printf("The current local time is: %s", ctime(&rawtime));
  MPMbox Simulation;
  Simulation.showAppBanner();

  StackTracer::initSignals(mySigHandler);
  SimuHandler = &Simulation;

  if (argc == 1) {
    std::cout << "Usage: " << argv[0] << " simulationFileName" << std::endl;
    return 0;
  } else
    Simulation.read(argv[1]);

  //printf("Simultation::read done");
  SimuChrono.start();
  Simulation.init();
  //printf("Simulation::init done");
  Simulation.run();
  //printf("run finished");

  SimuChrono.stop();
  return 0;
}
