#include <ctime>

#include <tclap/CmdLine.h>

#include "Core/MPMbox.hpp"
#include "ExecChrono.hpp"
#include "Spies/Spy.hpp"
#include "stackTracer.hpp"

//#include "spdlog/sinks/stdout_color_sinks.h"
//#include "spdlog/spdlog.h"

ExecChrono SimuChrono;
MPMbox* SimuHandler;

void mySigHandler(int sig) {
  std::cerr << std::endl << std::endl;
  SimuChrono.stop();

  for (size_t s = 0; s < SimuHandler->Spies.size(); ++s) {
    SimuHandler->Spies[s]->end();
  }

	/*
	std::cerr << "Timer data saved in 'stopped' file\n";
  outputManager tmp_out("stopped");
  tmp_out.writeFile();
  tmp_out.printTimeTable();
  */
	
  // SimuHandler->save_state("data", SimuHandler->step);
  StackTracer::defaultSigHandler(sig);
}

int main(int argc, char** argv) {
  INIT_TIMERS();
  time_t rawtime;
  time(&rawtime);
  printf("The current local time is: %s", ctime(&rawtime));

  std::string confFileName;
  int nbThreads = 1;
  int verboseLevel = 0;

  try {
    TCLAP::CmdLine cmd("This is the command line interface for MPMbox", ' ', "0.3");
    TCLAP::UnlabeledValueArg<std::string> nameArg("input", "Name of the conf-file", true, "conf0", "conf-file");
    TCLAP::ValueArg<int> nbThreadsArg("j", "nbThreads", "Number of threads to be used", false, 1, "int");
    TCLAP::ValueArg<int> verboseArg("v", "verbose", "Verbose level", false, 4, "int");

    cmd.add(nameArg);
    cmd.add(nbThreadsArg);
    cmd.add(verboseArg);

    cmd.parse(argc, argv);

    confFileName = nameArg.getValue();
    nbThreads = nbThreadsArg.getValue();
    verboseLevel = verboseArg.getValue();
  } catch (TCLAP::ArgException& e) {
    std::cerr << "Tclap error: " << e.error() << " for argument " << e.argId() << std::endl;
  }

  MPMbox Simulation;
  Simulation.setVerboseLevel(verboseLevel);
  Simulation.showAppBanner();

  StackTracer::initSignals(mySigHandler);
  SimuHandler = &Simulation;

#ifdef _OPENMP
  omp_set_num_threads(nbThreads);
  Logger::info("OpenMP acceleration (Number of threads = {})", nbThreads);
#else
  Logger::info("No multithreading (Number of threads = {})", nbThreads);
#endif

  Simulation.read(confFileName.c_str());

  Logger::info("COMPUTATION STARTS");

  SimuChrono.start();
  Simulation.init();
  Simulation.run();

  SimuChrono.stop();
  Logger::info("COMPUTATION NORMALLY STOPPED");

  PRINT_TIMERS("mpmbox");

  return 0;
}
