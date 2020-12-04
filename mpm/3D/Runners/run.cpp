#include "../Core/MPMbox.hpp"
#include "../../../common/stackTracer.hpp"
#include "../../../common/ExecChrono.hpp"

ExecChrono SimuChrono;
MPMbox * SimuHandler;

void mySigHandler(int sig)
{
	std::cerr << std::endl << std::endl;
	SimuChrono.stop();
	//to be used later on
	// for (size_t s = 0 ; s < SimuHandler->Spies.size() ; ++s) {
	// 	SimuHandler->Spies[s]->end();
	// }
	// SimuHandler->save_state("data", SimuHandler->step);

	StackTracer::defaultSigHandler(sig);

}


int main(int argc, char ** argv)
{
	MPMbox Simulation;
	Simulation.showAppBanner();

	StackTracer::initSignals(mySigHandler);
	SimuHandler = &Simulation;

	if (argc == 1) {
		std::cout << "Usage: " << argv[0] << " simulationFileName" << std::endl;
		return 0;
	}
	else Simulation.read(argv[1]);

	time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    std::cout << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-' <<  now->tm_mday << "\t" << now->tm_hour<<":"<<now->tm_min<<":"<<now->tm_sec<<std::endl;

	clock_t begin = std::clock();

	Simulation.init(argv[1]);
	Simulation.save_vtk_grid();
	//Simulation.save_vtk_obst();
	Simulation.run();

	clock_t end = std::clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	std::cout<<"Elapsed time (m): "<<elapsed_secs/60<<std::endl;

	return 0;
}
