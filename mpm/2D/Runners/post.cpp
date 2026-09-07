#include <cstdlib>
#include <iostream>

#include "profiler.hpp"

#include "Post/PostSession.hpp"

//
// mpmpost -- post-processing of an MPMbox computation.
//
// Usage:  mpmpost [commandFile]        (default: post.txt)
//
// The tool loads the configuration files one after the other, exactly the way
// 'see' does, and applies to each of them the actions listed in the command
// file. See Doc/SyntaxMPMpost.md for the syntax.
//
int main(int argc, char **argv) {
  // Same reason as in See/see.cpp: MPMbox::postProcess reaches functions that
  // carry a START_TIMER, and START_TIMER dereferences the 'current' timer that
  // INIT_TIMERS allocates. Without this the tool dies on a null pointer at the
  // first configuration.
  INIT_TIMERS();

  const char *commandFile = (argc > 1) ? argv[1] : "post.txt";

  PostSession session;
  if (!session.read(commandFile)) { return EXIT_FAILURE; }

  std::cout << "Command file : " << commandFile << '\n';
  std::cout << "Source folder: " << session.sourceFolder << '\n';
  std::cout << "Result folder: " << session.resultFolder << '\n';
  std::cout << "Configurations: from " << session.confFirst << " to " << session.confLast << " by "
            << session.confStride << '\n';
  std::cout << "Fields        : " << (session.smoothing ? "smoothed on the grid" : "raw Material Point values")
            << '\n';

  session.run();

  return EXIT_SUCCESS;
}
