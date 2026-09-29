#pragma once

#include <iostream>

class PostSession;

//
// Base class of the post-processing actions of mpmpost.
//
// One action corresponds to one line
//
//     Post <Name> <parameters...>
//
// of the command file. The factory builds the object from <Name>, then
// read() is given the rest of the line. The object is called by
// PostSession::run() as:
//
//     begin()   once, before the loop over the configurations
//     exec()    once per loaded configuration
//     end()     once, after the loop
//
// Inside exec(), the configuration being processed is session->Conf, its
// smoothed fields are session->Data and its number is session->confNum.
//
// To add an action: derive from this class, and register the derived class
// in PostSession::ExplicitRegistrations().
//
struct PostProcessor {
  PostSession *session{nullptr};

  virtual void plug(PostSession *S);

  // Reads the parameters that follow the name on the 'Post' line.
  virtual void read(std::istream &is) = 0;

  virtual void begin();
  virtual void exec() = 0;
  virtual void end();

  virtual ~PostProcessor();
};
