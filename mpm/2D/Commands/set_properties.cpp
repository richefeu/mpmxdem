#include "set_properties.hpp"

#include <Core/MPMbox.hpp>

#include <factory.hpp>
static Registrar<Command, set_properties> registrar("set_properties");

void set_properties::read(std::istream& is) {
  std::cout << "*****PROPERTIES ARE NO LONGER SET WITH 'set_properties'*****" << '\n';
  is >> mu >> kn >> en2 >> kt;
}

void set_properties::exec() {
  box->dataTable.set("mu", 0, 0, mu);
  box->dataTable.set("kn", 0, 0, kn);
  box->dataTable.set("en2", 0, 0, en2);
  box->dataTable.set("kt", 0, 0, kt);
}
