#ifndef CEREAL_TYPES_QUAT_HPP_
#define CEREAL_TYPES_QUAT_HPP_

#include <cereal/cereal.hpp>
#include "vec3.hpp" // needs the cereal version of vec3
#include "../quat.hpp"

namespace cereal {

template<class Archive>
void CEREAL_SERIALIZE_FUNCTION_NAME(Archive & archive,
quat & m)
{
	archive( 
		cereal::make_nvp("v", m.v),
		cereal::make_nvp("s", m.s)
	);
}

}

#endif
