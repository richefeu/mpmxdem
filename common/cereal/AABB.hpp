#ifndef CEREAL_TYPES_AABB_HPP_
#define CEREAL_TYPES_AABB_HPP_

#include <cereal/cereal.hpp>
#include "vec3.hpp" // needs the cereal version of vec3
#include "../AABB.hpp"

namespace cereal {

template<class Archive>
void CEREAL_SERIALIZE_FUNCTION_NAME(Archive & archive,
AABB & m)
{
	archive( 
		cereal::make_nvp("min", m.min),
		cereal::make_nvp("max", m.max)
	);
}

}

#endif
