#ifndef CEREAL_TYPES_VEC3_HPP_
#define CEREAL_TYPES_VEC3_HPP_

#include <cereal/cereal.hpp>
#include "../vec3.hpp"

namespace cereal {

template<class Archive, typename T>
void CEREAL_SERIALIZE_FUNCTION_NAME(Archive & archive,
vec3<T> & m)
{
	archive( 
		cereal::make_nvp("x", m.x),
		cereal::make_nvp("y", m.y),
		cereal::make_nvp("z", m.z)
	);
}

}

#endif
