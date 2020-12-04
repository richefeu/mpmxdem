#ifndef CEREAL_TYPES_VEC2_HPP_
#define CEREAL_TYPES_VEC2_HPP_

#include <cereal/cereal.hpp>
#include "../vec2.hpp"

namespace cereal {

template<class Archive, typename T>
void CEREAL_SERIALIZE_FUNCTION_NAME(Archive & archive,
vec2<T> & m)
{
	archive(
		cereal::make_nvp("x", m.x),
		cereal::make_nvp("y", m.y)
	);
}

}

#endif
