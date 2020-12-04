#ifndef CEREAL_TYPES_OBB_HPP_
#define CEREAL_TYPES_OBB_HPP_

#include <cereal/cereal.hpp>
#include <cereal/types/common.hpp> // for serializing c-style array
#include "vec3.hpp" // needs the cereal version of vec3
#include "../OBB.hpp"

namespace cereal {

template<class Archive>
void CEREAL_SERIALIZE_FUNCTION_NAME(Archive & archive,
OBB & m)
{
	archive( 
		cereal::make_nvp("center", m.center),
		cereal::make_nvp("e", m.e),
		cereal::make_nvp("extent", m.extent)
	);
}

}

#endif
