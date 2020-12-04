#ifndef CEREAL_TYPES_MAT4SYM_HPP_
#define CEREAL_TYPES_MAT4SYM_HPP_

#include <cereal/cereal.hpp>
#include "../mat4sym.hpp"

namespace cereal {

template<class Archive, typename T>
void CEREAL_SERIALIZE_FUNCTION_NAME(Archive & archive,
mat4sym<T> & m)
{
	archive( 
		cereal::make_nvp("xx", m.xx),
		cereal::make_nvp("xy", m.xy),
		cereal::make_nvp("yy", m.yy)
	);
}

}

#endif
