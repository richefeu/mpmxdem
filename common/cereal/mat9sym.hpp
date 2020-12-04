#ifndef CEREAL_TYPES_MAT9SYM_HPP_
#define CEREAL_TYPES_MAT9SYM_HPP_

#include <cereal/cereal.hpp>
#include "../mat9sym.hpp"

namespace cereal {

template<class Archive, typename T>
void CEREAL_SERIALIZE_FUNCTION_NAME(Archive & archive,
mat9sym<T> & m)
{
	archive( 
		cereal::make_nvp("xx", m.xx),
		cereal::make_nvp("xy", m.xy),
		cereal::make_nvp("xz", m.xz),
		cereal::make_nvp("yy", m.yy),
		cereal::make_nvp("yz", m.yz),
		cereal::make_nvp("zz", m.zz)
	);
}

}

#endif
