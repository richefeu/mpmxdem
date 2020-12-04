#include "mat9.hpp"

template <> const mat9r mat9r::zero(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
template <> const mat9i mat9i::zero(0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const mat9ui mat9ui::zero(0, 0, 0, 0, 0, 0, 0, 0, 0);
template <> const mat9b mat9b::zero(false, false, false, false, false, false, false, false, false);

template <> const mat9r mat9r::unit(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
template <> const mat9i mat9i::unit(1, 0, 0, 0, 1, 0, 0, 0, 1);
template <> const mat9ui mat9ui::unit(1, 0, 0, 0, 1, 0, 0, 0, 1);
template <> const mat9b mat9b::unit(true, false, false, false, true, false, false, false, true);
