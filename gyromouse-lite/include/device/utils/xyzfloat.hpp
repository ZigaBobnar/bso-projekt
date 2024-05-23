#pragma once

#include <stdint.h>

template <typename T>
struct XYZGeneric {
    T x;
    T y;
    T z;
};

using XYZFloat = XYZGeneric<float>;
using XYZDouble = XYZGeneric<double>;
using XYZInt8 = XYZGeneric<int8_t>;
using XYZUint8 = XYZGeneric<uint8_t>;
using XYZInt16 = XYZGeneric<int16_t>;
using XYZUint16 = XYZGeneric<uint16_t>;
using XYZInt32 = XYZGeneric<int32_t>;
using XYZUint32 = XYZGeneric<uint32_t>;
using XYZBool = XYZGeneric<bool>;
