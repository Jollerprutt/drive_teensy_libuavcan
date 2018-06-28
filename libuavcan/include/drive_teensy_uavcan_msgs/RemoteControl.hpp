/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /lnk/phoenix/drive_teensy_DJI/lib/drive_teensy_uavcan_msgs/5000.RemoteControl.uavcan
 */

#ifndef DRIVE_TEENSY_UAVCAN_MSGS_REMOTECONTROL_HPP_INCLUDED
#define DRIVE_TEENSY_UAVCAN_MSGS_REMOTECONTROL_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
uint8 MODE_RC_DISCONNECTED  = 0
uint8 MODE_MANUAL           = 1
uint8 MODE_SEMI_AUTONOMOUS  = 2
uint8 MODE_AUTONOMOUS       = 3
uint8 mode

float32 velocity_front
******************************************************************************/

/********************* DSDL signature source definition ***********************
drive_teensy_uavcan_msgs.RemoteControl
saturated uint8 mode
saturated float32 velocity_front
******************************************************************************/

#undef mode
#undef velocity_front
#undef MODE_RC_DISCONNECTED
#undef MODE_MANUAL
#undef MODE_SEMI_AUTONOMOUS
#undef MODE_AUTONOMOUS

namespace drive_teensy_uavcan_msgs
{

template <int _tmpl>
struct UAVCAN_EXPORT RemoteControl_
{
    typedef const RemoteControl_<_tmpl>& ParameterType;
    typedef RemoteControl_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > MODE_RC_DISCONNECTED;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > MODE_MANUAL;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > MODE_SEMI_AUTONOMOUS;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > MODE_AUTONOMOUS;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > mode;
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > velocity_front;
    };

    enum
    {
        MinBitLen
            = FieldTypes::mode::MinBitLen
            + FieldTypes::velocity_front::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::mode::MaxBitLen
            + FieldTypes::velocity_front::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::MODE_RC_DISCONNECTED >::Type MODE_RC_DISCONNECTED; // 0
    static const typename ::uavcan::StorageType< typename ConstantTypes::MODE_MANUAL >::Type MODE_MANUAL; // 1
    static const typename ::uavcan::StorageType< typename ConstantTypes::MODE_SEMI_AUTONOMOUS >::Type MODE_SEMI_AUTONOMOUS; // 2
    static const typename ::uavcan::StorageType< typename ConstantTypes::MODE_AUTONOMOUS >::Type MODE_AUTONOMOUS; // 3

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::mode >::Type mode;
    typename ::uavcan::StorageType< typename FieldTypes::velocity_front >::Type velocity_front;

    RemoteControl_()
        : mode()
        , velocity_front()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<40 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    enum { DefaultDataTypeID = 5000 };

    static const char* getDataTypeFullName()
    {
        return "drive_teensy_uavcan_msgs.RemoteControl";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool RemoteControl_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        mode == rhs.mode &&
        velocity_front == rhs.velocity_front;
}

template <int _tmpl>
bool RemoteControl_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(mode, rhs.mode) &&
        ::uavcan::areClose(velocity_front, rhs.velocity_front);
}

template <int _tmpl>
int RemoteControl_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::mode::encode(self.mode, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::velocity_front::encode(self.velocity_front, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int RemoteControl_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::mode::decode(self.mode, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::velocity_front::decode(self.velocity_front, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature RemoteControl_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x7F87C36E1B768443ULL);

    FieldTypes::mode::extendDataTypeSignature(signature);
    FieldTypes::velocity_front::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename RemoteControl_<_tmpl>::ConstantTypes::MODE_RC_DISCONNECTED >::Type
    RemoteControl_<_tmpl>::MODE_RC_DISCONNECTED = 0U; // 0

template <int _tmpl>
const typename ::uavcan::StorageType< typename RemoteControl_<_tmpl>::ConstantTypes::MODE_MANUAL >::Type
    RemoteControl_<_tmpl>::MODE_MANUAL = 1U; // 1

template <int _tmpl>
const typename ::uavcan::StorageType< typename RemoteControl_<_tmpl>::ConstantTypes::MODE_SEMI_AUTONOMOUS >::Type
    RemoteControl_<_tmpl>::MODE_SEMI_AUTONOMOUS = 2U; // 2

template <int _tmpl>
const typename ::uavcan::StorageType< typename RemoteControl_<_tmpl>::ConstantTypes::MODE_AUTONOMOUS >::Type
    RemoteControl_<_tmpl>::MODE_AUTONOMOUS = 3U; // 3

/*
 * Final typedef
 */
typedef RemoteControl_<0> RemoteControl;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::drive_teensy_uavcan_msgs::RemoteControl > _uavcan_gdtr_registrator_RemoteControl;

}

} // Namespace drive_teensy_uavcan_msgs

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::drive_teensy_uavcan_msgs::RemoteControl >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::drive_teensy_uavcan_msgs::RemoteControl::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::drive_teensy_uavcan_msgs::RemoteControl >::stream(Stream& s, ::drive_teensy_uavcan_msgs::RemoteControl::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "mode: ";
    YamlStreamer< ::drive_teensy_uavcan_msgs::RemoteControl::FieldTypes::mode >::stream(s, obj.mode, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "velocity_front: ";
    YamlStreamer< ::drive_teensy_uavcan_msgs::RemoteControl::FieldTypes::velocity_front >::stream(s, obj.velocity_front, level + 1);
}

}

namespace drive_teensy_uavcan_msgs
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::drive_teensy_uavcan_msgs::RemoteControl::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::drive_teensy_uavcan_msgs::RemoteControl >::stream(s, obj, 0);
    return s;
}

} // Namespace drive_teensy_uavcan_msgs

#endif // DRIVE_TEENSY_UAVCAN_MSGS_REMOTECONTROL_HPP_INCLUDED