# prologue.jl

const uint64_t = UInt64
const int64_t = Int64
const uint32_t = UInt32
const int32_t = Int32
const uint16_t = UInt16
const int16_t = Int16
const uint8_t = UInt8
const int8_t = Int8
const double = Float64
const float = Float32
const size_t = UInt
const uintptr_t = UInt
const intptr_t = Int
const ptrdiff_t = Int  
const char = UInt8
const bool = Bool
const void = Nothing

const FILE = Ptr{Cvoid}

struct pollfd
    fd::Cint
    events::Cshort
    revents::Cshort
end

const LWS_POLLHUP = 0x0010
const LWS_POLLIN = 0x0001
const LWS_POLLOUT = 0x0004
const LWS_POLLERR = 0x0008

const LWS_POLLHUP_ERR = LWS_POLLHUP | LWS_POLLERR


if Sys.ARCH == :x86_64
    const _LWS_PAD_SIZE = 16
else
    const _LWS_PAD_SIZE = Sys.WORD_SIZE ÷ 8 
end

function _LWS_PAD(n::Int)
    return (n % _LWS_PAD_SIZE) != 0 ? (n + (_LWS_PAD_SIZE - (n % _LWS_PAD_SIZE))) : n
end

const LWS_PRE = _LWS_PAD(4 + 10 + 2)