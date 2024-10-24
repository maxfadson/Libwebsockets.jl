to_c_type(t::Type) = t
to_c_type_pairs(va_list) = map(enumerate(to_c_type.(va_list))) do (ind, type)
    :(va_list[$ind]::$type)
end

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

const __darwin_socklen_t = UInt32

const __darwin_time_t = Clong

const __darwin_gid_t = UInt32

const __darwin_uid_t = UInt32

const uid_t = __darwin_uid_t

const gid_t = __darwin_gid_t

const time_t = __darwin_time_t

const socklen_t = __darwin_socklen_t

const in_addr_t = UInt32

const in_port_t = UInt16

const sa_family_t = UInt8

struct sockaddr
    sa_len::UInt8
    sa_family::sa_family_t
    sa_data::NTuple{14, Cchar}
end

struct sockaddr_storage
    ss_len::UInt8
    ss_family::sa_family_t
    __ss_pad1::NTuple{6, Cchar}
    __ss_align::Int64
    __ss_pad2::NTuple{112, Cchar}
end

struct in_addr
    s_addr::in_addr_t
end

struct sockaddr_in
    sin_len::UInt8
    sin_family::sa_family_t
    sin_port::in_port_t
    sin_addr::in_addr
    sin_zero::NTuple{8, Cchar}
end

struct addrinfo
    ai_flags::Cint
    ai_family::Cint
    ai_socktype::Cint
    ai_protocol::Cint
    ai_addrlen::socklen_t
    ai_canonname::Ptr{Cchar}
    ai_addr::Ptr{sockaddr}
    ai_next::Ptr{addrinfo}
end

const lws_usec_t = Int64

# automatic type deduction for variadic arguments may not be what you want, please use with caution
@generated function _lws_log_cx(cx, prep, obj, filter, _fun, format, va_list...)
        :(@ccall(libwebsockets._lws_log_cx(cx::Ptr{lws_log_cx_t}, prep::lws_log_prepend_cx_t, obj::Ptr{Cvoid}, filter::Cint, _fun::Ptr{Cchar}, format::Ptr{Cchar}; $(to_c_type_pairs(va_list)...))::Cvoid))
    end

mutable struct lws_context end

struct __JL_Ctag_66
    data::NTuple{8, UInt8}
end

function Base.getproperty(x::Ptr{__JL_Ctag_66}, f::Symbol)
    f === :emit && return Ptr{lws_log_emit_t}(x + 0)
    f === :emit_cx && return Ptr{lws_log_emit_cx_t}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_66, f::Symbol)
    r = Ref{__JL_Ctag_66}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_66}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_66}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

# typedef void ( * lws_log_use_cx_t ) ( struct lws_log_cx * cx , int _new )
const lws_log_use_cx_t = Ptr{Cvoid}

# typedef void ( * lws_log_prepend_cx_t ) ( struct lws_log_cx * cx , void * obj , char * * p , char * e )
const lws_log_prepend_cx_t = Ptr{Cvoid}

struct lws_log_cx
    data::NTuple{56, UInt8}
end

function Base.getproperty(x::Ptr{lws_log_cx}, f::Symbol)
    f === :u && return Ptr{__JL_Ctag_66}(x + 0)
    f === :refcount_cb && return Ptr{lws_log_use_cx_t}(x + 8)
    f === :prepend && return Ptr{lws_log_prepend_cx_t}(x + 16)
    f === :parent && return Ptr{Ptr{lws_log_cx}}(x + 24)
    f === :opaque && return Ptr{Ptr{Cvoid}}(x + 32)
    f === :stg && return Ptr{Ptr{Cvoid}}(x + 40)
    f === :lll_flags && return Ptr{UInt32}(x + 48)
    f === :refcount && return Ptr{Int32}(x + 52)
    return getfield(x, f)
end

function Base.getproperty(x::lws_log_cx, f::Symbol)
    r = Ref{lws_log_cx}(x)
    ptr = Base.unsafe_convert(Ptr{lws_log_cx}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{lws_log_cx}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

function lwsl_context_get_cx(cx)
    @ccall libwebsockets.lwsl_context_get_cx(cx::Ptr{lws_context})::Ptr{lws_log_cx}
end

function lws_log_prepend_context(cx, obj, p, e)
    @ccall libwebsockets.lws_log_prepend_context(cx::Ptr{lws_log_cx}, obj::Ptr{Cvoid}, p::Ptr{Ptr{Cchar}}, e::Ptr{Cchar})::Cvoid
end

mutable struct lws_vhost end

function lwsl_vhost_get_cx(vh)
    @ccall libwebsockets.lwsl_vhost_get_cx(vh::Ptr{lws_vhost})::Ptr{lws_log_cx}
end

function lws_log_prepend_vhost(cx, obj, p, e)
    @ccall libwebsockets.lws_log_prepend_vhost(cx::Ptr{lws_log_cx}, obj::Ptr{Cvoid}, p::Ptr{Ptr{Cchar}}, e::Ptr{Cchar})::Cvoid
end

mutable struct lws end

function lwsl_wsi_get_cx(wsi)
    @ccall libwebsockets.lwsl_wsi_get_cx(wsi::Ptr{lws})::Ptr{lws_log_cx}
end

function lws_log_prepend_wsi(cx, obj, p, e)
    @ccall libwebsockets.lws_log_prepend_wsi(cx::Ptr{lws_log_cx}, obj::Ptr{Cvoid}, p::Ptr{Ptr{Cchar}}, e::Ptr{Cchar})::Cvoid
end

const lws_log_cx_t = lws_log_cx

function lwsl_hexdump_level_cx(cx, prep, obj, hexdump_level, vbuf, len)
    @ccall libwebsockets.lwsl_hexdump_level_cx(cx::Ptr{lws_log_cx_t}, prep::lws_log_prepend_cx_t, obj::Ptr{Cvoid}, hexdump_level::Cint, vbuf::Ptr{Cvoid}, len::Csize_t)::Cvoid
end

# automatic type deduction for variadic arguments may not be what you want, please use with caution
@generated function _lws_log(filter, format, va_list...)
        :(@ccall(libwebsockets._lws_log(filter::Cint, format::Ptr{Cchar}; $(to_c_type_pairs(va_list)...))::Cvoid))
    end

function lwsl_hexdump_level(level, vbuf, len)
    @ccall libwebsockets.lwsl_hexdump_level(level::Cint, vbuf::Ptr{Cvoid}, len::Csize_t)::Cvoid
end

struct lws_dll2
    prev::Ptr{lws_dll2}
    next::Ptr{lws_dll2}
    owner::Ptr{Cvoid} # owner::Ptr{lws_dll2_owner}
end

function Base.getproperty(x::lws_dll2, f::Symbol)
    f === :owner && return Ptr{lws_dll2_owner}(getfield(x, f))
    return getfield(x, f)
end

struct lws_dll2_owner
    tail::Ptr{lws_dll2}
    head::Ptr{lws_dll2}
    count::UInt32
end

const lws_dll2_owner_t = lws_dll2_owner

function _lws_dll2_search_sz_pl(own, name, namelen, dll2_ofs, ptr_ofs)
    @ccall libwebsockets._lws_dll2_search_sz_pl(own::Ptr{lws_dll2_owner_t}, name::Ptr{Cchar}, namelen::Csize_t, dll2_ofs::Csize_t, ptr_ofs::Csize_t)::Ptr{Cvoid}
end

mutable struct lws_map end

const lws_map_t = lws_map

const lws_map_key_t = Ptr{Cvoid}

const lws_map_value_t = Ptr{Cvoid}

mutable struct lws_map_item end

function lws_map_item_create(map, key, keylen, value, valuelen)
    @ccall libwebsockets.lws_map_item_create(map::Ptr{lws_map_t}, key::lws_map_key_t, keylen::Csize_t, value::lws_map_value_t, valuelen::Csize_t)::Ptr{lws_map_item}
end

function lws_map_item_lookup(map, key, keylen)
    @ccall libwebsockets.lws_map_item_lookup(map::Ptr{lws_map_t}, key::lws_map_key_t, keylen::Csize_t)::Ptr{lws_map_item}
end

function lws_now_usecs()
    @ccall libwebsockets.lws_now_usecs()::lws_usec_t
end

const pending_timeout = UInt32
const NO_PENDING_TIMEOUT = 0 % UInt32
const PENDING_TIMEOUT_AWAITING_PROXY_RESPONSE = 1 % UInt32
const PENDING_TIMEOUT_AWAITING_CONNECT_RESPONSE = 2 % UInt32
const PENDING_TIMEOUT_ESTABLISH_WITH_SERVER = 3 % UInt32
const PENDING_TIMEOUT_AWAITING_SERVER_RESPONSE = 4 % UInt32
const PENDING_TIMEOUT_AWAITING_PING = 5 % UInt32
const PENDING_TIMEOUT_CLOSE_ACK = 6 % UInt32
const PENDING_TIMEOUT_UNUSED1 = 7 % UInt32
const PENDING_TIMEOUT_SENT_CLIENT_HANDSHAKE = 8 % UInt32
const PENDING_TIMEOUT_SSL_ACCEPT = 9 % UInt32
const PENDING_TIMEOUT_HTTP_CONTENT = 10 % UInt32
const PENDING_TIMEOUT_AWAITING_CLIENT_HS_SEND = 11 % UInt32
const PENDING_FLUSH_STORED_SEND_BEFORE_CLOSE = 12 % UInt32
const PENDING_TIMEOUT_SHUTDOWN_FLUSH = 13 % UInt32
const PENDING_TIMEOUT_CGI = 14 % UInt32
const PENDING_TIMEOUT_HTTP_KEEPALIVE_IDLE = 15 % UInt32
const PENDING_TIMEOUT_WS_PONG_CHECK_SEND_PING = 16 % UInt32
const PENDING_TIMEOUT_WS_PONG_CHECK_GET_PONG = 17 % UInt32
const PENDING_TIMEOUT_CLIENT_ISSUE_PAYLOAD = 18 % UInt32
const PENDING_TIMEOUT_AWAITING_SOCKS_GREETING_REPLY = 19 % UInt32
const PENDING_TIMEOUT_AWAITING_SOCKS_CONNECT_REPLY = 20 % UInt32
const PENDING_TIMEOUT_AWAITING_SOCKS_AUTH_REPLY = 21 % UInt32
const PENDING_TIMEOUT_KILLED_BY_SSL_INFO = 22 % UInt32
const PENDING_TIMEOUT_KILLED_BY_PARENT = 23 % UInt32
const PENDING_TIMEOUT_CLOSE_SEND = 24 % UInt32
const PENDING_TIMEOUT_HOLDING_AH = 25 % UInt32
const PENDING_TIMEOUT_UDP_IDLE = 26 % UInt32
const PENDING_TIMEOUT_CLIENT_CONN_IDLE = 27 % UInt32
const PENDING_TIMEOUT_LAGGING = 28 % UInt32
const PENDING_TIMEOUT_THREADPOOL = 29 % UInt32
const PENDING_TIMEOUT_THREADPOOL_TASK = 30 % UInt32
const PENDING_TIMEOUT_KILLED_BY_PROXY_CLIENT_CLOSE = 31 % UInt32
const PENDING_TIMEOUT_USER_OK = 32 % UInt32
const PENDING_TIMEOUT_USER_REASON_BASE = 1000 % UInt32

function lws_set_timeout(wsi, reason, secs)
    @ccall libwebsockets.lws_set_timeout(wsi::Ptr{lws}, reason::pending_timeout, secs::Cint)::Cvoid
end

const lws_filepos_t = Culonglong

function lws_strncpy(dest, src, size)
    @ccall libwebsockets.lws_strncpy(dest::Ptr{Cchar}, src::Ptr{Cchar}, size::Csize_t)::Ptr{Cchar}
end

function lws_service_tsi(context, timeout_ms, tsi)
    @ccall libwebsockets.lws_service_tsi(context::Ptr{lws_context}, timeout_ms::Cint, tsi::Cint)::Cint
end

const lws_write_protocol = UInt32
const LWS_WRITE_TEXT = 0 % UInt32
const LWS_WRITE_BINARY = 1 % UInt32
const LWS_WRITE_CONTINUATION = 2 % UInt32
const LWS_WRITE_HTTP = 3 % UInt32
const LWS_WRITE_PING = 5 % UInt32
const LWS_WRITE_PONG = 6 % UInt32
const LWS_WRITE_HTTP_FINAL = 7 % UInt32
const LWS_WRITE_HTTP_HEADERS = 8 % UInt32
const LWS_WRITE_HTTP_HEADERS_CONTINUATION = 9 % UInt32
const LWS_WRITE_BUFLIST = 32 % UInt32
const LWS_WRITE_NO_FIN = 64 % UInt32
const LWS_WRITE_H2_STREAM_END = 128 % UInt32
const LWS_WRITE_CLIENT_IGNORE_XOR_MASK = 128 % UInt32

function lws_write(wsi, buf, len, protocol)
    @ccall libwebsockets.lws_write(wsi::Ptr{lws}, buf::Ptr{Cuchar}, len::Csize_t, protocol::lws_write_protocol)::Cint
end

mutable struct lws_ring end

function lws_ring_get_oldest_tail(ring)
    @ccall libwebsockets.lws_ring_get_oldest_tail(ring::Ptr{lws_ring})::UInt32
end

function lws_ring_consume(ring, tail, dest, max_count)
    @ccall libwebsockets.lws_ring_consume(ring::Ptr{lws_ring}, tail::Ptr{UInt32}, dest::Ptr{Cvoid}, max_count::Csize_t)::Csize_t
end

function lws_ring_get_count_waiting_elements(ring, tail)
    @ccall libwebsockets.lws_ring_get_count_waiting_elements(ring::Ptr{lws_ring}, tail::Ptr{UInt32})::Csize_t
end

function lws_ring_update_oldest_tail(ring, tail)
    @ccall libwebsockets.lws_ring_update_oldest_tail(ring::Ptr{lws_ring}, tail::UInt32)::Cvoid
end

mutable struct lwsac end

function lwsac_use_zero(head, ensure, chunk_size)
    @ccall libwebsockets.lwsac_use_zero(head::Ptr{Ptr{lwsac}}, ensure::Csize_t, chunk_size::Csize_t)::Ptr{Cvoid}
end

struct lws_i2c_ops
    init::Ptr{Cvoid}
    start::Ptr{Cvoid}
    stop::Ptr{Cvoid}
    write::Ptr{Cvoid}
    read::Ptr{Cvoid}
    set_ack::Ptr{Cvoid}
end

const lws_i2c_ops_t = lws_i2c_ops

function lws_bb_i2c_init(octx)
    @ccall libwebsockets.lws_bb_i2c_init(octx::Ptr{lws_i2c_ops_t})::Cint
end

function lws_bb_i2c_start(octx)
    @ccall libwebsockets.lws_bb_i2c_start(octx::Ptr{lws_i2c_ops_t})::Cint
end

function lws_bb_i2c_stop(octx)
    @ccall libwebsockets.lws_bb_i2c_stop(octx::Ptr{lws_i2c_ops_t})::Cvoid
end

function lws_bb_i2c_write(octx, data)
    @ccall libwebsockets.lws_bb_i2c_write(octx::Ptr{lws_i2c_ops_t}, data::UInt8)::Cint
end

function lws_bb_i2c_read(octx)
    @ccall libwebsockets.lws_bb_i2c_read(octx::Ptr{lws_i2c_ops_t})::Cint
end

function lws_bb_i2c_set_ack(octx, ack)
    @ccall libwebsockets.lws_bb_i2c_set_ack(octx::Ptr{lws_i2c_ops_t}, ack::Cint)::Cvoid
end

struct lws_spi_ops
    init::Ptr{Cvoid}
    queue::Ptr{Cvoid}
    bus_mode::UInt8
end

const lws_spi_ops_t = lws_spi_ops

function lws_bb_spi_init(octx)
    @ccall libwebsockets.lws_bb_spi_init(octx::Ptr{lws_spi_ops_t})::Cint
end

# typedef int ( * lws_spi_cb_t ) ( void * opaque )
const lws_spi_cb_t = Ptr{Cvoid}

struct lws_spi_desc
    src::Ptr{UInt8}
    data::Ptr{UInt8}
    dest::Ptr{UInt8}
    opaque::Ptr{Cvoid}
    completion_cb::lws_spi_cb_t
    count_cmd::UInt16
    count_write::UInt16
    count_read::UInt16
    txn_type::UInt8
    channel::UInt8
end

const lws_spi_desc_t = lws_spi_desc

function lws_bb_spi_queue(octx, desc)
    @ccall libwebsockets.lws_bb_spi_queue(octx::Ptr{lws_spi_ops_t}, desc::Ptr{lws_spi_desc_t})::Cint
end

struct lws_led_ops
    intensity::Ptr{Cvoid}
    create::Ptr{Cvoid}
    destroy::Ptr{Cvoid}
end

const lws_led_ops_t = lws_led_ops

mutable struct lws_led_state end

function lws_led_gpio_create(led_ops)
    @ccall libwebsockets.lws_led_gpio_create(led_ops::Ptr{lws_led_ops_t})::Ptr{lws_led_state}
end

function lws_led_gpio_destroy(lcs)
    @ccall libwebsockets.lws_led_gpio_destroy(lcs::Ptr{lws_led_state})::Cvoid
end

const lws_led_intensity_t = UInt16

function lws_led_gpio_intensity(lo, name, inten)
    @ccall libwebsockets.lws_led_gpio_intensity(lo::Ptr{lws_led_ops}, name::Ptr{Cchar}, inten::lws_led_intensity_t)::Cvoid
end

const _lws_plat_gpio_t = Cint

struct lws_pwm_map
    gpio::_lws_plat_gpio_t
    index::UInt8
    active_level::UInt8
end

const lws_pwm_map_t = lws_pwm_map

struct lws_pwm_ops
    init::Ptr{Cvoid}
    intensity::Ptr{Cvoid}
    pwm_map::Ptr{lws_pwm_map_t}
    count_pwm_map::UInt8
end

function lws_pwm_plat_init(lo)
    @ccall libwebsockets.lws_pwm_plat_init(lo::Ptr{lws_pwm_ops})::Cint
end

function lws_pwm_plat_intensity(lo, gpio, inten)
    @ccall libwebsockets.lws_pwm_plat_intensity(lo::Ptr{lws_pwm_ops}, gpio::_lws_plat_gpio_t, inten::lws_led_intensity_t)::Cvoid
end

const lws_pwm_ops_t = lws_pwm_ops

# typedef lws_led_intensity_t ( * lws_led_lookup_t ) ( lws_led_seq_phase_t ph )
const lws_led_lookup_t = Ptr{Cvoid}

const lws_led_seq_phase_t = UInt16

struct lws_led_sequence_def_t
    func::lws_led_lookup_t
    ledphase_offset::lws_led_seq_phase_t
    ledphase_total::Cint
    ms::UInt16
    flags::UInt8
end

const lws_display_scalar = UInt16

struct lws_display
    init::Ptr{Cvoid}
    bl_pwm_ops::Ptr{lws_pwm_ops_t}
    contrast::Ptr{Cvoid}
    blit::Ptr{Cvoid}
    power::Ptr{Cvoid}
    bl_active::Ptr{lws_led_sequence_def_t}
    bl_dim::Ptr{lws_led_sequence_def_t}
    bl_transition::Ptr{lws_led_sequence_def_t}
    variant::Ptr{Cvoid}
    bl_index::Cint
    w::lws_display_scalar
    h::lws_display_scalar
    latency_wake_ms::UInt8
end

function lws_display_ssd1306_i2c_init(disp)
    @ccall libwebsockets.lws_display_ssd1306_i2c_init(disp::Ptr{lws_display})::Cint
end

function lws_display_ssd1306_i2c_contrast(disp, b)
    @ccall libwebsockets.lws_display_ssd1306_i2c_contrast(disp::Ptr{lws_display}, b::UInt8)::Cint
end

function lws_display_ssd1306_i2c_blit(disp, src, x, y, w, h)
    @ccall libwebsockets.lws_display_ssd1306_i2c_blit(disp::Ptr{lws_display}, src::Ptr{UInt8}, x::lws_display_scalar, y::lws_display_scalar, w::lws_display_scalar, h::lws_display_scalar)::Cint
end

function lws_display_ssd1306_i2c_power(disp, state)
    @ccall libwebsockets.lws_display_ssd1306_i2c_power(disp::Ptr{lws_display}, state::Cint)::Cint
end

function lws_display_ili9341_spi_init(disp)
    @ccall libwebsockets.lws_display_ili9341_spi_init(disp::Ptr{lws_display})::Cint
end

function lws_display_ili9341_spi_blit(disp, src, x, y, w, h)
    @ccall libwebsockets.lws_display_ili9341_spi_blit(disp::Ptr{lws_display}, src::Ptr{UInt8}, x::lws_display_scalar, y::lws_display_scalar, w::lws_display_scalar, h::lws_display_scalar)::Cint
end

function lws_display_ili9341_spi_power(disp, state)
    @ccall libwebsockets.lws_display_ili9341_spi_power(disp::Ptr{lws_display}, state::Cint)::Cint
end

struct lws_settings_ops
    get::Ptr{Cvoid}
    set::Ptr{Cvoid}
end

struct lws_settings_instance_t
    handle_plat::Ptr{Cvoid}
    so::Ptr{lws_settings_ops}
    refcount::UInt8
    opaque_plat::Ptr{Cvoid}
end

function lws_settings_plat_get(si, name, dest, max_actual)
    @ccall libwebsockets.lws_settings_plat_get(si::Ptr{lws_settings_instance_t}, name::Ptr{Cchar}, dest::Ptr{UInt8}, max_actual::Ptr{Csize_t})::Cint
end

function lws_settings_plat_set(si, name, src, len)
    @ccall libwebsockets.lws_settings_plat_set(si::Ptr{lws_settings_instance_t}, name::Ptr{Cchar}, src::Ptr{UInt8}, len::Csize_t)::Cint
end

struct lws_netdev_ops
    create::Ptr{Cvoid}
    configure::Ptr{Cvoid}
    up::Ptr{Cvoid}
    down::Ptr{Cvoid}
    event::Ptr{Cvoid}
    destroy::Ptr{Cvoid}
    connect::Ptr{Cvoid}
    scan::Ptr{Cvoid}
end

const lws_netdev_ops_t = lws_netdev_ops

const lws_dll2_t = lws_dll2

struct lws_netdev_instance
    name::Ptr{Cchar}
    ops::Ptr{lws_netdev_ops_t}
    platinfo::Ptr{Cvoid}
    list::lws_dll2_t
    mac::NTuple{6, UInt8}
    type::UInt8
end

function lws_netdev_wifi_create_plat(ctx, ops, name, platinfo)
    @ccall libwebsockets.lws_netdev_wifi_create_plat(ctx::Ptr{lws_context}, ops::Ptr{lws_netdev_ops_t}, name::Ptr{Cchar}, platinfo::Ptr{Cvoid})::Ptr{lws_netdev_instance}
end

struct lws_netdev_config
    plat_config::Ptr{Cvoid}
end

const lws_netdev_config_t = lws_netdev_config

function lws_netdev_wifi_configure_plat(nd, config)
    @ccall libwebsockets.lws_netdev_wifi_configure_plat(nd::Ptr{lws_netdev_instance}, config::Ptr{lws_netdev_config_t})::Cint
end

function lws_netdev_wifi_event_plat(nd, timestamp, buf, len)
    @ccall libwebsockets.lws_netdev_wifi_event_plat(nd::Ptr{lws_netdev_instance}, timestamp::lws_usec_t, buf::Ptr{Cvoid}, len::Csize_t)::Cint
end

function lws_netdev_wifi_up_plat(nd)
    @ccall libwebsockets.lws_netdev_wifi_up_plat(nd::Ptr{lws_netdev_instance})::Cint
end

function lws_netdev_wifi_down_plat(nd)
    @ccall libwebsockets.lws_netdev_wifi_down_plat(nd::Ptr{lws_netdev_instance})::Cint
end

const lws_netdev_instance_t = lws_netdev_instance

function lws_netdev_wifi_connect_plat(wnd, ssid, passphrase, bssid)
    @ccall libwebsockets.lws_netdev_wifi_connect_plat(wnd::Ptr{lws_netdev_instance_t}, ssid::Ptr{Cchar}, passphrase::Ptr{Cchar}, bssid::Ptr{UInt8})::Cint
end

function lws_netdev_wifi_scan_plat(nd)
    @ccall libwebsockets.lws_netdev_wifi_scan_plat(nd::Ptr{lws_netdev_instance_t})::Cvoid
end

function lws_netdev_wifi_destroy_plat(pnd)
    @ccall libwebsockets.lws_netdev_wifi_destroy_plat(pnd::Ptr{Ptr{lws_netdev_instance}})::Cvoid
end

const lws_ctx_t = Ptr{lws_context}

mutable struct lws_dsh end

# typedef void ( * lws_log_emit_t ) ( int level , const char * line )
const lws_log_emit_t = Ptr{Cvoid}

# typedef void ( * lws_log_emit_cx_t ) ( struct lws_log_cx * cx , int level , const char * line , size_t len )
const lws_log_emit_cx_t = Ptr{Cvoid}

function lwsl_timestamp(level, p, len)
    @ccall libwebsockets.lwsl_timestamp(level::Cint, p::Ptr{Cchar}, len::Csize_t)::Cint
end

function lws_log_emit_cx_file(cx, level, line, len)
    @ccall libwebsockets.lws_log_emit_cx_file(cx::Ptr{lws_log_cx}, level::Cint, line::Ptr{Cchar}, len::Csize_t)::Cvoid
end

function lws_log_use_cx_file(cx, _new)
    @ccall libwebsockets.lws_log_use_cx_file(cx::Ptr{lws_log_cx}, _new::Cint)::Cvoid
end

function lwsl_hexdump(buf, len)
    @ccall libwebsockets.lwsl_hexdump(buf::Ptr{Cvoid}, len::Csize_t)::Cvoid
end

function lws_is_be()
    @ccall libwebsockets.lws_is_be()::Cint
end

function lws_set_log_level(level, log_emit_function)
    @ccall libwebsockets.lws_set_log_level(level::Cint, log_emit_function::lws_log_emit_t)::Cvoid
end

function lwsl_emit_syslog(level, line)
    @ccall libwebsockets.lwsl_emit_syslog(level::Cint, line::Ptr{Cchar})::Cvoid
end

function lwsl_emit_stderr(level, line)
    @ccall libwebsockets.lwsl_emit_stderr(level::Cint, line::Ptr{Cchar})::Cvoid
end

function lwsl_emit_stderr_notimestamp(level, line)
    @ccall libwebsockets.lwsl_emit_stderr_notimestamp(level::Cint, line::Ptr{Cchar})::Cvoid
end

function lwsl_visible(level)
    @ccall libwebsockets.lwsl_visible(level::Cint)::Cint
end

function lws_wsi_tag(wsi)
    @ccall libwebsockets.lws_wsi_tag(wsi::Ptr{lws})::Ptr{Cchar}
end

function lwsl_refcount_cx(cx, _new)
    @ccall libwebsockets.lwsl_refcount_cx(cx::Ptr{lws_log_cx_t}, _new::Cint)::Cvoid
end

const lws_sockfd_type = Cint

const lws_filefd_type = Cint

const lws_fileofs_t = Clonglong

const lws_fop_flags_t = UInt32

struct lws_pollargs
    fd::lws_sockfd_type
    events::Cint
    prev_events::Cint
end

function lws_dll2_is_detached(d)
    @ccall libwebsockets.lws_dll2_is_detached(d::Ptr{lws_dll2})::Cint
end

function lws_dll2_owner(d)
    @ccall libwebsockets.lws_dll2_owner(d::Ptr{lws_dll2})::Ptr{lws_dll2_owner}
end

function lws_dll2_get_head(owner)
    @ccall libwebsockets.lws_dll2_get_head(owner::Ptr{lws_dll2_owner})::Ptr{lws_dll2}
end

function lws_dll2_get_tail(owner)
    @ccall libwebsockets.lws_dll2_get_tail(owner::Ptr{lws_dll2_owner})::Ptr{lws_dll2}
end

function lws_dll2_add_head(d, owner)
    @ccall libwebsockets.lws_dll2_add_head(d::Ptr{lws_dll2}, owner::Ptr{lws_dll2_owner})::Cvoid
end

function lws_dll2_add_tail(d, owner)
    @ccall libwebsockets.lws_dll2_add_tail(d::Ptr{lws_dll2}, owner::Ptr{lws_dll2_owner})::Cvoid
end

function lws_dll2_remove(d)
    @ccall libwebsockets.lws_dll2_remove(d::Ptr{lws_dll2})::Cvoid
end

# typedef int ( * lws_dll2_foreach_cb_t ) ( struct lws_dll2 * d , void * user )
const lws_dll2_foreach_cb_t = Ptr{Cvoid}

function lws_dll2_foreach_safe(owner, user, cb)
    @ccall libwebsockets.lws_dll2_foreach_safe(owner::Ptr{lws_dll2_owner}, user::Ptr{Cvoid}, cb::lws_dll2_foreach_cb_t)::Cint
end

function lws_dll2_clear(d)
    @ccall libwebsockets.lws_dll2_clear(d::Ptr{lws_dll2})::Cvoid
end

function lws_dll2_owner_clear(d)
    @ccall libwebsockets.lws_dll2_owner_clear(d::Ptr{lws_dll2_owner})::Cvoid
end

function lws_dll2_add_before(d, after)
    @ccall libwebsockets.lws_dll2_add_before(d::Ptr{lws_dll2}, after::Ptr{lws_dll2})::Cvoid
end

function lws_dll2_add_sorted(d, own, compare)
    @ccall libwebsockets.lws_dll2_add_sorted(d::Ptr{lws_dll2_t}, own::Ptr{lws_dll2_owner_t}, compare::Ptr{Cvoid})::Cvoid
end

function lws_dll2_add_sorted_priv(d, own, priv, compare3)
    @ccall libwebsockets.lws_dll2_add_sorted_priv(d::Ptr{lws_dll2_t}, own::Ptr{lws_dll2_owner_t}, priv::Ptr{Cvoid}, compare3::Ptr{Cvoid})::Cvoid
end

function lws_dll2_describe(owner, desc)
    @ccall libwebsockets.lws_dll2_describe(owner::Ptr{lws_dll2_owner}, desc::Ptr{Cchar})::Cvoid
end

const lws_map_hash_t = UInt32

# typedef lws_map_hash_t ( * lws_map_hash_from_key_t ) ( const lws_map_key_t key , size_t kl )
const lws_map_hash_from_key_t = Ptr{Cvoid}

# typedef int ( * lws_map_compare_key_t ) ( const lws_map_key_t key1 , size_t kl1 , const lws_map_value_t key2 , size_t kl2 )
const lws_map_compare_key_t = Ptr{Cvoid}

# typedef void * ( * lws_map_alloc_t ) ( struct lws_map * mo , size_t x )
const lws_map_alloc_t = Ptr{Cvoid}

# typedef void ( * lws_map_free_t ) ( void * )
const lws_map_free_t = Ptr{Cvoid}

struct lws_map_info
    _hash::lws_map_hash_from_key_t
    _compare::lws_map_compare_key_t
    _alloc::lws_map_alloc_t
    _free::lws_map_free_t
    opaque::Ptr{Cvoid}
    aux::Ptr{Cvoid}
    modulo::Csize_t
end

const lws_map_info_t = lws_map_info

function lws_map_item_key(_item)
    @ccall libwebsockets.lws_map_item_key(_item::Ptr{lws_map_item})::Ptr{Cvoid}
end

function lws_map_item_value(_item)
    @ccall libwebsockets.lws_map_item_value(_item::Ptr{lws_map_item})::Ptr{Cvoid}
end

function lws_map_item_key_len(_item)
    @ccall libwebsockets.lws_map_item_key_len(_item::Ptr{lws_map_item})::Csize_t
end

function lws_map_item_value_len(_item)
    @ccall libwebsockets.lws_map_item_value_len(_item::Ptr{lws_map_item})::Csize_t
end

function lws_map_create(info)
    @ccall libwebsockets.lws_map_create(info::Ptr{lws_map_info_t})::Ptr{lws_map_t}
end

function lws_map_alloc_lwsac(map, x)
    @ccall libwebsockets.lws_map_alloc_lwsac(map::Ptr{lws_map}, x::Csize_t)::Ptr{Cvoid}
end

function lws_map_free_lwsac(v)
    @ccall libwebsockets.lws_map_free_lwsac(v::Ptr{Cvoid})::Cvoid
end

function lws_map_destroy(pmap)
    @ccall libwebsockets.lws_map_destroy(pmap::Ptr{Ptr{lws_map_t}})::Cvoid
end

function lws_map_item_destroy(item)
    @ccall libwebsockets.lws_map_item_destroy(item::Ptr{lws_map_item})::Cvoid
end

struct lws_xos
    s::NTuple{4, UInt64}
end

const lws_xos_t = lws_xos

function lws_xos_init(xos, seed)
    @ccall libwebsockets.lws_xos_init(xos::Ptr{lws_xos}, seed::UInt64)::Cvoid
end

function lws_xos(xos)
    @ccall libwebsockets.lws_xos(xos::Ptr{lws_xos})::UInt64
end

function lws_xos_percent(xos, percent)
    @ccall libwebsockets.lws_xos_percent(xos::Ptr{lws_xos}, percent::Cint)::Cint
end

function lws_set_timeout_us(wsi, reason, us)
    @ccall libwebsockets.lws_set_timeout_us(wsi::Ptr{lws}, reason::pending_timeout, us::lws_usec_t)::Cvoid
end

function lws_set_timer_usecs(wsi, usecs)
    @ccall libwebsockets.lws_set_timer_usecs(wsi::Ptr{lws}, usecs::lws_usec_t)::Cvoid
end

# typedef void ( * sul_cb_t ) ( struct lws_sorted_usec_list * sul )
const sul_cb_t = Ptr{Cvoid}

struct lws_sorted_usec_list
    list::lws_dll2
    us::lws_usec_t
    cb::sul_cb_t
    latency_us::UInt32
end

const lws_sorted_usec_list_t = lws_sorted_usec_list

function lws_sul2_schedule(context, tsi, flags, sul)
    @ccall libwebsockets.lws_sul2_schedule(context::Ptr{lws_context}, tsi::Cint, flags::Cint, sul::Ptr{lws_sorted_usec_list_t})::Cvoid
end

function lws_sul_cancel(sul)
    @ccall libwebsockets.lws_sul_cancel(sul::Ptr{lws_sorted_usec_list_t})::Cvoid
end

function lws_sul_earliest_wakeable_event(ctx, pearliest)
    @ccall libwebsockets.lws_sul_earliest_wakeable_event(ctx::Ptr{lws_context}, pearliest::Ptr{lws_usec_t})::Cint
end

function lws_sul_schedule(ctx, tsi, sul, _cb, _us)
    @ccall libwebsockets.lws_sul_schedule(ctx::Ptr{lws_context}, tsi::Cint, sul::Ptr{lws_sorted_usec_list_t}, _cb::sul_cb_t, _us::lws_usec_t)::Cvoid
end

function lws_sul_schedule_wakesuspend(ctx, tsi, sul, _cb, _us)
    @ccall libwebsockets.lws_sul_schedule_wakesuspend(ctx::Ptr{lws_context}, tsi::Cint, sul::Ptr{lws_sorted_usec_list_t}, _cb::sul_cb_t, _us::lws_usec_t)::Cvoid
end

function lws_validity_confirmed(wsi)
    @ccall libwebsockets.lws_validity_confirmed(wsi::Ptr{lws})::Cvoid
end

function __lws_sul_insert(own, sul)
    @ccall libwebsockets.__lws_sul_insert(own::Ptr{lws_dll2_owner_t}, sul::Ptr{lws_sorted_usec_list_t})::Cint
end

function __lws_sul_service_ripe(own, own_len, usnow)
    @ccall libwebsockets.__lws_sul_service_ripe(own::Ptr{lws_dll2_owner_t}, own_len::Cint, usnow::lws_usec_t)::lws_usec_t
end

mutable struct lws_cache_ttl_lru end

function lws_cache_write_through(cache, specific_key, source, size, expiry, ppay)
    @ccall libwebsockets.lws_cache_write_through(cache::Ptr{lws_cache_ttl_lru}, specific_key::Ptr{Cchar}, source::Ptr{UInt8}, size::Csize_t, expiry::lws_usec_t, ppay::Ptr{Ptr{Cvoid}})::Cint
end

struct lws_cache_match
    list::lws_dll2_t
    expiry::lws_usec_t
    payload_size::Csize_t
    tag_size::Csize_t
end

const lws_cache_match_t = lws_cache_match

function lws_cache_lookup(cache, wildcard_key, pdata, psize)
    @ccall libwebsockets.lws_cache_lookup(cache::Ptr{lws_cache_ttl_lru}, wildcard_key::Ptr{Cchar}, pdata::Ptr{Ptr{Cvoid}}, psize::Ptr{Csize_t})::Cint
end

function lws_cache_item_get(cache, specific_key, pdata, psize)
    @ccall libwebsockets.lws_cache_item_get(cache::Ptr{lws_cache_ttl_lru}, specific_key::Ptr{Cchar}, pdata::Ptr{Ptr{Cvoid}}, psize::Ptr{Csize_t})::Cint
end

function lws_cache_item_remove(cache, wildcard_key)
    @ccall libwebsockets.lws_cache_item_remove(cache::Ptr{lws_cache_ttl_lru}, wildcard_key::Ptr{Cchar})::Cint
end

function lws_cache_footprint(cache)
    @ccall libwebsockets.lws_cache_footprint(cache::Ptr{lws_cache_ttl_lru})::UInt64
end

function lws_cache_debug_dump(cache)
    @ccall libwebsockets.lws_cache_debug_dump(cache::Ptr{lws_cache_ttl_lru})::Cvoid
end

struct lws_cache_results
    ptr::Ptr{UInt8}
    size::Csize_t
    payload_len::Csize_t
    tag_len::Csize_t
    tag::Ptr{UInt8}
end

const lws_cache_results_t = lws_cache_results

function lws_cache_results_walk(walk_ctx)
    @ccall libwebsockets.lws_cache_results_walk(walk_ctx::Ptr{lws_cache_results_t})::Cint
end

# typedef void ( * lws_cache_item_destroy_cb ) ( void * item , size_t size )
const lws_cache_item_destroy_cb = Ptr{Cvoid}

struct lws_cache_ops
    create::Ptr{Cvoid}
    destroy::Ptr{Cvoid}
    expunge::Ptr{Cvoid}
    write::Ptr{Cvoid}
    tag_match::Ptr{Cvoid}
    lookup::Ptr{Cvoid}
    invalidate::Ptr{Cvoid}
    get::Ptr{Cvoid}
    debug_dump::Ptr{Cvoid}
end

struct __JL_Ctag_64
    data::NTuple{8, UInt8}
end

function Base.getproperty(x::Ptr{__JL_Ctag_64}, f::Symbol)
    f === :nscookiejar && return Ptr{__JL_Ctag_65}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_64, f::Symbol)
    r = Ref{__JL_Ctag_64}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_64}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_64}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

struct lws_cache_creation_info
    data::NTuple{80, UInt8}
end

function Base.getproperty(x::Ptr{lws_cache_creation_info}, f::Symbol)
    f === :cx && return Ptr{Ptr{lws_context}}(x + 0)
    f === :name && return Ptr{Ptr{Cchar}}(x + 8)
    f === :cb && return Ptr{lws_cache_item_destroy_cb}(x + 16)
    f === :parent && return Ptr{Ptr{lws_cache_ttl_lru}}(x + 24)
    f === :ops && return Ptr{Ptr{lws_cache_ops}}(x + 32)
    f === :u && return Ptr{__JL_Ctag_64}(x + 40)
    f === :max_footprint && return Ptr{Csize_t}(x + 48)
    f === :max_items && return Ptr{Csize_t}(x + 56)
    f === :max_payload && return Ptr{Csize_t}(x + 64)
    f === :tsi && return Ptr{Cint}(x + 72)
    return getfield(x, f)
end

function Base.getproperty(x::lws_cache_creation_info, f::Symbol)
    r = Ref{lws_cache_creation_info}(x)
    ptr = Base.unsafe_convert(Ptr{lws_cache_creation_info}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{lws_cache_creation_info}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

function lws_cache_create(info)
    @ccall libwebsockets.lws_cache_create(info::Ptr{lws_cache_creation_info})::Ptr{lws_cache_ttl_lru}
end

function lws_cache_destroy(cache)
    @ccall libwebsockets.lws_cache_destroy(cache::Ptr{Ptr{lws_cache_ttl_lru}})::Cvoid
end

function lws_cache_expunge(cache)
    @ccall libwebsockets.lws_cache_expunge(cache::Ptr{lws_cache_ttl_lru})::Cint
end

const lws_smd_class_t = UInt32

mutable struct lws_smd_msg end

mutable struct lws_smd_peer end

const __JL_Ctag_8 = UInt32
const LWSSMDCL_INTERACTION = 1 % UInt32
const LWSSMDCL_SYSTEM_STATE = 2 % UInt32
const LWSSMDCL_NETWORK = 4 % UInt32
const LWSSMDCL_METRICS = 8 % UInt32
const LWSSMDCL_USER_BASE_BITNUM = 24 % UInt32

function lws_smd_msg_alloc(ctx, _class, len)
    @ccall libwebsockets.lws_smd_msg_alloc(ctx::Ptr{lws_context}, _class::lws_smd_class_t, len::Csize_t)::Ptr{Cvoid}
end

function lws_smd_msg_free(payload)
    @ccall libwebsockets.lws_smd_msg_free(payload::Ptr{Ptr{Cvoid}})::Cvoid
end

function lws_smd_msg_send(ctx, payload)
    @ccall libwebsockets.lws_smd_msg_send(ctx::Ptr{lws_context}, payload::Ptr{Cvoid})::Cint
end

# automatic type deduction for variadic arguments may not be what you want, please use with caution
@generated function lws_smd_msg_printf(ctx, _class, format, va_list...)
        :(@ccall(libwebsockets.lws_smd_msg_printf(ctx::Ptr{lws_context}, _class::lws_smd_class_t, format::Ptr{Cchar}; $(to_c_type_pairs(va_list)...))::Cint))
    end

mutable struct lws_ss_handle end

# automatic type deduction for variadic arguments may not be what you want, please use with caution
@generated function lws_smd_ss_msg_printf(tag, buf, len, _class, format, va_list...)
        :(@ccall(libwebsockets.lws_smd_ss_msg_printf(tag::Ptr{Cchar}, buf::Ptr{UInt8}, len::Ptr{Csize_t}, _class::lws_smd_class_t, format::Ptr{Cchar}; $(to_c_type_pairs(va_list)...))::Cint))
    end

function lws_smd_ss_rx_forward(ss_user, buf, len)
    @ccall libwebsockets.lws_smd_ss_rx_forward(ss_user::Ptr{Cvoid}, buf::Ptr{UInt8}, len::Csize_t)::Cint
end

function lws_smd_sspc_rx_forward(ss_user, buf, len)
    @ccall libwebsockets.lws_smd_sspc_rx_forward(ss_user::Ptr{Cvoid}, buf::Ptr{UInt8}, len::Csize_t)::Cint
end

# typedef int ( * lws_smd_notification_cb_t ) ( void * opaque , lws_smd_class_t _class , lws_usec_t timestamp , void * buf , size_t len )
const lws_smd_notification_cb_t = Ptr{Cvoid}

function lws_smd_register(ctx, opaque, flags, _class_filter, cb)
    @ccall libwebsockets.lws_smd_register(ctx::Ptr{lws_context}, opaque::Ptr{Cvoid}, flags::Cint, _class_filter::lws_smd_class_t, cb::lws_smd_notification_cb_t)::Ptr{lws_smd_peer}
end

function lws_smd_unregister(pr)
    @ccall libwebsockets.lws_smd_unregister(pr::Ptr{lws_smd_peer})::Cvoid
end

# typedef int ( * lws_state_notify_t ) ( struct lws_state_manager * mgr , struct lws_state_notify_link * link , int current , int target )
const lws_state_notify_t = Ptr{Cvoid}

struct lws_state_notify_link
    list::lws_dll2_t
    notify_cb::lws_state_notify_t
    name::Ptr{Cchar}
end

const lws_state_notify_link_t = lws_state_notify_link

struct lws_state_manager
    notify_list::lws_dll2_owner_t
    context::Ptr{lws_context}
    parent::Ptr{Cvoid}
    smd_class::lws_smd_class_t
    state_names::Ptr{Ptr{Cchar}}
    name::Ptr{Cchar}
    state::Cint
end

const lws_state_manager_t = lws_state_manager

function lws_state_reg_notifier(mgr, nl)
    @ccall libwebsockets.lws_state_reg_notifier(mgr::Ptr{lws_state_manager_t}, nl::Ptr{lws_state_notify_link_t})::Cvoid
end

function lws_state_reg_deregister(nl)
    @ccall libwebsockets.lws_state_reg_deregister(nl::Ptr{lws_state_notify_link_t})::Cvoid
end

function lws_state_reg_notifier_list(mgr, nl)
    @ccall libwebsockets.lws_state_reg_notifier_list(mgr::Ptr{lws_state_manager_t}, nl::Ptr{Ptr{lws_state_notify_link_t}})::Cvoid
end

function lws_state_transition_steps(mgr, target)
    @ccall libwebsockets.lws_state_transition_steps(mgr::Ptr{lws_state_manager_t}, target::Cint)::Cint
end

function lws_state_transition(mgr, target)
    @ccall libwebsockets.lws_state_transition(mgr::Ptr{lws_state_manager_t}, target::Cint)::Cint
end

struct lws_retry_bo
    retry_ms_table::Ptr{UInt32}
    retry_ms_table_count::UInt16
    conceal_count::UInt16
    secs_since_valid_ping::UInt16
    secs_since_valid_hangup::UInt16
    jitter_percent::UInt8
end

const lws_retry_bo_t = lws_retry_bo

function lws_retry_get_delay_ms(context, retry, ctry, conceal)
    @ccall libwebsockets.lws_retry_get_delay_ms(context::Ptr{lws_context}, retry::Ptr{lws_retry_bo_t}, ctry::Ptr{UInt16}, conceal::Ptr{Cchar})::Cuint
end

function lws_retry_sul_schedule(context, tid, sul, retry, cb, ctry)
    @ccall libwebsockets.lws_retry_sul_schedule(context::Ptr{lws_context}, tid::Cint, sul::Ptr{lws_sorted_usec_list_t}, retry::Ptr{lws_retry_bo_t}, cb::sul_cb_t, ctry::Ptr{UInt16})::Cint
end

function lws_retry_sul_schedule_retry_wsi(wsi, sul, cb, ctry)
    @ccall libwebsockets.lws_retry_sul_schedule_retry_wsi(wsi::Ptr{lws}, sul::Ptr{lws_sorted_usec_list_t}, cb::sul_cb_t, ctry::Ptr{UInt16})::Cint
end

function lws_adopt_socket(context, accept_fd)
    @ccall libwebsockets.lws_adopt_socket(context::Ptr{lws_context}, accept_fd::lws_sockfd_type)::Ptr{lws}
end

function lws_adopt_socket_vhost(vh, accept_fd)
    @ccall libwebsockets.lws_adopt_socket_vhost(vh::Ptr{lws_vhost}, accept_fd::lws_sockfd_type)::Ptr{lws}
end

const lws_adoption_type = UInt32
const LWS_ADOPT_RAW_FILE_DESC = 0 % UInt32
const LWS_ADOPT_HTTP = 1 % UInt32
const LWS_ADOPT_SOCKET = 2 % UInt32
const LWS_ADOPT_ALLOW_SSL = 4 % UInt32
const LWS_ADOPT_FLAG_UDP = 16 % UInt32
const LWS_ADOPT_FLAG_RAW_PROXY = 32 % UInt32
const LWS_ADOPT_RAW_SOCKET_UDP = 18 % UInt32

struct lws_sock_file_fd_type
    data::NTuple{4, UInt8}
end

function Base.getproperty(x::Ptr{lws_sock_file_fd_type}, f::Symbol)
    f === :sockfd && return Ptr{lws_sockfd_type}(x + 0)
    f === :filefd && return Ptr{lws_filefd_type}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::lws_sock_file_fd_type, f::Symbol)
    r = Ref{lws_sock_file_fd_type}(x)
    ptr = Base.unsafe_convert(Ptr{lws_sock_file_fd_type}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{lws_sock_file_fd_type}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

struct lws_sockaddr46
    data::NTuple{16, UInt8}
end

function Base.getproperty(x::Ptr{lws_sockaddr46}, f::Symbol)
    f === :sa4 && return Ptr{sockaddr_in}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::lws_sockaddr46, f::Symbol)
    r = Ref{lws_sockaddr46}(x)
    ptr = Base.unsafe_convert(Ptr{lws_sockaddr46}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{lws_sockaddr46}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

struct lws_udp
    data::NTuple{36, UInt8}
end

function Base.getproperty(x::Ptr{lws_udp}, f::Symbol)
    f === :sa46 && return Ptr{lws_sockaddr46}(x + 0)
    f === :sa46_pending && return Ptr{lws_sockaddr46}(x + 16)
    f === :connected && return (Ptr{UInt8}(x + 32), 0, 1)
    return getfield(x, f)
end

function Base.getproperty(x::lws_udp, f::Symbol)
    r = Ref{lws_udp}(x)
    ptr = Base.unsafe_convert(Ptr{lws_udp}, r)
    fptr = getproperty(ptr, f)
    begin
        if fptr isa Ptr
            return GC.@preserve(r, unsafe_load(fptr))
        else
            (baseptr, offset, width) = fptr
            ty = eltype(baseptr)
            baseptr32 = convert(Ptr{UInt32}, baseptr)
            u64 = GC.@preserve(r, unsafe_load(baseptr32))
            if offset + width > 32
                u64 |= GC.@preserve(r, unsafe_load(baseptr32 + 4)) << 32
            end
            u64 = u64 >> offset & (1 << width - 1)
            return u64 % ty
        end
    end
end

function Base.setproperty!(x::Ptr{lws_udp}, f::Symbol, v)
    fptr = getproperty(x, f)
    if fptr isa Ptr
        unsafe_store!(getproperty(x, f), v)
    else
        (baseptr, offset, width) = fptr
        baseptr32 = convert(Ptr{UInt32}, baseptr)
        u64 = unsafe_load(baseptr32)
        straddle = offset + width > 32
        if straddle
            u64 |= unsafe_load(baseptr32 + 4) << 32
        end
        mask = 1 << width - 1
        u64 &= ~(mask << offset)
        u64 |= (unsigned(v) & mask) << offset
        unsafe_store!(baseptr32, u64 & typemax(UInt32))
        if straddle
            unsafe_store!(baseptr32 + 4, u64 >> 32)
        end
    end
end

function lws_adopt_descriptor_vhost(vh, type, fd, vh_prot_name, parent)
    @ccall libwebsockets.lws_adopt_descriptor_vhost(vh::Ptr{lws_vhost}, type::lws_adoption_type, fd::lws_sock_file_fd_type, vh_prot_name::Ptr{Cchar}, parent::Ptr{lws})::Ptr{lws}
end

struct lws_adopt_desc
    vh::Ptr{lws_vhost}
    type::lws_adoption_type
    fd::lws_sock_file_fd_type
    vh_prot_name::Ptr{Cchar}
    parent::Ptr{lws}
    opaque::Ptr{Cvoid}
    fi_wsi_name::Ptr{Cchar}
end

const lws_adopt_desc_t = lws_adopt_desc

function lws_adopt_descriptor_vhost_via_info(info)
    @ccall libwebsockets.lws_adopt_descriptor_vhost_via_info(info::Ptr{lws_adopt_desc_t})::Ptr{lws}
end

function lws_adopt_socket_readbuf(context, accept_fd, readbuf, len)
    @ccall libwebsockets.lws_adopt_socket_readbuf(context::Ptr{lws_context}, accept_fd::lws_sockfd_type, readbuf::Ptr{Cchar}, len::Csize_t)::Ptr{lws}
end

function lws_adopt_socket_vhost_readbuf(vhost, accept_fd, readbuf, len)
    @ccall libwebsockets.lws_adopt_socket_vhost_readbuf(vhost::Ptr{lws_vhost}, accept_fd::lws_sockfd_type, readbuf::Ptr{Cchar}, len::Csize_t)::Ptr{lws}
end

function lws_create_adopt_udp(vhost, ads, port, flags, protocol_name, ifname, parent_wsi, opaque, retry_policy, fi_wsi_name)
    @ccall libwebsockets.lws_create_adopt_udp(vhost::Ptr{lws_vhost}, ads::Ptr{Cchar}, port::Cint, flags::Cint, protocol_name::Ptr{Cchar}, ifname::Ptr{Cchar}, parent_wsi::Ptr{lws}, opaque::Ptr{Cvoid}, retry_policy::Ptr{lws_retry_bo_t}, fi_wsi_name::Ptr{Cchar})::Ptr{lws}
end

const lws_route_uidx_t = UInt16

struct lws_dns_score
    precedence::UInt8
    label::UInt8
end

const lws_dns_score_t = lws_dns_score

struct lws_route
    data::NTuple{104, UInt8}
end

function Base.getproperty(x::Ptr{lws_route}, f::Symbol)
    f === :list && return Ptr{lws_dll2_t}(x + 0)
    f === :src && return Ptr{lws_sockaddr46}(x + 24)
    f === :dest && return Ptr{lws_sockaddr46}(x + 40)
    f === :gateway && return Ptr{lws_sockaddr46}(x + 56)
    f === :source && return Ptr{Ptr{lws_route}}(x + 72)
    f === :score && return Ptr{lws_dns_score_t}(x + 80)
    f === :if_idx && return Ptr{Cint}(x + 84)
    f === :priority && return Ptr{Cint}(x + 88)
    f === :ifa_flags && return Ptr{Cint}(x + 92)
    f === :uidx && return Ptr{lws_route_uidx_t}(x + 96)
    f === :proto && return Ptr{UInt8}(x + 98)
    f === :dest_len && return Ptr{UInt8}(x + 99)
    f === :src_len && return Ptr{UInt8}(x + 100)
    f === :scope && return Ptr{UInt8}(x + 101)
    f === :af && return Ptr{UInt8}(x + 102)
    f === :source_ads && return (Ptr{UInt8}(x + 100), 24, 1)
    return getfield(x, f)
end

function Base.getproperty(x::lws_route, f::Symbol)
    r = Ref{lws_route}(x)
    ptr = Base.unsafe_convert(Ptr{lws_route}, r)
    fptr = getproperty(ptr, f)
    begin
        if fptr isa Ptr
            return GC.@preserve(r, unsafe_load(fptr))
        else
            (baseptr, offset, width) = fptr
            ty = eltype(baseptr)
            baseptr32 = convert(Ptr{UInt32}, baseptr)
            u64 = GC.@preserve(r, unsafe_load(baseptr32))
            if offset + width > 32
                u64 |= GC.@preserve(r, unsafe_load(baseptr32 + 4)) << 32
            end
            u64 = u64 >> offset & (1 << width - 1)
            return u64 % ty
        end
    end
end

function Base.setproperty!(x::Ptr{lws_route}, f::Symbol, v)
    fptr = getproperty(x, f)
    if fptr isa Ptr
        unsafe_store!(getproperty(x, f), v)
    else
        (baseptr, offset, width) = fptr
        baseptr32 = convert(Ptr{UInt32}, baseptr)
        u64 = unsafe_load(baseptr32)
        straddle = offset + width > 32
        if straddle
            u64 |= unsafe_load(baseptr32 + 4) << 32
        end
        mask = 1 << width - 1
        u64 &= ~(mask << offset)
        u64 |= (unsigned(v) & mask) << offset
        unsafe_store!(baseptr32, u64 & typemax(UInt32))
        if straddle
            unsafe_store!(baseptr32 + 4, u64 >> 32)
        end
    end
end

const lws_route_t = lws_route

const lws_dns_sort_t = lws_route_t

function lws_canonical_hostname(context)
    @ccall libwebsockets.lws_canonical_hostname(context::Ptr{lws_context})::Ptr{Cchar}
end

function lws_get_peer_addresses(wsi, fd, name, name_len, rip, rip_len)
    @ccall libwebsockets.lws_get_peer_addresses(wsi::Ptr{lws}, fd::lws_sockfd_type, name::Ptr{Cchar}, name_len::Cint, rip::Ptr{Cchar}, rip_len::Cint)::Cvoid
end

function lws_get_peer_simple(wsi, name, namelen)
    @ccall libwebsockets.lws_get_peer_simple(wsi::Ptr{lws}, name::Ptr{Cchar}, namelen::Csize_t)::Ptr{Cchar}
end

function lws_get_peer_simple_fd(fd, name, namelen)
    @ccall libwebsockets.lws_get_peer_simple_fd(fd::lws_sockfd_type, name::Ptr{Cchar}, namelen::Csize_t)::Ptr{Cchar}
end

function lws_interface_to_sa(ipv6, ifname, addr, addrlen)
    @ccall libwebsockets.lws_interface_to_sa(ipv6::Cint, ifname::Ptr{Cchar}, addr::Ptr{sockaddr_in}, addrlen::Csize_t)::Cint
end

function lws_sa46_compare_ads(sa46a, sa46b)
    @ccall libwebsockets.lws_sa46_compare_ads(sa46a::Ptr{lws_sockaddr46}, sa46b::Ptr{lws_sockaddr46})::Cint
end

function lws_sa46_on_net(sa46a, sa46_net, net_len)
    @ccall libwebsockets.lws_sa46_on_net(sa46a::Ptr{lws_sockaddr46}, sa46_net::Ptr{lws_sockaddr46}, net_len::Cint)::Cint
end

function lws_parse_numeric_address(ads, result, max_len)
    @ccall libwebsockets.lws_parse_numeric_address(ads::Ptr{Cchar}, result::Ptr{UInt8}, max_len::Csize_t)::Cint
end

function lws_sa46_parse_numeric_address(ads, sa46)
    @ccall libwebsockets.lws_sa46_parse_numeric_address(ads::Ptr{Cchar}, sa46::Ptr{lws_sockaddr46})::Cint
end

function lws_write_numeric_address(ads, size, buf, len)
    @ccall libwebsockets.lws_write_numeric_address(ads::Ptr{UInt8}, size::Cint, buf::Ptr{Cchar}, len::Csize_t)::Cint
end

function lws_sa46_write_numeric_address(sa46, buf, len)
    @ccall libwebsockets.lws_sa46_write_numeric_address(sa46::Ptr{lws_sockaddr46}, buf::Ptr{Cchar}, len::Csize_t)::Cint
end

const u_mt_t = UInt64

const __JL_Ctag_12 = UInt32
const LWSMTFL_REPORT_OUTLIERS = 1 % UInt32
const LWSMTFL_REPORT_OOB = 2 % UInt32
const LWSMTFL_REPORT_INACTIVITY_AT_PERIODIC = 4 % UInt32
const LWSMTFL_REPORT_MEAN = 8 % UInt32
const LWSMTFL_REPORT_ONLY_GO = 16 % UInt32
const LWSMTFL_REPORT_DUTY_WALLCLOCK_US = 32 % UInt32
const LWSMTFL_REPORT_HIST = 64 % UInt32

struct lws_metrics_tag
    list::lws_dll2_t
    name::Ptr{Cchar}
end

const lws_metrics_tag_t = lws_metrics_tag

function lws_metrics_tag_add(owner, name, val)
    @ccall libwebsockets.lws_metrics_tag_add(owner::Ptr{lws_dll2_owner_t}, name::Ptr{Cchar}, val::Ptr{Cchar})::Cint
end

function lws_metrics_tags_destroy(owner)
    @ccall libwebsockets.lws_metrics_tags_destroy(owner::Ptr{lws_dll2_owner_t})::Cvoid
end

function lws_metrics_tags_serialize(owner, buf, len)
    @ccall libwebsockets.lws_metrics_tags_serialize(owner::Ptr{lws_dll2_owner_t}, buf::Ptr{Cchar}, len::Csize_t)::Csize_t
end

function lws_metrics_tag_get(owner, name)
    @ccall libwebsockets.lws_metrics_tag_get(owner::Ptr{lws_dll2_owner_t}, name::Ptr{Cchar})::Ptr{Cchar}
end

struct lws_metric_bucket
    next::Ptr{lws_metric_bucket}
    count::UInt64
end

const lws_metric_bucket_t = lws_metric_bucket

struct __JL_Ctag_70
    data::NTuple{40, UInt8}
end

function Base.getproperty(x::Ptr{__JL_Ctag_70}, f::Symbol)
    f === :agg && return Ptr{__JL_Ctag_71}(x + 0)
    f === :hist && return Ptr{__JL_Ctag_72}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_70, f::Symbol)
    r = Ref{__JL_Ctag_70}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_70}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_70}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

struct lws_metric_pub
    data::NTuple{88, UInt8}
end

function Base.getproperty(x::Ptr{lws_metric_pub}, f::Symbol)
    f === :name && return Ptr{Ptr{Cchar}}(x + 0)
    f === :backend_opaque && return Ptr{Ptr{Cvoid}}(x + 8)
    f === :us_first && return Ptr{lws_usec_t}(x + 16)
    f === :us_last && return Ptr{lws_usec_t}(x + 24)
    f === :us_dumped && return Ptr{lws_usec_t}(x + 32)
    f === :u && return Ptr{__JL_Ctag_70}(x + 40)
    f === :flags && return Ptr{UInt8}(x + 80)
    return getfield(x, f)
end

function Base.getproperty(x::lws_metric_pub, f::Symbol)
    r = Ref{lws_metric_pub}(x)
    ptr = Base.unsafe_convert(Ptr{lws_metric_pub}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{lws_metric_pub}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

const lws_metric_pub_t = lws_metric_pub

function lws_metrics_hist_bump_priv_tagged(mt, tow, tow2)
    @ccall libwebsockets.lws_metrics_hist_bump_priv_tagged(mt::Ptr{lws_metric_pub_t}, tow::Ptr{lws_dll2_owner_t}, tow2::Ptr{lws_dll2_owner_t})::Cvoid
end

mutable struct lws_metric end

struct lws_metric_caliper
    mtags_owner::lws_dll2_owner
    mt::Ptr{lws_metric}
    us_start::lws_usec_t
end

const lws_metric_caliper_t = lws_metric_caliper

function lws_metrics_format(pub, sub, buf, len)
    @ccall libwebsockets.lws_metrics_format(pub::Ptr{lws_metric_pub_t}, sub::Ptr{Ptr{lws_metric_bucket_t}}, buf::Ptr{Cchar}, len::Csize_t)::Cint
end

function lws_metrics_hist_bump_(pub, name)
    @ccall libwebsockets.lws_metrics_hist_bump_(pub::Ptr{lws_metric_pub_t}, name::Ptr{Cchar})::Cint
end

function lws_metrics_foreach(ctx, user, cb)
    @ccall libwebsockets.lws_metrics_foreach(ctx::Ptr{lws_context}, user::Ptr{Cvoid}, cb::Ptr{Cvoid})::Cint
end

function lws_metrics_hist_bump_describe_wsi(wsi, pub, name)
    @ccall libwebsockets.lws_metrics_hist_bump_describe_wsi(wsi::Ptr{lws}, pub::Ptr{lws_metric_pub_t}, name::Ptr{Cchar})::Cint
end

const __JL_Ctag_13 = UInt32
const LMT_NORMAL = 0 % UInt32
const LMT_OUTLIER = 1 % UInt32
const LMT_FAIL = 2 % UInt32
const LMT_COUNT = 3 % UInt32

const lws_metric_rpt = UInt32
const LMR_PERIODIC = 0 % UInt32
const LMR_OUTLIER = 1 % UInt32

const lws_metric_rpt_kind_t = lws_metric_rpt

const lws_system_blob_item_t = UInt32
const LWS_SYSBLOB_TYPE_AUTH = 0 % UInt32
const LWS_SYSBLOB_TYPE_CLIENT_CERT_DER = 2 % UInt32
const LWS_SYSBLOB_TYPE_CLIENT_KEY_DER = 3 % UInt32
const LWS_SYSBLOB_TYPE_DEVICE_SERIAL = 4 % UInt32
const LWS_SYSBLOB_TYPE_DEVICE_FW_VERSION = 5 % UInt32
const LWS_SYSBLOB_TYPE_DEVICE_TYPE = 6 % UInt32
const LWS_SYSBLOB_TYPE_NTP_SERVER = 7 % UInt32
const LWS_SYSBLOB_TYPE_MQTT_CLIENT_ID = 8 % UInt32
const LWS_SYSBLOB_TYPE_MQTT_USERNAME = 9 % UInt32
const LWS_SYSBLOB_TYPE_MQTT_PASSWORD = 10 % UInt32
const LWS_SYSBLOB_TYPE_COUNT = 11 % UInt32

mutable struct lws_system_blob end

const lws_system_blob_t = lws_system_blob

function lws_system_blob_direct_set(b, ptr, len)
    @ccall libwebsockets.lws_system_blob_direct_set(b::Ptr{lws_system_blob_t}, ptr::Ptr{UInt8}, len::Csize_t)::Cvoid
end

function lws_system_blob_heap_empty(b)
    @ccall libwebsockets.lws_system_blob_heap_empty(b::Ptr{lws_system_blob_t})::Cvoid
end

function lws_system_blob_heap_append(b, ptr, len)
    @ccall libwebsockets.lws_system_blob_heap_append(b::Ptr{lws_system_blob_t}, ptr::Ptr{UInt8}, len::Csize_t)::Cint
end

function lws_system_blob_get_size(b)
    @ccall libwebsockets.lws_system_blob_get_size(b::Ptr{lws_system_blob_t})::Csize_t
end

function lws_system_blob_get_single_ptr(b, ptr)
    @ccall libwebsockets.lws_system_blob_get_single_ptr(b::Ptr{lws_system_blob_t}, ptr::Ptr{Ptr{UInt8}})::Cint
end

function lws_system_blob_get(b, ptr, len, ofs)
    @ccall libwebsockets.lws_system_blob_get(b::Ptr{lws_system_blob_t}, ptr::Ptr{UInt8}, len::Ptr{Csize_t}, ofs::Csize_t)::Cint
end

function lws_system_blob_destroy(b)
    @ccall libwebsockets.lws_system_blob_destroy(b::Ptr{lws_system_blob_t})::Cvoid
end

function lws_system_get_blob(context, type, idx)
    @ccall libwebsockets.lws_system_get_blob(context::Ptr{lws_context}, type::lws_system_blob_item_t, idx::Cint)::Ptr{lws_system_blob_t}
end

const lws_system_states_t = UInt32
const LWS_SYSTATE_UNKNOWN = 0 % UInt32
const LWS_SYSTATE_CONTEXT_CREATED = 1 % UInt32
const LWS_SYSTATE_INITIALIZED = 2 % UInt32
const LWS_SYSTATE_IFACE_COLDPLUG = 3 % UInt32
const LWS_SYSTATE_DHCP = 4 % UInt32
const LWS_SYSTATE_CPD_PRE_TIME = 5 % UInt32
const LWS_SYSTATE_TIME_VALID = 6 % UInt32
const LWS_SYSTATE_CPD_POST_TIME = 7 % UInt32
const LWS_SYSTATE_POLICY_VALID = 8 % UInt32
const LWS_SYSTATE_REGISTERED = 9 % UInt32
const LWS_SYSTATE_AUTH1 = 10 % UInt32
const LWS_SYSTATE_AUTH2 = 11 % UInt32
const LWS_SYSTATE_OPERATIONAL = 12 % UInt32
const LWS_SYSTATE_POLICY_INVALID = 13 % UInt32
const LWS_SYSTATE_CONTEXT_DESTROYING = 14 % UInt32

const lws_cpd_result_t = UInt32
const LWS_CPD_UNKNOWN = 0 % UInt32
const LWS_CPD_INTERNET_OK = 1 % UInt32
const LWS_CPD_CAPTIVE_PORTAL = 2 % UInt32
const LWS_CPD_NO_INTERNET = 3 % UInt32

# typedef void ( * lws_attach_cb_t ) ( struct lws_context * context , int tsi , void * opaque )
const lws_attach_cb_t = Ptr{Cvoid}

mutable struct lws_attach_item end

function lws_tls_jit_trust_got_cert_cb(cx, got_opaque, skid, skid_len, der, der_len)
    @ccall libwebsockets.lws_tls_jit_trust_got_cert_cb(cx::Ptr{lws_context}, got_opaque::Ptr{Cvoid}, skid::Ptr{UInt8}, skid_len::Csize_t, der::Ptr{UInt8}, der_len::Csize_t)::Cint
end

struct lws_system_ops
    reboot::Ptr{Cvoid}
    set_clock::Ptr{Cvoid}
    attach::Ptr{Cvoid}
    captive_portal_detect_request::Ptr{Cvoid}
    metric_report::Ptr{Cvoid}
    jit_trust_query::Ptr{Cvoid}
    wake_latency_us::UInt32
end

const lws_system_ops_t = lws_system_ops

function lws_system_get_state_manager(context)
    @ccall libwebsockets.lws_system_get_state_manager(context::Ptr{lws_context})::Ptr{lws_state_manager_t}
end

function lws_system_get_ops(context)
    @ccall libwebsockets.lws_system_get_ops(context::Ptr{lws_context})::Ptr{lws_system_ops_t}
end

function lws_system_context_from_system_mgr(mgr)
    @ccall libwebsockets.lws_system_context_from_system_mgr(mgr::Ptr{lws_state_manager_t})::Ptr{lws_context}
end

function __lws_system_attach(context, tsi, cb, state, opaque, get)
    @ccall libwebsockets.__lws_system_attach(context::Ptr{lws_context}, tsi::Cint, cb::lws_attach_cb_t, state::lws_system_states_t, opaque::Ptr{Cvoid}, get::Ptr{Ptr{lws_attach_item}})::Cint
end

const __JL_Ctag_17 = UInt32
const LWSDH_IPV4_SUBNET_MASK = 0 % UInt32
const LWSDH_IPV4_BROADCAST = 1 % UInt32
const LWSDH_LEASE_SECS = 2 % UInt32
const LWSDH_REBINDING_SECS = 3 % UInt32
const LWSDH_RENEWAL_SECS = 4 % UInt32
const _LWSDH_NUMS_COUNT = 5 % UInt32
const LWSDH_SA46_IP = 0 % UInt32
const LWSDH_SA46_DNS_SRV_1 = 1 % UInt32
const LWSDH_SA46_DNS_SRV_2 = 2 % UInt32
const LWSDH_SA46_DNS_SRV_3 = 3 % UInt32
const LWSDH_SA46_DNS_SRV_4 = 4 % UInt32
const LWSDH_SA46_IPV4_ROUTER = 5 % UInt32
const LWSDH_SA46_NTP_SERVER = 6 % UInt32
const LWSDH_SA46_DHCP_SERVER = 7 % UInt32
const _LWSDH_SA46_COUNT = 8 % UInt32

struct lws_dhcpc_ifstate
    ifname::NTuple{16, Cchar}
    domain::NTuple{64, Cchar}
    mac::NTuple{6, UInt8}
    nums::NTuple{5, UInt32}
    sa46::NTuple{8, lws_sockaddr46}
end

const lws_dhcpc_ifstate_t = lws_dhcpc_ifstate

# typedef int ( * dhcpc_cb_t ) ( void * opaque , lws_dhcpc_ifstate_t * is )
const dhcpc_cb_t = Ptr{Cvoid}

function lws_dhcpc_request(c, i, af, cb, opaque)
    @ccall libwebsockets.lws_dhcpc_request(c::Ptr{lws_context}, i::Ptr{Cchar}, af::Cint, cb::dhcpc_cb_t, opaque::Ptr{Cvoid})::Cint
end

function lws_dhcpc_remove(context, iface)
    @ccall libwebsockets.lws_dhcpc_remove(context::Ptr{lws_context}, iface::Ptr{Cchar})::Cint
end

function lws_dhcpc_status(context, sa46)
    @ccall libwebsockets.lws_dhcpc_status(context::Ptr{lws_context}, sa46::Ptr{lws_sockaddr46})::Cint
end

function lws_system_cpd_start(context)
    @ccall libwebsockets.lws_system_cpd_start(context::Ptr{lws_context})::Cint
end

function lws_system_cpd_start_defer(cx, defer_us)
    @ccall libwebsockets.lws_system_cpd_start_defer(cx::Ptr{lws_context}, defer_us::lws_usec_t)::Cvoid
end

function lws_system_cpd_set(context, result)
    @ccall libwebsockets.lws_system_cpd_set(context::Ptr{lws_context}, result::lws_cpd_result_t)::Cvoid
end

function lws_system_cpd_state_get(context)
    @ccall libwebsockets.lws_system_cpd_state_get(context::Ptr{lws_context})::lws_cpd_result_t
end

const lws_close_status = UInt32
const LWS_CLOSE_STATUS_NOSTATUS = 0 % UInt32
const LWS_CLOSE_STATUS_NORMAL = 1000 % UInt32
const LWS_CLOSE_STATUS_GOINGAWAY = 1001 % UInt32
const LWS_CLOSE_STATUS_PROTOCOL_ERR = 1002 % UInt32
const LWS_CLOSE_STATUS_UNACCEPTABLE_OPCODE = 1003 % UInt32
const LWS_CLOSE_STATUS_RESERVED = 1004 % UInt32
const LWS_CLOSE_STATUS_NO_STATUS = 1005 % UInt32
const LWS_CLOSE_STATUS_ABNORMAL_CLOSE = 1006 % UInt32
const LWS_CLOSE_STATUS_INVALID_PAYLOAD = 1007 % UInt32
const LWS_CLOSE_STATUS_POLICY_VIOLATION = 1008 % UInt32
const LWS_CLOSE_STATUS_MESSAGE_TOO_LARGE = 1009 % UInt32
const LWS_CLOSE_STATUS_EXTENSION_REQUIRED = 1010 % UInt32
const LWS_CLOSE_STATUS_UNEXPECTED_CONDITION = 1011 % UInt32
const LWS_CLOSE_STATUS_TLS_FAILURE = 1015 % UInt32
const LWS_CLOSE_STATUS_CLIENT_TRANSACTION_DONE = 2000 % UInt32
const LWS_CLOSE_STATUS_NOSTATUS_CONTEXT_DESTROY = 9999 % UInt32

function lws_close_reason(wsi, status, buf, len)
    @ccall libwebsockets.lws_close_reason(wsi::Ptr{lws}, status::lws_close_status, buf::Ptr{Cuchar}, len::Csize_t)::Cvoid
end

struct lws_ssl_info
    where::Cint
    ret::Cint
end

const lws_cert_update_state = UInt32
const LWS_CUS_IDLE = 0 % UInt32
const LWS_CUS_STARTING = 1 % UInt32
const LWS_CUS_SUCCESS = 2 % UInt32
const LWS_CUS_FAILED = 3 % UInt32
const LWS_CUS_CREATE_KEYS = 4 % UInt32
const LWS_CUS_REG = 5 % UInt32
const LWS_CUS_AUTH = 6 % UInt32
const LWS_CUS_CHALLENGE = 7 % UInt32
const LWS_CUS_CREATE_REQ = 8 % UInt32
const LWS_CUS_REQ = 9 % UInt32
const LWS_CUS_CONFIRM = 10 % UInt32
const LWS_CUS_ISSUE = 11 % UInt32

const __JL_Ctag_18 = UInt32
const LWS_TLS_REQ_ELEMENT_COUNTRY = 0 % UInt32
const LWS_TLS_REQ_ELEMENT_STATE = 1 % UInt32
const LWS_TLS_REQ_ELEMENT_LOCALITY = 2 % UInt32
const LWS_TLS_REQ_ELEMENT_ORGANIZATION = 3 % UInt32
const LWS_TLS_REQ_ELEMENT_COMMON_NAME = 4 % UInt32
const LWS_TLS_REQ_ELEMENT_SUBJECT_ALT_NAME = 5 % UInt32
const LWS_TLS_REQ_ELEMENT_EMAIL = 6 % UInt32
const LWS_TLS_REQ_ELEMENT_COUNT = 7 % UInt32
const LWS_TLS_SET_DIR_URL = 7 % UInt32
const LWS_TLS_SET_AUTH_PATH = 8 % UInt32
const LWS_TLS_SET_CERT_PATH = 9 % UInt32
const LWS_TLS_SET_KEY_PATH = 10 % UInt32
const LWS_TLS_TOTAL_COUNT = 11 % UInt32

struct lws_acme_cert_aging_args
    vh::Ptr{lws_vhost}
    element_overrides::NTuple{11, Ptr{Cchar}}
end

struct lws_filter_network_conn_args
    cli_addr::sockaddr_storage
    clilen::socklen_t
    accept_fd::lws_sockfd_type
end

const lws_callback_reasons = UInt32
const LWS_CALLBACK_PROTOCOL_INIT = 27 % UInt32
const LWS_CALLBACK_PROTOCOL_DESTROY = 28 % UInt32
const LWS_CALLBACK_WSI_CREATE = 29 % UInt32
const LWS_CALLBACK_WSI_DESTROY = 30 % UInt32
const LWS_CALLBACK_WSI_TX_CREDIT_GET = 103 % UInt32
const LWS_CALLBACK_OPENSSL_LOAD_EXTRA_CLIENT_VERIFY_CERTS = 21 % UInt32
const LWS_CALLBACK_OPENSSL_LOAD_EXTRA_SERVER_VERIFY_CERTS = 22 % UInt32
const LWS_CALLBACK_OPENSSL_PERFORM_CLIENT_CERT_VERIFICATION = 23 % UInt32
const LWS_CALLBACK_SSL_INFO = 67 % UInt32
const LWS_CALLBACK_OPENSSL_PERFORM_SERVER_CERT_VERIFICATION = 58 % UInt32
const LWS_CALLBACK_SERVER_NEW_CLIENT_INSTANTIATED = 19 % UInt32
const LWS_CALLBACK_HTTP = 12 % UInt32
const LWS_CALLBACK_HTTP_BODY = 13 % UInt32
const LWS_CALLBACK_HTTP_BODY_COMPLETION = 14 % UInt32
const LWS_CALLBACK_HTTP_FILE_COMPLETION = 15 % UInt32
const LWS_CALLBACK_HTTP_WRITEABLE = 16 % UInt32
const LWS_CALLBACK_CLOSED_HTTP = 5 % UInt32
const LWS_CALLBACK_FILTER_HTTP_CONNECTION = 18 % UInt32
const LWS_CALLBACK_ADD_HEADERS = 53 % UInt32
const LWS_CALLBACK_VERIFY_BASIC_AUTHORIZATION = 102 % UInt32
const LWS_CALLBACK_CHECK_ACCESS_RIGHTS = 51 % UInt32
const LWS_CALLBACK_PROCESS_HTML = 52 % UInt32
const LWS_CALLBACK_HTTP_BIND_PROTOCOL = 49 % UInt32
const LWS_CALLBACK_HTTP_DROP_PROTOCOL = 50 % UInt32
const LWS_CALLBACK_HTTP_CONFIRM_UPGRADE = 86 % UInt32
const LWS_CALLBACK_ESTABLISHED_CLIENT_HTTP = 44 % UInt32
const LWS_CALLBACK_CLOSED_CLIENT_HTTP = 45 % UInt32
const LWS_CALLBACK_RECEIVE_CLIENT_HTTP_READ = 48 % UInt32
const LWS_CALLBACK_RECEIVE_CLIENT_HTTP = 46 % UInt32
const LWS_CALLBACK_COMPLETED_CLIENT_HTTP = 47 % UInt32
const LWS_CALLBACK_CLIENT_HTTP_WRITEABLE = 57 % UInt32
const LWS_CALLBACK_CLIENT_HTTP_REDIRECT = 104 % UInt32
const LWS_CALLBACK_CLIENT_HTTP_BIND_PROTOCOL = 85 % UInt32
const LWS_CALLBACK_CLIENT_HTTP_DROP_PROTOCOL = 76 % UInt32
const LWS_CALLBACK_ESTABLISHED = 0 % UInt32
const LWS_CALLBACK_CLOSED = 4 % UInt32
const LWS_CALLBACK_SERVER_WRITEABLE = 11 % UInt32
const LWS_CALLBACK_RECEIVE = 6 % UInt32
const LWS_CALLBACK_RECEIVE_PONG = 7 % UInt32
const LWS_CALLBACK_WS_PEER_INITIATED_CLOSE = 38 % UInt32
const LWS_CALLBACK_FILTER_PROTOCOL_CONNECTION = 20 % UInt32
const LWS_CALLBACK_CONFIRM_EXTENSION_OKAY = 25 % UInt32
const LWS_CALLBACK_WS_SERVER_BIND_PROTOCOL = 77 % UInt32
const LWS_CALLBACK_WS_SERVER_DROP_PROTOCOL = 78 % UInt32
const LWS_CALLBACK_CLIENT_CONNECTION_ERROR = 1 % UInt32
const LWS_CALLBACK_CLIENT_FILTER_PRE_ESTABLISH = 2 % UInt32
const LWS_CALLBACK_CLIENT_ESTABLISHED = 3 % UInt32
const LWS_CALLBACK_CLIENT_CLOSED = 75 % UInt32
const LWS_CALLBACK_CLIENT_APPEND_HANDSHAKE_HEADER = 24 % UInt32
const LWS_CALLBACK_CLIENT_RECEIVE = 8 % UInt32
const LWS_CALLBACK_CLIENT_RECEIVE_PONG = 9 % UInt32
const LWS_CALLBACK_CLIENT_WRITEABLE = 10 % UInt32
const LWS_CALLBACK_CLIENT_CONFIRM_EXTENSION_SUPPORTED = 26 % UInt32
const LWS_CALLBACK_WS_EXT_DEFAULTS = 39 % UInt32
const LWS_CALLBACK_FILTER_NETWORK_CONNECTION = 17 % UInt32
const LWS_CALLBACK_WS_CLIENT_BIND_PROTOCOL = 79 % UInt32
const LWS_CALLBACK_WS_CLIENT_DROP_PROTOCOL = 80 % UInt32
const LWS_CALLBACK_GET_THREAD_ID = 31 % UInt32
const LWS_CALLBACK_ADD_POLL_FD = 32 % UInt32
const LWS_CALLBACK_DEL_POLL_FD = 33 % UInt32
const LWS_CALLBACK_CHANGE_MODE_POLL_FD = 34 % UInt32
const LWS_CALLBACK_LOCK_POLL = 35 % UInt32
const LWS_CALLBACK_UNLOCK_POLL = 36 % UInt32
const LWS_CALLBACK_CGI = 40 % UInt32
const LWS_CALLBACK_CGI_TERMINATED = 41 % UInt32
const LWS_CALLBACK_CGI_STDIN_DATA = 42 % UInt32
const LWS_CALLBACK_CGI_STDIN_COMPLETED = 43 % UInt32
const LWS_CALLBACK_CGI_PROCESS_ATTACH = 70 % UInt32
const LWS_CALLBACK_SESSION_INFO = 54 % UInt32
const LWS_CALLBACK_GS_EVENT = 55 % UInt32
const LWS_CALLBACK_HTTP_PMO = 56 % UInt32
const LWS_CALLBACK_RAW_PROXY_CLI_RX = 89 % UInt32
const LWS_CALLBACK_RAW_PROXY_SRV_RX = 90 % UInt32
const LWS_CALLBACK_RAW_PROXY_CLI_CLOSE = 91 % UInt32
const LWS_CALLBACK_RAW_PROXY_SRV_CLOSE = 92 % UInt32
const LWS_CALLBACK_RAW_PROXY_CLI_WRITEABLE = 93 % UInt32
const LWS_CALLBACK_RAW_PROXY_SRV_WRITEABLE = 94 % UInt32
const LWS_CALLBACK_RAW_PROXY_CLI_ADOPT = 95 % UInt32
const LWS_CALLBACK_RAW_PROXY_SRV_ADOPT = 96 % UInt32
const LWS_CALLBACK_RAW_PROXY_CLI_BIND_PROTOCOL = 97 % UInt32
const LWS_CALLBACK_RAW_PROXY_SRV_BIND_PROTOCOL = 98 % UInt32
const LWS_CALLBACK_RAW_PROXY_CLI_DROP_PROTOCOL = 99 % UInt32
const LWS_CALLBACK_RAW_PROXY_SRV_DROP_PROTOCOL = 100 % UInt32
const LWS_CALLBACK_RAW_RX = 59 % UInt32
const LWS_CALLBACK_RAW_CLOSE = 60 % UInt32
const LWS_CALLBACK_RAW_WRITEABLE = 61 % UInt32
const LWS_CALLBACK_RAW_ADOPT = 62 % UInt32
const LWS_CALLBACK_RAW_CONNECTED = 101 % UInt32
const LWS_CALLBACK_RAW_SKT_BIND_PROTOCOL = 81 % UInt32
const LWS_CALLBACK_RAW_SKT_DROP_PROTOCOL = 82 % UInt32
const LWS_CALLBACK_RAW_ADOPT_FILE = 63 % UInt32
const LWS_CALLBACK_RAW_RX_FILE = 64 % UInt32
const LWS_CALLBACK_RAW_WRITEABLE_FILE = 65 % UInt32
const LWS_CALLBACK_RAW_CLOSE_FILE = 66 % UInt32
const LWS_CALLBACK_RAW_FILE_BIND_PROTOCOL = 83 % UInt32
const LWS_CALLBACK_RAW_FILE_DROP_PROTOCOL = 84 % UInt32
const LWS_CALLBACK_TIMER = 73 % UInt32
const LWS_CALLBACK_EVENT_WAIT_CANCELLED = 71 % UInt32
const LWS_CALLBACK_CHILD_CLOSING = 69 % UInt32
const LWS_CALLBACK_CONNECTING = 105 % UInt32
const LWS_CALLBACK_VHOST_CERT_AGING = 72 % UInt32
const LWS_CALLBACK_VHOST_CERT_UPDATE = 74 % UInt32
const LWS_CALLBACK_MQTT_NEW_CLIENT_INSTANTIATED = 200 % UInt32
const LWS_CALLBACK_MQTT_IDLE = 201 % UInt32
const LWS_CALLBACK_MQTT_CLIENT_ESTABLISHED = 202 % UInt32
const LWS_CALLBACK_MQTT_SUBSCRIBED = 203 % UInt32
const LWS_CALLBACK_MQTT_CLIENT_WRITEABLE = 204 % UInt32
const LWS_CALLBACK_MQTT_CLIENT_RX = 205 % UInt32
const LWS_CALLBACK_MQTT_UNSUBSCRIBED = 206 % UInt32
const LWS_CALLBACK_MQTT_DROP_PROTOCOL = 207 % UInt32
const LWS_CALLBACK_MQTT_CLIENT_CLOSED = 208 % UInt32
const LWS_CALLBACK_MQTT_ACK = 209 % UInt32
const LWS_CALLBACK_MQTT_RESEND = 210 % UInt32
const LWS_CALLBACK_MQTT_UNSUBSCRIBE_TIMEOUT = 211 % UInt32
const LWS_CALLBACK_MQTT_SHADOW_TIMEOUT = 212 % UInt32
const LWS_CALLBACK_USER = 1000 % UInt32

# typedef int lws_callback_function ( struct lws * wsi , enum lws_callback_reasons reason , void * user , void * in , size_t len )
const lws_callback_function = Cvoid

function lws_send_pipe_choked(wsi)
    @ccall libwebsockets.lws_send_pipe_choked(wsi::Ptr{lws})::Cint
end

function lws_is_final_fragment(wsi)
    @ccall libwebsockets.lws_is_final_fragment(wsi::Ptr{lws})::Cint
end

function lws_is_first_fragment(wsi)
    @ccall libwebsockets.lws_is_first_fragment(wsi::Ptr{lws})::Cint
end

function lws_get_reserved_bits(wsi)
    @ccall libwebsockets.lws_get_reserved_bits(wsi::Ptr{lws})::Cuchar
end

function lws_partial_buffered(wsi)
    @ccall libwebsockets.lws_partial_buffered(wsi::Ptr{lws})::Cint
end

function lws_frame_is_binary(wsi)
    @ccall libwebsockets.lws_frame_is_binary(wsi::Ptr{lws})::Cint
end

const lws_extension_callback_reasons = UInt32
const LWS_EXT_CB_CONSTRUCT = 4 % UInt32
const LWS_EXT_CB_CLIENT_CONSTRUCT = 5 % UInt32
const LWS_EXT_CB_DESTROY = 8 % UInt32
const LWS_EXT_CB_PACKET_TX_PRESEND = 12 % UInt32
const LWS_EXT_CB_PAYLOAD_TX = 21 % UInt32
const LWS_EXT_CB_PAYLOAD_RX = 22 % UInt32
const LWS_EXT_CB_OPTION_DEFAULT = 23 % UInt32
const LWS_EXT_CB_OPTION_SET = 24 % UInt32
const LWS_EXT_CB_OPTION_CONFIRM = 25 % UInt32
const LWS_EXT_CB_NAMED_OPTION_SET = 26 % UInt32

const lws_ext_options_types = UInt32
const EXTARG_NONE = 0 % UInt32
const EXTARG_DEC = 1 % UInt32
const EXTARG_OPT_DEC = 2 % UInt32

struct lws_ext_options
    name::Ptr{Cchar}
    type::lws_ext_options_types
end

struct lws_ext_option_arg
    option_name::Ptr{Cchar}
    option_index::Cint
    start::Ptr{Cchar}
    len::Cint
end

# typedef int lws_extension_callback_function ( struct lws_context * context , const struct lws_extension * ext , struct lws * wsi , enum lws_extension_callback_reasons reason , void * user , void * in , size_t len )
const lws_extension_callback_function = Cvoid

struct lws_extension
    name::Ptr{Cchar}
    callback::Ptr{lws_extension_callback_function}
    client_offer::Ptr{Cchar}
end

function lws_set_extension_option(wsi, ext_name, opt_name, opt_val)
    @ccall libwebsockets.lws_set_extension_option(wsi::Ptr{lws}, ext_name::Ptr{Cchar}, opt_name::Ptr{Cchar}, opt_val::Ptr{Cchar})::Cint
end

function lws_ext_parse_options(ext, wsi, ext_user, opts, o, len)
    @ccall libwebsockets.lws_ext_parse_options(ext::Ptr{lws_extension}, wsi::Ptr{lws}, ext_user::Ptr{Cvoid}, opts::Ptr{lws_ext_options}, o::Ptr{Cchar}, len::Cint)::Cint
end

function lws_extension_callback_pm_deflate(context, ext, wsi, reason, user, in, len)
    @ccall libwebsockets.lws_extension_callback_pm_deflate(context::Ptr{lws_context}, ext::Ptr{lws_extension}, wsi::Ptr{lws}, reason::lws_extension_callback_reasons, user::Ptr{Cvoid}, in::Ptr{Cvoid}, len::Csize_t)::Cint
end

struct lws_protocols
    name::Ptr{Cchar}
    callback::Ptr{lws_callback_function}
    per_session_data_size::Csize_t
    rx_buffer_size::Csize_t
    id::Cuint
    user::Ptr{Cvoid}
    tx_packet_size::Csize_t
end

function lws_vhost_name_to_protocol(vh, name)
    @ccall libwebsockets.lws_vhost_name_to_protocol(vh::Ptr{lws_vhost}, name::Ptr{Cchar})::Ptr{lws_protocols}
end

function lws_get_protocol(wsi)
    @ccall libwebsockets.lws_get_protocol(wsi::Ptr{lws})::Ptr{lws_protocols}
end

function lws_protocol_get(wsi)
    @ccall libwebsockets.lws_protocol_get(wsi::Ptr{lws})::Ptr{lws_protocols}
end

function lws_protocol_vh_priv_zalloc(vhost, prot, size)
    @ccall libwebsockets.lws_protocol_vh_priv_zalloc(vhost::Ptr{lws_vhost}, prot::Ptr{lws_protocols}, size::Cint)::Ptr{Cvoid}
end

function lws_protocol_vh_priv_get(vhost, prot)
    @ccall libwebsockets.lws_protocol_vh_priv_get(vhost::Ptr{lws_vhost}, prot::Ptr{lws_protocols})::Ptr{Cvoid}
end

function lws_vhd_find_by_pvo(cx, protname, pvo_name, pvo_value)
    @ccall libwebsockets.lws_vhd_find_by_pvo(cx::Ptr{lws_context}, protname::Ptr{Cchar}, pvo_name::Ptr{Cchar}, pvo_value::Ptr{Cchar})::Ptr{Cvoid}
end

function lws_adjust_protocol_psds(wsi, new_size)
    @ccall libwebsockets.lws_adjust_protocol_psds(wsi::Ptr{lws}, new_size::Csize_t)::Ptr{Cvoid}
end

function lws_finalize_startup(context)
    @ccall libwebsockets.lws_finalize_startup(context::Ptr{lws_context})::Cint
end

struct lws_protocol_vhost_options
    next::Ptr{lws_protocol_vhost_options}
    options::Ptr{lws_protocol_vhost_options}
    name::Ptr{Cchar}
    value::Ptr{Cchar}
end

function lws_pvo_search(pvo, name)
    @ccall libwebsockets.lws_pvo_search(pvo::Ptr{lws_protocol_vhost_options}, name::Ptr{Cchar})::Ptr{lws_protocol_vhost_options}
end

function lws_pvo_get_str(in, name, result)
    @ccall libwebsockets.lws_pvo_get_str(in::Ptr{Cvoid}, name::Ptr{Cchar}, result::Ptr{Ptr{Cchar}})::Cint
end

function lws_protocol_init(context)
    @ccall libwebsockets.lws_protocol_init(context::Ptr{lws_context})::Cint
end

struct lws_plugin_header
    name::Ptr{Cchar}
    _class::Ptr{Cchar}
    lws_build_hash::Ptr{Cchar}
    api_magic::Cuint
end

const lws_plugin_header_t = lws_plugin_header

struct lws_plugin_protocol
    hdr::lws_plugin_header_t
    protocols::Ptr{lws_protocols}
    extensions::Ptr{lws_extension}
    count_protocols::Cint
    count_extensions::Cint
end

const lws_plugin_protocol_t = lws_plugin_protocol

struct __JL_Ctag_69
    data::NTuple{8, UInt8}
end

function Base.getproperty(x::Ptr{__JL_Ctag_69}, f::Symbol)
    f === :l && return Ptr{Ptr{Cvoid}}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_69, f::Symbol)
    r = Ref{__JL_Ctag_69}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_69}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_69}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

struct lws_plugin
    data::NTuple{24, UInt8}
end

function Base.getproperty(x::Ptr{lws_plugin}, f::Symbol)
    f === :list && return Ptr{Ptr{lws_plugin}}(x + 0)
    f === :hdr && return Ptr{Ptr{lws_plugin_header_t}}(x + 8)
    f === :u && return Ptr{__JL_Ctag_69}(x + 16)
    return getfield(x, f)
end

function Base.getproperty(x::lws_plugin, f::Symbol)
    r = Ref{lws_plugin}(x)
    ptr = Base.unsafe_convert(Ptr{lws_plugin}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{lws_plugin}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

struct lws_event_loop_ops
    name::Ptr{Cchar}
    init_context::Ptr{Cvoid}
    destroy_context1::Ptr{Cvoid}
    destroy_context2::Ptr{Cvoid}
    init_vhost_listen_wsi::Ptr{Cvoid}
    init_pt::Ptr{Cvoid}
    wsi_logical_close::Ptr{Cvoid}
    check_client_connect_ok::Ptr{Cvoid}
    close_handle_manually::Ptr{Cvoid}
    sock_accept::Ptr{Cvoid}
    io::Ptr{Cvoid}
    run_pt::Ptr{Cvoid}
    destroy_pt::Ptr{Cvoid}
    destroy_wsi::Ptr{Cvoid}
    foreign_thread::Ptr{Cvoid}
    flags::UInt8
    evlib_size_ctx::UInt16
    evlib_size_pt::UInt16
    evlib_size_vh::UInt16
    evlib_size_wsi::UInt16
end

struct lws_plugin_evlib
    hdr::lws_plugin_header_t
    ops::Ptr{lws_event_loop_ops}
end

const lws_plugin_evlib_t = lws_plugin_evlib

# typedef int ( * each_plugin_cb_t ) ( struct lws_plugin * p , void * user )
const each_plugin_cb_t = Ptr{Cvoid}

function lws_plugins_init(pplugin, d, _class, filter, each, each_user)
    @ccall libwebsockets.lws_plugins_init(pplugin::Ptr{Ptr{lws_plugin}}, d::Ptr{Ptr{Cchar}}, _class::Ptr{Cchar}, filter::Ptr{Cchar}, each::each_plugin_cb_t, each_user::Ptr{Cvoid})::Cint
end

function lws_plugins_destroy(pplugin, each, each_user)
    @ccall libwebsockets.lws_plugins_destroy(pplugin::Ptr{Ptr{lws_plugin}}, each::each_plugin_cb_t, each_user::Ptr{Cvoid})::Cint
end

mutable struct lws_ss_plugin end

# typedef int ( * lws_context_ready_cb_t ) ( struct lws_context * context )
const lws_context_ready_cb_t = Ptr{Cvoid}

# typedef int ( * lws_peer_limits_notify_t ) ( struct lws_context * ctx , lws_sockfd_type sockfd , lws_sockaddr46 * sa46 )
const lws_peer_limits_notify_t = Ptr{Cvoid}

struct lws_context_creation_info
    iface::Ptr{Cchar}
    protocols::Ptr{lws_protocols}
    extensions::Ptr{lws_extension}
    token_limits::Ptr{lws_token_limits}
    http_proxy_address::Ptr{Cchar}
    headers::Ptr{lws_protocol_vhost_options}
    reject_service_keywords::Ptr{lws_protocol_vhost_options}
    pvo::Ptr{lws_protocol_vhost_options}
    log_filepath::Ptr{Cchar}
    mounts::Ptr{lws_http_mount}
    server_string::Ptr{Cchar}
    error_document_404::Ptr{Cchar}
    port::Cint
    http_proxy_port::Cuint
    max_http_header_data2::Cuint
    max_http_header_pool2::Cuint
    keepalive_timeout::Cint
    http2_settings::NTuple{7, UInt32}
    max_http_header_data::Cushort
    max_http_header_pool::Cushort
    ssl_private_key_password::Ptr{Cchar}
    ssl_cert_filepath::Ptr{Cchar}
    ssl_private_key_filepath::Ptr{Cchar}
    ssl_ca_filepath::Ptr{Cchar}
    ssl_cipher_list::Ptr{Cchar}
    ecdh_curve::Ptr{Cchar}
    tls1_3_plus_cipher_list::Ptr{Cchar}
    server_ssl_cert_mem::Ptr{Cvoid}
    server_ssl_private_key_mem::Ptr{Cvoid}
    server_ssl_ca_mem::Ptr{Cvoid}
    ssl_options_set::Clong
    ssl_options_clear::Clong
    simultaneous_ssl_restriction::Cint
    simultaneous_ssl_handshake_restriction::Cint
    ssl_info_event_mask::Cint
    server_ssl_cert_mem_len::Cuint
    server_ssl_private_key_mem_len::Cuint
    server_ssl_ca_mem_len::Cuint
    alpn::Ptr{Cchar}
    client_ssl_private_key_password::Ptr{Cchar}
    client_ssl_cert_filepath::Ptr{Cchar}
    client_ssl_cert_mem::Ptr{Cvoid}
    client_ssl_cert_mem_len::Cuint
    client_ssl_private_key_filepath::Ptr{Cchar}
    client_ssl_key_mem::Ptr{Cvoid}
    client_ssl_ca_filepath::Ptr{Cchar}
    client_ssl_ca_mem::Ptr{Cvoid}
    client_ssl_cipher_list::Ptr{Cchar}
    client_tls_1_3_plus_cipher_list::Ptr{Cchar}
    ssl_client_options_set::Clong
    ssl_client_options_clear::Clong
    client_ssl_ca_mem_len::Cuint
    client_ssl_key_mem_len::Cuint
    provided_client_ssl_ctx::Ptr{Cint}
    ka_time::Cint
    ka_probes::Cint
    ka_interval::Cint
    timeout_secs::Cuint
    connect_timeout_secs::Cuint
    bind_iface::Cint
    timeout_secs_ah_idle::Cuint
    tls_session_timeout::UInt32
    tls_session_cache_max::UInt32
    gid::gid_t
    uid::uid_t
    options::UInt64
    user::Ptr{Cvoid}
    count_threads::Cuint
    fd_limit_per_thread::Cuint
    vhost_name::Ptr{Cchar}
    external_baggage_free_on_destroy::Ptr{Cvoid}
    pt_serv_buf_size::Cuint
    fops::Ptr{lws_plat_file_ops}
    foreign_loops::Ptr{Ptr{Cvoid}}
    signal_cb::Ptr{Cvoid}
    pcontext::Ptr{Ptr{lws_context}}
    finalize::Ptr{Cvoid}
    finalize_arg::Ptr{Cvoid}
    listen_accept_role::Ptr{Cchar}
    listen_accept_protocol::Ptr{Cchar}
    pprotocols::Ptr{Ptr{lws_protocols}}
    username::Ptr{Cchar}
    groupname::Ptr{Cchar}
    unix_socket_perms::Ptr{Cchar}
    system_ops::Ptr{lws_system_ops_t}
    retry_and_idle_policy::Ptr{lws_retry_bo_t}
    register_notifier_list::Ptr{Ptr{lws_state_notify_link_t}}
    rlimit_nofile::Cint
    early_smd_cb::lws_smd_notification_cb_t
    early_smd_opaque::Ptr{Cvoid}
    early_smd_class_filter::lws_smd_class_t
    smd_ttl_us::lws_usec_t
    smd_queue_depth::UInt16
    fo_listen_queue::Cint
    event_lib_custom::Ptr{lws_plugin_evlib}
    log_cx::Ptr{lws_log_cx_t}
    http_nsc_filepath::Ptr{Cchar}
    http_nsc_heap_max_footprint::Csize_t
    http_nsc_heap_max_items::Csize_t
    http_nsc_heap_max_payload::Csize_t
    _unused::NTuple{2, Ptr{Cvoid}}
end

function lws_create_context(info)
    @ccall libwebsockets.lws_create_context(info::Ptr{lws_context_creation_info})::Ptr{lws_context}
end

function lws_context_destroy(context)
    @ccall libwebsockets.lws_context_destroy(context::Ptr{lws_context})::Cvoid
end

# typedef int ( * lws_reload_func ) ( void )
const lws_reload_func = Ptr{Cvoid}

function lws_context_deprecate(context, cb)
    @ccall libwebsockets.lws_context_deprecate(context::Ptr{lws_context}, cb::lws_reload_func)::Cvoid
end

function lws_context_is_deprecated(context)
    @ccall libwebsockets.lws_context_is_deprecated(context::Ptr{lws_context})::Cint
end

function lws_set_proxy(vhost, proxy)
    @ccall libwebsockets.lws_set_proxy(vhost::Ptr{lws_vhost}, proxy::Ptr{Cchar})::Cint
end

function lws_set_socks(vhost, socks)
    @ccall libwebsockets.lws_set_socks(vhost::Ptr{lws_vhost}, socks::Ptr{Cchar})::Cint
end

function lws_create_vhost(context, info)
    @ccall libwebsockets.lws_create_vhost(context::Ptr{lws_context}, info::Ptr{lws_context_creation_info})::Ptr{lws_vhost}
end

function lws_vhost_destroy(vh)
    @ccall libwebsockets.lws_vhost_destroy(vh::Ptr{lws_vhost})::Cvoid
end

function lwsws_get_config_globals(info, d, config_strings, len)
    @ccall libwebsockets.lwsws_get_config_globals(info::Ptr{lws_context_creation_info}, d::Ptr{Cchar}, config_strings::Ptr{Ptr{Cchar}}, len::Ptr{Cint})::Cint
end

function lwsws_get_config_vhosts(context, info, d, config_strings, len)
    @ccall libwebsockets.lwsws_get_config_vhosts(context::Ptr{lws_context}, info::Ptr{lws_context_creation_info}, d::Ptr{Cchar}, config_strings::Ptr{Ptr{Cchar}}, len::Ptr{Cint})::Cint
end

function lws_get_vhost(wsi)
    @ccall libwebsockets.lws_get_vhost(wsi::Ptr{lws})::Ptr{lws_vhost}
end

function lws_get_vhost_name(vhost)
    @ccall libwebsockets.lws_get_vhost_name(vhost::Ptr{lws_vhost})::Ptr{Cchar}
end

function lws_get_vhost_by_name(context, name)
    @ccall libwebsockets.lws_get_vhost_by_name(context::Ptr{lws_context}, name::Ptr{Cchar})::Ptr{lws_vhost}
end

function lws_get_vhost_port(vhost)
    @ccall libwebsockets.lws_get_vhost_port(vhost::Ptr{lws_vhost})::Cint
end

function lws_get_vhost_user(vhost)
    @ccall libwebsockets.lws_get_vhost_user(vhost::Ptr{lws_vhost})::Ptr{Cvoid}
end

function lws_get_vhost_iface(vhost)
    @ccall libwebsockets.lws_get_vhost_iface(vhost::Ptr{lws_vhost})::Ptr{Cchar}
end

function lws_json_dump_vhost(vh, buf, len)
    @ccall libwebsockets.lws_json_dump_vhost(vh::Ptr{lws_vhost}, buf::Ptr{Cchar}, len::Cint)::Cint
end

function lws_json_dump_context(context, buf, len, hide_vhosts)
    @ccall libwebsockets.lws_json_dump_context(context::Ptr{lws_context}, buf::Ptr{Cchar}, len::Cint, hide_vhosts::Cint)::Cint
end

function lws_vhost_user(vhost)
    @ccall libwebsockets.lws_vhost_user(vhost::Ptr{lws_vhost})::Ptr{Cvoid}
end

function lws_context_user(context)
    @ccall libwebsockets.lws_context_user(context::Ptr{lws_context})::Ptr{Cvoid}
end

function lws_vh_tag(vh)
    @ccall libwebsockets.lws_vh_tag(vh::Ptr{lws_vhost})::Ptr{Cchar}
end

function lws_context_is_being_destroyed(context)
    @ccall libwebsockets.lws_context_is_being_destroyed(context::Ptr{lws_context})::Cint
end

const lws_mount_protocols = UInt32
const LWSMPRO_HTTP = 0 % UInt32
const LWSMPRO_HTTPS = 1 % UInt32
const LWSMPRO_FILE = 2 % UInt32
const LWSMPRO_CGI = 3 % UInt32
const LWSMPRO_REDIR_HTTP = 4 % UInt32
const LWSMPRO_REDIR_HTTPS = 5 % UInt32
const LWSMPRO_CALLBACK = 6 % UInt32

const lws_authentication_mode = UInt32
const LWSAUTHM_DEFAULT = 0 % UInt32
const LWSAUTHM_BASIC_AUTH_CALLBACK = 268435456 % UInt32

struct lws_http_mount
    data::NTuple{88, UInt8}
end

function Base.getproperty(x::Ptr{lws_http_mount}, f::Symbol)
    f === :mount_next && return Ptr{Ptr{lws_http_mount}}(x + 0)
    f === :mountpoint && return Ptr{Ptr{Cchar}}(x + 8)
    f === :origin && return Ptr{Ptr{Cchar}}(x + 16)
    f === :def && return Ptr{Ptr{Cchar}}(x + 24)
    f === :protocol && return Ptr{Ptr{Cchar}}(x + 32)
    f === :cgienv && return Ptr{Ptr{lws_protocol_vhost_options}}(x + 40)
    f === :extra_mimetypes && return Ptr{Ptr{lws_protocol_vhost_options}}(x + 48)
    f === :interpret && return Ptr{Ptr{lws_protocol_vhost_options}}(x + 56)
    f === :cgi_timeout && return Ptr{Cint}(x + 64)
    f === :cache_max_age && return Ptr{Cint}(x + 68)
    f === :auth_mask && return Ptr{Cuint}(x + 72)
    f === :cache_reusable && return (Ptr{Cuint}(x + 76), 0, 1)
    f === :cache_revalidate && return (Ptr{Cuint}(x + 76), 1, 1)
    f === :cache_intermediaries && return (Ptr{Cuint}(x + 76), 2, 1)
    f === :origin_protocol && return Ptr{Cuchar}(x + 77)
    f === :mountpoint_len && return Ptr{Cuchar}(x + 78)
    f === :basic_auth_login_file && return Ptr{Ptr{Cchar}}(x + 80)
    return getfield(x, f)
end

function Base.getproperty(x::lws_http_mount, f::Symbol)
    r = Ref{lws_http_mount}(x)
    ptr = Base.unsafe_convert(Ptr{lws_http_mount}, r)
    fptr = getproperty(ptr, f)
    begin
        if fptr isa Ptr
            return GC.@preserve(r, unsafe_load(fptr))
        else
            (baseptr, offset, width) = fptr
            ty = eltype(baseptr)
            baseptr32 = convert(Ptr{UInt32}, baseptr)
            u64 = GC.@preserve(r, unsafe_load(baseptr32))
            if offset + width > 32
                u64 |= GC.@preserve(r, unsafe_load(baseptr32 + 4)) << 32
            end
            u64 = u64 >> offset & (1 << width - 1)
            return u64 % ty
        end
    end
end

function Base.setproperty!(x::Ptr{lws_http_mount}, f::Symbol, v)
    fptr = getproperty(x, f)
    if fptr isa Ptr
        unsafe_store!(getproperty(x, f), v)
    else
        (baseptr, offset, width) = fptr
        baseptr32 = convert(Ptr{UInt32}, baseptr)
        u64 = unsafe_load(baseptr32)
        straddle = offset + width > 32
        if straddle
            u64 |= unsafe_load(baseptr32 + 4) << 32
        end
        mask = 1 << width - 1
        u64 &= ~(mask << offset)
        u64 |= (unsigned(v) & mask) << offset
        unsafe_store!(baseptr32, u64 & typemax(UInt32))
        if straddle
            unsafe_store!(baseptr32 + 4, u64 >> 32)
        end
    end
end

const lws_conmon_interval_us_t = UInt32

const lws_conmon_pcol = UInt32
const LWSCONMON_PCOL_NONE = 0 % UInt32
const LWSCONMON_PCOL_HTTP = 1 % UInt32

const lws_conmon_pcol_t = lws_conmon_pcol

const lws_conmon_dns_disposition = UInt32
const LWSCONMON_DNS_NONE = 0 % UInt32
const LWSCONMON_DNS_OK = 1 % UInt32
const LWSCONMON_DNS_SERVER_UNREACHABLE = 2 % UInt32
const LWSCONMON_DNS_NO_RESULT = 3 % UInt32

const lws_conmon_dns_disposition_t = lws_conmon_dns_disposition

struct __JL_Ctag_67
    data::NTuple{4, UInt8}
end

function Base.getproperty(x::Ptr{__JL_Ctag_67}, f::Symbol)
    f === :http && return Ptr{__JL_Ctag_68}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_67, f::Symbol)
    r = Ref{__JL_Ctag_67}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_67}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_67}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

struct lws_conmon
    data::NTuple{56, UInt8}
end

function Base.getproperty(x::Ptr{lws_conmon}, f::Symbol)
    f === :peer46 && return Ptr{lws_sockaddr46}(x + 0)
    f === :protocol_specific && return Ptr{__JL_Ctag_67}(x + 16)
    f === :dns_results_copy && return Ptr{Ptr{addrinfo}}(x + 24)
    f === :ciu_dns && return Ptr{lws_conmon_interval_us_t}(x + 32)
    f === :ciu_sockconn && return Ptr{lws_conmon_interval_us_t}(x + 36)
    f === :ciu_tls && return Ptr{lws_conmon_interval_us_t}(x + 40)
    f === :ciu_txn_resp && return Ptr{lws_conmon_interval_us_t}(x + 44)
    f === :pcol && return Ptr{lws_conmon_pcol_t}(x + 48)
    f === :dns_disposition && return Ptr{lws_conmon_dns_disposition_t}(x + 52)
    return getfield(x, f)
end

function Base.getproperty(x::lws_conmon, f::Symbol)
    r = Ref{lws_conmon}(x)
    ptr = Base.unsafe_convert(Ptr{lws_conmon}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{lws_conmon}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

function lws_conmon_wsi_take(wsi, dest)
    @ccall libwebsockets.lws_conmon_wsi_take(wsi::Ptr{lws}, dest::Ptr{lws_conmon})::Cvoid
end

function lws_conmon_release(conmon)
    @ccall libwebsockets.lws_conmon_release(conmon::Ptr{lws_conmon})::Cvoid
end

const lws_client_connect_ssl_connection_flags = UInt32
const LCCSCF_USE_SSL = 1 % UInt32
const LCCSCF_ALLOW_SELFSIGNED = 2 % UInt32
const LCCSCF_SKIP_SERVER_CERT_HOSTNAME_CHECK = 4 % UInt32
const LCCSCF_ALLOW_EXPIRED = 8 % UInt32
const LCCSCF_ALLOW_INSECURE = 16 % UInt32
const LCCSCF_H2_QUIRK_NGHTTP2_END_STREAM = 32 % UInt32
const LCCSCF_H2_QUIRK_OVERFLOWS_TXCR = 64 % UInt32
const LCCSCF_H2_AUTH_BEARER = 128 % UInt32
const LCCSCF_H2_HEXIFY_AUTH_TOKEN = 256 % UInt32
const LCCSCF_H2_MANUAL_RXFLOW = 512 % UInt32
const LCCSCF_HTTP_MULTIPART_MIME = 1024 % UInt32
const LCCSCF_HTTP_X_WWW_FORM_URLENCODED = 2048 % UInt32
const LCCSCF_HTTP_NO_FOLLOW_REDIRECT = 4096 % UInt32
const LCCSCF_PIPELINE = 65536 % UInt32
const LCCSCF_MUXABLE_STREAM = 131072 % UInt32
const LCCSCF_H2_PRIOR_KNOWLEDGE = 262144 % UInt32
const LCCSCF_WAKE_SUSPEND__VALIDITY = 524288 % UInt32
const LCCSCF_PRIORITIZE_READS = 1048576 % UInt32
const LCCSCF_SECSTREAM_CLIENT = 2097152 % UInt32
const LCCSCF_SECSTREAM_PROXY_LINK = 4194304 % UInt32
const LCCSCF_SECSTREAM_PROXY_ONWARD = 8388608 % UInt32
const LCCSCF_IP_LOW_LATENCY = 16777216 % UInt32
const LCCSCF_IP_HIGH_THROUGHPUT = 33554432 % UInt32
const LCCSCF_IP_HIGH_RELIABILITY = 67108864 % UInt32
const LCCSCF_IP_LOW_COST = 134217728 % UInt32
const LCCSCF_CONMON = 268435456 % UInt32
const LCCSCF_ACCEPT_TLS_DOWNGRADE_REDIRECTS = 536870912 % UInt32
const LCCSCF_CACHE_COOKIES = 1073741824 % UInt32

mutable struct lws_sequencer end

struct lws_client_connect_info
    context::Ptr{lws_context}
    address::Ptr{Cchar}
    port::Cint
    ssl_connection::Cint
    path::Ptr{Cchar}
    host::Ptr{Cchar}
    origin::Ptr{Cchar}
    protocol::Ptr{Cchar}
    ietf_version_or_minus_one::Cint
    userdata::Ptr{Cvoid}
    client_exts::Ptr{Cvoid}
    method::Ptr{Cchar}
    parent_wsi::Ptr{lws}
    uri_replace_from::Ptr{Cchar}
    uri_replace_to::Ptr{Cchar}
    vhost::Ptr{lws_vhost}
    pwsi::Ptr{Ptr{lws}}
    iface::Ptr{Cchar}
    local_protocol_name::Ptr{Cchar}
    alpn::Ptr{Cchar}
    seq::Ptr{lws_sequencer}
    opaque_user_data::Ptr{Cvoid}
    retry_and_idle_policy::Ptr{lws_retry_bo_t}
    manual_initial_tx_credit::Cint
    sys_tls_client_cert::UInt8
    priority::UInt8
    mqtt_cp::Ptr{Cvoid}
    fi_wsi_name::Ptr{Cchar}
    keep_warm_secs::UInt16
    log_cx::Ptr{lws_log_cx_t}
    _unused::NTuple{4, Ptr{Cvoid}}
end

function lws_client_connect_via_info(ccinfo)
    @ccall libwebsockets.lws_client_connect_via_info(ccinfo::Ptr{lws_client_connect_info})::Ptr{lws}
end

function lws_init_vhost_client_ssl(info, vhost)
    @ccall libwebsockets.lws_init_vhost_client_ssl(info::Ptr{lws_context_creation_info}, vhost::Ptr{lws_vhost})::Cint
end

function lws_http_client_read(wsi, buf, len)
    @ccall libwebsockets.lws_http_client_read(wsi::Ptr{lws}, buf::Ptr{Ptr{Cchar}}, len::Ptr{Cint})::Cint
end

function lws_http_client_http_response(wsi)
    @ccall libwebsockets.lws_http_client_http_response(wsi::Ptr{lws})::Cuint
end

function lws_tls_client_vhost_extra_cert_mem(vh, der, der_len)
    @ccall libwebsockets.lws_tls_client_vhost_extra_cert_mem(vh::Ptr{lws_vhost}, der::Ptr{UInt8}, der_len::Csize_t)::Cint
end

function lws_client_http_body_pending(wsi, something_left_to_send)
    @ccall libwebsockets.lws_client_http_body_pending(wsi::Ptr{lws}, something_left_to_send::Cint)::Cvoid
end

function lws_client_http_multipart(wsi, name, filename, content_type, p, _end)
    @ccall libwebsockets.lws_client_http_multipart(wsi::Ptr{lws}, name::Ptr{Cchar}, filename::Ptr{Cchar}, content_type::Ptr{Cchar}, p::Ptr{Ptr{Cchar}}, _end::Ptr{Cchar})::Cint
end

function lws_http_basic_auth_gen(user, pw, buf, len)
    @ccall libwebsockets.lws_http_basic_auth_gen(user::Ptr{Cchar}, pw::Ptr{Cchar}, buf::Ptr{Cchar}, len::Csize_t)::Cint
end

function lws_tls_session_is_reused(wsi)
    @ccall libwebsockets.lws_tls_session_is_reused(wsi::Ptr{lws})::Cint
end

function lws_get_mimetype(file, m)
    @ccall libwebsockets.lws_get_mimetype(file::Ptr{Cchar}, m::Ptr{lws_http_mount})::Ptr{Cchar}
end

function lws_serve_http_file(wsi, file, content_type, other_headers, other_headers_len)
    @ccall libwebsockets.lws_serve_http_file(wsi::Ptr{lws}, file::Ptr{Cchar}, content_type::Ptr{Cchar}, other_headers::Ptr{Cchar}, other_headers_len::Cint)::Cint
end

function lws_serve_http_file_fragment(wsi)
    @ccall libwebsockets.lws_serve_http_file_fragment(wsi::Ptr{lws})::Cint
end

const http_status = UInt32
const HTTP_STATUS_CONTINUE = 100 % UInt32
const HTTP_STATUS_OK = 200 % UInt32
const HTTP_STATUS_NO_CONTENT = 204 % UInt32
const HTTP_STATUS_PARTIAL_CONTENT = 206 % UInt32
const HTTP_STATUS_MOVED_PERMANENTLY = 301 % UInt32
const HTTP_STATUS_FOUND = 302 % UInt32
const HTTP_STATUS_SEE_OTHER = 303 % UInt32
const HTTP_STATUS_NOT_MODIFIED = 304 % UInt32
const HTTP_STATUS_BAD_REQUEST = 400 % UInt32
const HTTP_STATUS_UNAUTHORIZED = 401 % UInt32
const HTTP_STATUS_PAYMENT_REQUIRED = 402 % UInt32
const HTTP_STATUS_FORBIDDEN = 403 % UInt32
const HTTP_STATUS_NOT_FOUND = 404 % UInt32
const HTTP_STATUS_METHOD_NOT_ALLOWED = 405 % UInt32
const HTTP_STATUS_NOT_ACCEPTABLE = 406 % UInt32
const HTTP_STATUS_PROXY_AUTH_REQUIRED = 407 % UInt32
const HTTP_STATUS_REQUEST_TIMEOUT = 408 % UInt32
const HTTP_STATUS_CONFLICT = 409 % UInt32
const HTTP_STATUS_GONE = 410 % UInt32
const HTTP_STATUS_LENGTH_REQUIRED = 411 % UInt32
const HTTP_STATUS_PRECONDITION_FAILED = 412 % UInt32
const HTTP_STATUS_REQ_ENTITY_TOO_LARGE = 413 % UInt32
const HTTP_STATUS_REQ_URI_TOO_LONG = 414 % UInt32
const HTTP_STATUS_UNSUPPORTED_MEDIA_TYPE = 415 % UInt32
const HTTP_STATUS_REQ_RANGE_NOT_SATISFIABLE = 416 % UInt32
const HTTP_STATUS_EXPECTATION_FAILED = 417 % UInt32
const HTTP_STATUS_INTERNAL_SERVER_ERROR = 500 % UInt32
const HTTP_STATUS_NOT_IMPLEMENTED = 501 % UInt32
const HTTP_STATUS_BAD_GATEWAY = 502 % UInt32
const HTTP_STATUS_SERVICE_UNAVAILABLE = 503 % UInt32
const HTTP_STATUS_GATEWAY_TIMEOUT = 504 % UInt32
const HTTP_STATUS_HTTP_VERSION_NOT_SUPPORTED = 505 % UInt32

struct lws_process_html_args
    p::Ptr{Cchar}
    len::Cint
    max_len::Cint
    final::Cint
    chunked::Cint
end

# typedef const char * ( * lws_process_html_state_cb ) ( void * data , int index )
const lws_process_html_state_cb = Ptr{Cvoid}

struct lws_process_html_state
    start::Ptr{Cchar}
    swallow::NTuple{16, Cchar}
    pos::Cint
    data::Ptr{Cvoid}
    vars::Ptr{Ptr{Cchar}}
    count_vars::Cint
    replace::lws_process_html_state_cb
end

function lws_chunked_html_process(args, s)
    @ccall libwebsockets.lws_chunked_html_process(args::Ptr{lws_process_html_args}, s::Ptr{lws_process_html_state})::Cint
end

struct lws_tokens
    token::Ptr{Cuchar}
    len::Cint
end

const lws_token_indexes = UInt32
const WSI_TOKEN_GET_URI = 0 % UInt32
const WSI_TOKEN_POST_URI = 1 % UInt32
const WSI_TOKEN_OPTIONS_URI = 2 % UInt32
const WSI_TOKEN_HOST = 3 % UInt32
const WSI_TOKEN_CONNECTION = 4 % UInt32
const WSI_TOKEN_UPGRADE = 5 % UInt32
const WSI_TOKEN_ORIGIN = 6 % UInt32
const WSI_TOKEN_DRAFT = 7 % UInt32
const WSI_TOKEN_CHALLENGE = 8 % UInt32
const WSI_TOKEN_EXTENSIONS = 9 % UInt32
const WSI_TOKEN_KEY1 = 10 % UInt32
const WSI_TOKEN_KEY2 = 11 % UInt32
const WSI_TOKEN_PROTOCOL = 12 % UInt32
const WSI_TOKEN_ACCEPT = 13 % UInt32
const WSI_TOKEN_NONCE = 14 % UInt32
const WSI_TOKEN_HTTP = 15 % UInt32
const WSI_TOKEN_HTTP2_SETTINGS = 16 % UInt32
const WSI_TOKEN_HTTP_ACCEPT = 17 % UInt32
const WSI_TOKEN_HTTP_AC_REQUEST_HEADERS = 18 % UInt32
const WSI_TOKEN_HTTP_IF_MODIFIED_SINCE = 19 % UInt32
const WSI_TOKEN_HTTP_IF_NONE_MATCH = 20 % UInt32
const WSI_TOKEN_HTTP_ACCEPT_ENCODING = 21 % UInt32
const WSI_TOKEN_HTTP_ACCEPT_LANGUAGE = 22 % UInt32
const WSI_TOKEN_HTTP_PRAGMA = 23 % UInt32
const WSI_TOKEN_HTTP_CACHE_CONTROL = 24 % UInt32
const WSI_TOKEN_HTTP_AUTHORIZATION = 25 % UInt32
const WSI_TOKEN_HTTP_COOKIE = 26 % UInt32
const WSI_TOKEN_HTTP_CONTENT_LENGTH = 27 % UInt32
const WSI_TOKEN_HTTP_CONTENT_TYPE = 28 % UInt32
const WSI_TOKEN_HTTP_DATE = 29 % UInt32
const WSI_TOKEN_HTTP_RANGE = 30 % UInt32
const WSI_TOKEN_HTTP_REFERER = 31 % UInt32
const WSI_TOKEN_KEY = 32 % UInt32
const WSI_TOKEN_VERSION = 33 % UInt32
const WSI_TOKEN_SWORIGIN = 34 % UInt32
const WSI_TOKEN_HTTP_COLON_AUTHORITY = 35 % UInt32
const WSI_TOKEN_HTTP_COLON_METHOD = 36 % UInt32
const WSI_TOKEN_HTTP_COLON_PATH = 37 % UInt32
const WSI_TOKEN_HTTP_COLON_SCHEME = 38 % UInt32
const WSI_TOKEN_HTTP_COLON_STATUS = 39 % UInt32
const WSI_TOKEN_HTTP_ACCEPT_CHARSET = 40 % UInt32
const WSI_TOKEN_HTTP_ACCEPT_RANGES = 41 % UInt32
const WSI_TOKEN_HTTP_ACCESS_CONTROL_ALLOW_ORIGIN = 42 % UInt32
const WSI_TOKEN_HTTP_AGE = 43 % UInt32
const WSI_TOKEN_HTTP_ALLOW = 44 % UInt32
const WSI_TOKEN_HTTP_CONTENT_DISPOSITION = 45 % UInt32
const WSI_TOKEN_HTTP_CONTENT_ENCODING = 46 % UInt32
const WSI_TOKEN_HTTP_CONTENT_LANGUAGE = 47 % UInt32
const WSI_TOKEN_HTTP_CONTENT_LOCATION = 48 % UInt32
const WSI_TOKEN_HTTP_CONTENT_RANGE = 49 % UInt32
const WSI_TOKEN_HTTP_ETAG = 50 % UInt32
const WSI_TOKEN_HTTP_EXPECT = 51 % UInt32
const WSI_TOKEN_HTTP_EXPIRES = 52 % UInt32
const WSI_TOKEN_HTTP_FROM = 53 % UInt32
const WSI_TOKEN_HTTP_IF_MATCH = 54 % UInt32
const WSI_TOKEN_HTTP_IF_RANGE = 55 % UInt32
const WSI_TOKEN_HTTP_IF_UNMODIFIED_SINCE = 56 % UInt32
const WSI_TOKEN_HTTP_LAST_MODIFIED = 57 % UInt32
const WSI_TOKEN_HTTP_LINK = 58 % UInt32
const WSI_TOKEN_HTTP_LOCATION = 59 % UInt32
const WSI_TOKEN_HTTP_MAX_FORWARDS = 60 % UInt32
const WSI_TOKEN_HTTP_PROXY_AUTHENTICATE = 61 % UInt32
const WSI_TOKEN_HTTP_PROXY_AUTHORIZATION = 62 % UInt32
const WSI_TOKEN_HTTP_REFRESH = 63 % UInt32
const WSI_TOKEN_HTTP_RETRY_AFTER = 64 % UInt32
const WSI_TOKEN_HTTP_SERVER = 65 % UInt32
const WSI_TOKEN_HTTP_SET_COOKIE = 66 % UInt32
const WSI_TOKEN_HTTP_STRICT_TRANSPORT_SECURITY = 67 % UInt32
const WSI_TOKEN_HTTP_TRANSFER_ENCODING = 68 % UInt32
const WSI_TOKEN_HTTP_USER_AGENT = 69 % UInt32
const WSI_TOKEN_HTTP_VARY = 70 % UInt32
const WSI_TOKEN_HTTP_VIA = 71 % UInt32
const WSI_TOKEN_HTTP_WWW_AUTHENTICATE = 72 % UInt32
const WSI_TOKEN_PATCH_URI = 73 % UInt32
const WSI_TOKEN_PUT_URI = 74 % UInt32
const WSI_TOKEN_DELETE_URI = 75 % UInt32
const WSI_TOKEN_HTTP_URI_ARGS = 76 % UInt32
const WSI_TOKEN_PROXY = 77 % UInt32
const WSI_TOKEN_HTTP_X_REAL_IP = 78 % UInt32
const WSI_TOKEN_HTTP1_0 = 79 % UInt32
const WSI_TOKEN_X_FORWARDED_FOR = 80 % UInt32
const WSI_TOKEN_CONNECT = 81 % UInt32
const WSI_TOKEN_HEAD_URI = 82 % UInt32
const WSI_TOKEN_TE = 83 % UInt32
const WSI_TOKEN_REPLAY_NONCE = 84 % UInt32
const WSI_TOKEN_COLON_PROTOCOL = 85 % UInt32
const WSI_TOKEN_X_AUTH_TOKEN = 86 % UInt32
const WSI_TOKEN_DSS_SIGNATURE = 87 % UInt32
const _WSI_TOKEN_CLIENT_SENT_PROTOCOLS = 88 % UInt32
const _WSI_TOKEN_CLIENT_PEER_ADDRESS = 89 % UInt32
const _WSI_TOKEN_CLIENT_URI = 90 % UInt32
const _WSI_TOKEN_CLIENT_HOST = 91 % UInt32
const _WSI_TOKEN_CLIENT_ORIGIN = 92 % UInt32
const _WSI_TOKEN_CLIENT_METHOD = 93 % UInt32
const _WSI_TOKEN_CLIENT_IFACE = 94 % UInt32
const _WSI_TOKEN_CLIENT_ALPN = 95 % UInt32
const WSI_TOKEN_COUNT = 96 % UInt32
const WSI_TOKEN_NAME_PART = 97 % UInt32
const WSI_TOKEN_UNKNOWN_VALUE_PART = 98 % UInt32
const WSI_TOKEN_SKIPPING = 99 % UInt32
const WSI_TOKEN_SKIPPING_SAW_CR = 100 % UInt32
const WSI_PARSING_COMPLETE = 101 % UInt32
const WSI_INIT_TOKEN_MUXURL = 102 % UInt32

struct lws_token_limits
    token_limit::NTuple{96, Cushort}
end

const lws_h2_settings = UInt32
const H2SET_HEADER_TABLE_SIZE = 1 % UInt32
const H2SET_ENABLE_PUSH = 2 % UInt32
const H2SET_MAX_CONCURRENT_STREAMS = 3 % UInt32
const H2SET_INITIAL_WINDOW_SIZE = 4 % UInt32
const H2SET_MAX_FRAME_SIZE = 5 % UInt32
const H2SET_MAX_HEADER_LIST_SIZE = 6 % UInt32
const H2SET_RESERVED7 = 7 % UInt32
const H2SET_ENABLE_CONNECT_PROTOCOL = 8 % UInt32
const H2SET_COUNT = 9 % UInt32

function lws_token_to_string(token)
    @ccall libwebsockets.lws_token_to_string(token::lws_token_indexes)::Ptr{Cuchar}
end

function lws_hdr_total_length(wsi, h)
    @ccall libwebsockets.lws_hdr_total_length(wsi::Ptr{lws}, h::lws_token_indexes)::Cint
end

function lws_hdr_fragment_length(wsi, h, frag_idx)
    @ccall libwebsockets.lws_hdr_fragment_length(wsi::Ptr{lws}, h::lws_token_indexes, frag_idx::Cint)::Cint
end

function lws_hdr_copy(wsi, dest, len, h)
    @ccall libwebsockets.lws_hdr_copy(wsi::Ptr{lws}, dest::Ptr{Cchar}, len::Cint, h::lws_token_indexes)::Cint
end

function lws_hdr_copy_fragment(wsi, dest, len, h, frag_idx)
    @ccall libwebsockets.lws_hdr_copy_fragment(wsi::Ptr{lws}, dest::Ptr{Cchar}, len::Cint, h::lws_token_indexes, frag_idx::Cint)::Cint
end

function lws_hdr_custom_length(wsi, name, nlen)
    @ccall libwebsockets.lws_hdr_custom_length(wsi::Ptr{lws}, name::Ptr{Cchar}, nlen::Cint)::Cint
end

function lws_hdr_custom_copy(wsi, dst, len, name, nlen)
    @ccall libwebsockets.lws_hdr_custom_copy(wsi::Ptr{lws}, dst::Ptr{Cchar}, len::Cint, name::Ptr{Cchar}, nlen::Cint)::Cint
end

# typedef void ( * lws_hdr_custom_fe_cb_t ) ( const char * name , int nlen , void * opaque )
const lws_hdr_custom_fe_cb_t = Ptr{Cvoid}

function lws_hdr_custom_name_foreach(wsi, cb, opaque)
    @ccall libwebsockets.lws_hdr_custom_name_foreach(wsi::Ptr{lws}, cb::lws_hdr_custom_fe_cb_t, opaque::Ptr{Cvoid})::Cint
end

function lws_get_urlarg_by_name_safe(wsi, name, buf, len)
    @ccall libwebsockets.lws_get_urlarg_by_name_safe(wsi::Ptr{lws}, name::Ptr{Cchar}, buf::Ptr{Cchar}, len::Cint)::Cint
end

function lws_get_urlarg_by_name(wsi, name, buf, len)
    @ccall libwebsockets.lws_get_urlarg_by_name(wsi::Ptr{lws}, name::Ptr{Cchar}, buf::Ptr{Cchar}, len::Cint)::Ptr{Cchar}
end

function lws_add_http_header_status(wsi, code, p, _end)
    @ccall libwebsockets.lws_add_http_header_status(wsi::Ptr{lws}, code::Cuint, p::Ptr{Ptr{Cuchar}}, _end::Ptr{Cuchar})::Cint
end

function lws_add_http_header_by_name(wsi, name, value, length, p, _end)
    @ccall libwebsockets.lws_add_http_header_by_name(wsi::Ptr{lws}, name::Ptr{Cuchar}, value::Ptr{Cuchar}, length::Cint, p::Ptr{Ptr{Cuchar}}, _end::Ptr{Cuchar})::Cint
end

function lws_add_http_header_by_token(wsi, token, value, length, p, _end)
    @ccall libwebsockets.lws_add_http_header_by_token(wsi::Ptr{lws}, token::lws_token_indexes, value::Ptr{Cuchar}, length::Cint, p::Ptr{Ptr{Cuchar}}, _end::Ptr{Cuchar})::Cint
end

function lws_add_http_header_content_length(wsi, content_length, p, _end)
    @ccall libwebsockets.lws_add_http_header_content_length(wsi::Ptr{lws}, content_length::lws_filepos_t, p::Ptr{Ptr{Cuchar}}, _end::Ptr{Cuchar})::Cint
end

function lws_finalize_http_header(wsi, p, _end)
    @ccall libwebsockets.lws_finalize_http_header(wsi::Ptr{lws}, p::Ptr{Ptr{Cuchar}}, _end::Ptr{Cuchar})::Cint
end

function lws_finalize_write_http_header(wsi, start, p, _end)
    @ccall libwebsockets.lws_finalize_write_http_header(wsi::Ptr{lws}, start::Ptr{Cuchar}, p::Ptr{Ptr{Cuchar}}, _end::Ptr{Cuchar})::Cint
end

function lws_add_http_common_headers(wsi, code, content_type, content_len, p, _end)
    @ccall libwebsockets.lws_add_http_common_headers(wsi::Ptr{lws}, code::Cuint, content_type::Ptr{Cchar}, content_len::lws_filepos_t, p::Ptr{Ptr{Cuchar}}, _end::Ptr{Cuchar})::Cint
end

const __JL_Ctag_19 = UInt32
const LWSHUMETH_GET = 0 % UInt32
const LWSHUMETH_POST = 1 % UInt32
const LWSHUMETH_OPTIONS = 2 % UInt32
const LWSHUMETH_PUT = 3 % UInt32
const LWSHUMETH_PATCH = 4 % UInt32
const LWSHUMETH_DELETE = 5 % UInt32
const LWSHUMETH_CONNECT = 6 % UInt32
const LWSHUMETH_HEAD = 7 % UInt32
const LWSHUMETH_COLON_PATH = 8 % UInt32

function lws_http_get_uri_and_method(wsi, puri_ptr, puri_len)
    @ccall libwebsockets.lws_http_get_uri_and_method(wsi::Ptr{lws}, puri_ptr::Ptr{Ptr{Cchar}}, puri_len::Ptr{Cint})::Cint
end

function lws_urlencode(escaped, string, len)
    @ccall libwebsockets.lws_urlencode(escaped::Ptr{Cchar}, string::Ptr{Cchar}, len::Cint)::Ptr{Cchar}
end

function lws_urldecode(string, escaped, len)
    @ccall libwebsockets.lws_urldecode(string::Ptr{Cchar}, escaped::Ptr{Cchar}, len::Cint)::Cint
end

function lws_http_date_render_from_unix(buf, len, t)
    @ccall libwebsockets.lws_http_date_render_from_unix(buf::Ptr{Cchar}, len::Csize_t, t::Ptr{time_t})::Cint
end

function lws_http_date_parse_unix(b, len, t)
    @ccall libwebsockets.lws_http_date_parse_unix(b::Ptr{Cchar}, len::Csize_t, t::Ptr{time_t})::Cint
end

function lws_http_check_retry_after(wsi, us_interval_in_out)
    @ccall libwebsockets.lws_http_check_retry_after(wsi::Ptr{lws}, us_interval_in_out::Ptr{lws_usec_t})::Cint
end

function lws_return_http_status(wsi, code, html_body)
    @ccall libwebsockets.lws_return_http_status(wsi::Ptr{lws}, code::Cuint, html_body::Ptr{Cchar})::Cint
end

function lws_http_redirect(wsi, code, loc, len, p, _end)
    @ccall libwebsockets.lws_http_redirect(wsi::Ptr{lws}, code::Cint, loc::Ptr{Cuchar}, len::Cint, p::Ptr{Ptr{Cuchar}}, _end::Ptr{Cuchar})::Cint
end

function lws_http_transaction_completed(wsi)
    @ccall libwebsockets.lws_http_transaction_completed(wsi::Ptr{lws})::Cint
end

function lws_http_headers_detach(wsi)
    @ccall libwebsockets.lws_http_headers_detach(wsi::Ptr{lws})::Cint
end

function lws_http_mark_sse(wsi)
    @ccall libwebsockets.lws_http_mark_sse(wsi::Ptr{lws})::Cint
end

function lws_h2_client_stream_long_poll_rxonly(wsi)
    @ccall libwebsockets.lws_h2_client_stream_long_poll_rxonly(wsi::Ptr{lws})::Cint
end

function lws_http_compression_apply(wsi, name, p, _end, decomp)
    @ccall libwebsockets.lws_http_compression_apply(wsi::Ptr{lws}, name::Ptr{Cchar}, p::Ptr{Ptr{Cuchar}}, _end::Ptr{Cuchar}, decomp::Cchar)::Cint
end

function lws_http_is_redirected_to_get(wsi)
    @ccall libwebsockets.lws_http_is_redirected_to_get(wsi::Ptr{lws})::Cint
end

function lws_http_cookie_get(wsi, name, buf, max)
    @ccall libwebsockets.lws_http_cookie_get(wsi::Ptr{lws}, name::Ptr{Cchar}, buf::Ptr{Cchar}, max::Ptr{Csize_t})::Cint
end

function lws_h2_update_peer_txcredit(wsi, sid, bump)
    @ccall libwebsockets.lws_h2_update_peer_txcredit(wsi::Ptr{lws}, sid::Cuint, bump::Cint)::Cint
end

function lws_h2_get_peer_txcredit_estimate(wsi)
    @ccall libwebsockets.lws_h2_get_peer_txcredit_estimate(wsi::Ptr{lws})::Cint
end

const lws_spa_fileupload_states = UInt32
const LWS_UFS_CONTENT = 0 % UInt32
const LWS_UFS_FINAL_CONTENT = 1 % UInt32
const LWS_UFS_OPEN = 2 % UInt32
const LWS_UFS_CLOSE = 3 % UInt32

# typedef int ( * lws_spa_fileupload_cb ) ( void * data , const char * name , const char * filename , char * buf , int len , enum lws_spa_fileupload_states state )
const lws_spa_fileupload_cb = Ptr{Cvoid}

mutable struct lws_spa end

function lws_spa_create(wsi, param_names, count_params, max_storage, opt_cb, opt_data)
    @ccall libwebsockets.lws_spa_create(wsi::Ptr{lws}, param_names::Ptr{Ptr{Cchar}}, count_params::Cint, max_storage::Cint, opt_cb::lws_spa_fileupload_cb, opt_data::Ptr{Cvoid})::Ptr{lws_spa}
end

struct lws_spa_create_info
    param_names::Ptr{Ptr{Cchar}}
    count_params::Cint
    max_storage::Cint
    opt_cb::lws_spa_fileupload_cb
    opt_data::Ptr{Cvoid}
    param_names_stride::Csize_t
    ac::Ptr{Ptr{lwsac}}
    ac_chunk_size::Csize_t
end

const lws_spa_create_info_t = lws_spa_create_info

function lws_spa_create_via_info(wsi, info)
    @ccall libwebsockets.lws_spa_create_via_info(wsi::Ptr{lws}, info::Ptr{lws_spa_create_info_t})::Ptr{lws_spa}
end

function lws_spa_process(spa, in, len)
    @ccall libwebsockets.lws_spa_process(spa::Ptr{lws_spa}, in::Ptr{Cchar}, len::Cint)::Cint
end

function lws_spa_finalize(spa)
    @ccall libwebsockets.lws_spa_finalize(spa::Ptr{lws_spa})::Cint
end

function lws_spa_get_length(spa, n)
    @ccall libwebsockets.lws_spa_get_length(spa::Ptr{lws_spa}, n::Cint)::Cint
end

function lws_spa_get_string(spa, n)
    @ccall libwebsockets.lws_spa_get_string(spa::Ptr{lws_spa}, n::Cint)::Ptr{Cchar}
end

function lws_spa_destroy(spa)
    @ccall libwebsockets.lws_spa_destroy(spa::Ptr{lws_spa})::Cint
end

function lws_sql_purify(escaped, string, len)
    @ccall libwebsockets.lws_sql_purify(escaped::Ptr{Cchar}, string::Ptr{Cchar}, len::Csize_t)::Ptr{Cchar}
end

function lws_sql_purify_len(p)
    @ccall libwebsockets.lws_sql_purify_len(p::Ptr{Cchar})::Cint
end

function lws_json_purify(escaped, string, len, in_used)
    @ccall libwebsockets.lws_json_purify(escaped::Ptr{Cchar}, string::Ptr{Cchar}, len::Cint, in_used::Ptr{Cint})::Ptr{Cchar}
end

function lws_json_purify_len(string)
    @ccall libwebsockets.lws_json_purify_len(string::Ptr{Cchar})::Cint
end

function lws_filename_purify_inplace(filename)
    @ccall libwebsockets.lws_filename_purify_inplace(filename::Ptr{Cchar})::Cvoid
end

function lws_plat_write_cert(vhost, is_key, fd, buf, len)
    @ccall libwebsockets.lws_plat_write_cert(vhost::Ptr{lws_vhost}, is_key::Cint, fd::Cint, buf::Ptr{Cvoid}, len::Csize_t)::Cint
end

function lws_plat_write_file(filename, buf, len)
    @ccall libwebsockets.lws_plat_write_file(filename::Ptr{Cchar}, buf::Ptr{Cvoid}, len::Csize_t)::Cint
end

function lws_plat_read_file(filename, buf, len)
    @ccall libwebsockets.lws_plat_read_file(filename::Ptr{Cchar}, buf::Ptr{Cvoid}, len::Csize_t)::Cint
end

function lws_plat_recommended_rsa_bits()
    @ccall libwebsockets.lws_plat_recommended_rsa_bits()::Cint
end

mutable struct lws_buflist end

function lws_buflist_append_segment(head, buf, len)
    @ccall libwebsockets.lws_buflist_append_segment(head::Ptr{Ptr{lws_buflist}}, buf::Ptr{UInt8}, len::Csize_t)::Cint
end

function lws_buflist_next_segment_len(head, buf)
    @ccall libwebsockets.lws_buflist_next_segment_len(head::Ptr{Ptr{lws_buflist}}, buf::Ptr{Ptr{UInt8}})::Csize_t
end

function lws_buflist_use_segment(head, len)
    @ccall libwebsockets.lws_buflist_use_segment(head::Ptr{Ptr{lws_buflist}}, len::Csize_t)::Csize_t
end

function lws_buflist_total_len(head)
    @ccall libwebsockets.lws_buflist_total_len(head::Ptr{Ptr{lws_buflist}})::Csize_t
end

function lws_buflist_linear_copy(head, ofs, buf, len)
    @ccall libwebsockets.lws_buflist_linear_copy(head::Ptr{Ptr{lws_buflist}}, ofs::Csize_t, buf::Ptr{UInt8}, len::Csize_t)::Cint
end

function lws_buflist_linear_use(head, buf, len)
    @ccall libwebsockets.lws_buflist_linear_use(head::Ptr{Ptr{lws_buflist}}, buf::Ptr{UInt8}, len::Csize_t)::Cint
end

function lws_buflist_fragment_use(head, buf, len, frag_first, frag_fin)
    @ccall libwebsockets.lws_buflist_fragment_use(head::Ptr{Ptr{lws_buflist}}, buf::Ptr{UInt8}, len::Csize_t, frag_first::Ptr{Cchar}, frag_fin::Ptr{Cchar})::Cint
end

function lws_buflist_destroy_all_segments(head)
    @ccall libwebsockets.lws_buflist_destroy_all_segments(head::Ptr{Ptr{lws_buflist}})::Cvoid
end

function lws_buflist_describe(head, id, reason)
    @ccall libwebsockets.lws_buflist_describe(head::Ptr{Ptr{lws_buflist}}, id::Ptr{Cvoid}, reason::Ptr{Cchar})::Cvoid
end

# automatic type deduction for variadic arguments may not be what you want, please use with caution
@generated function lws_snprintf(str, size, format, va_list...)
        :(@ccall(libwebsockets.lws_snprintf(str::Ptr{Cchar}, size::Csize_t, format::Ptr{Cchar}; $(to_c_type_pairs(va_list)...))::Cint))
    end

function lws_nstrstr(buf, len, name, nl)
    @ccall libwebsockets.lws_nstrstr(buf::Ptr{Cchar}, len::Csize_t, name::Ptr{Cchar}, nl::Csize_t)::Ptr{Cchar}
end

function lws_json_simple_find(buf, len, name, alen)
    @ccall libwebsockets.lws_json_simple_find(buf::Ptr{Cchar}, len::Csize_t, name::Ptr{Cchar}, alen::Ptr{Csize_t})::Ptr{Cchar}
end

function lws_json_simple_strcmp(buf, len, name, comp)
    @ccall libwebsockets.lws_json_simple_strcmp(buf::Ptr{Cchar}, len::Csize_t, name::Ptr{Cchar}, comp::Ptr{Cchar})::Cint
end

function lws_hex_to_byte_array(h, dest, max)
    @ccall libwebsockets.lws_hex_to_byte_array(h::Ptr{Cchar}, dest::Ptr{UInt8}, max::Cint)::Cint
end

function lws_hex_from_byte_array(src, slen, dest, len)
    @ccall libwebsockets.lws_hex_from_byte_array(src::Ptr{UInt8}, slen::Csize_t, dest::Ptr{Cchar}, len::Csize_t)::Cvoid
end

function lws_hex_random(context, dest, len)
    @ccall libwebsockets.lws_hex_random(context::Ptr{lws_context}, dest::Ptr{Cchar}, len::Csize_t)::Cint
end

function lws_timingsafe_bcmp(a, b, len)
    @ccall libwebsockets.lws_timingsafe_bcmp(a::Ptr{Cvoid}, b::Ptr{Cvoid}, len::UInt32)::Cint
end

function lws_get_random(context, buf, len)
    @ccall libwebsockets.lws_get_random(context::Ptr{lws_context}, buf::Ptr{Cvoid}, len::Csize_t)::Csize_t
end

function lws_daemonize(_lock_path)
    @ccall libwebsockets.lws_daemonize(_lock_path::Ptr{Cchar})::Cint
end

function lws_get_library_version()
    @ccall libwebsockets.lws_get_library_version()::Ptr{Cchar}
end

function lws_wsi_user(wsi)
    @ccall libwebsockets.lws_wsi_user(wsi::Ptr{lws})::Ptr{Cvoid}
end

function lws_wsi_tsi(wsi)
    @ccall libwebsockets.lws_wsi_tsi(wsi::Ptr{lws})::Cint
end

function lws_set_wsi_user(wsi, user)
    @ccall libwebsockets.lws_set_wsi_user(wsi::Ptr{lws}, user::Ptr{Cvoid})::Cvoid
end

function lws_parse_uri(p, prot, ads, port, path)
    @ccall libwebsockets.lws_parse_uri(p::Ptr{Cchar}, prot::Ptr{Ptr{Cchar}}, ads::Ptr{Ptr{Cchar}}, port::Ptr{Cint}, path::Ptr{Ptr{Cchar}})::Cint
end

function lws_cmdline_option(argc, argv, val)
    @ccall libwebsockets.lws_cmdline_option(argc::Cint, argv::Ptr{Ptr{Cchar}}, val::Ptr{Cchar})::Ptr{Cchar}
end

function lws_cmdline_option_handle_builtin(argc, argv, info)
    @ccall libwebsockets.lws_cmdline_option_handle_builtin(argc::Cint, argv::Ptr{Ptr{Cchar}}, info::Ptr{lws_context_creation_info})::Cvoid
end

function lws_now_secs()
    @ccall libwebsockets.lws_now_secs()::Culong
end

function lws_get_context(wsi)
    @ccall libwebsockets.lws_get_context(wsi::Ptr{lws})::Ptr{lws_context}
end

function lws_get_vhost_listen_port(vhost)
    @ccall libwebsockets.lws_get_vhost_listen_port(vhost::Ptr{lws_vhost})::Cint
end

function lws_get_count_threads(context)
    @ccall libwebsockets.lws_get_count_threads(context::Ptr{lws_context})::Cint
end

function lws_get_parent(wsi)
    @ccall libwebsockets.lws_get_parent(wsi::Ptr{lws})::Ptr{lws}
end

function lws_get_child(wsi)
    @ccall libwebsockets.lws_get_child(wsi::Ptr{lws})::Ptr{lws}
end

function lws_get_effective_uid_gid(context, uid, gid)
    @ccall libwebsockets.lws_get_effective_uid_gid(context::Ptr{lws_context}, uid::Ptr{uid_t}, gid::Ptr{gid_t})::Cvoid
end

function lws_get_udp(wsi)
    @ccall libwebsockets.lws_get_udp(wsi::Ptr{lws})::Ptr{lws_udp}
end

function lws_get_opaque_parent_data(wsi)
    @ccall libwebsockets.lws_get_opaque_parent_data(wsi::Ptr{lws})::Ptr{Cvoid}
end

function lws_set_opaque_parent_data(wsi, data)
    @ccall libwebsockets.lws_set_opaque_parent_data(wsi::Ptr{lws}, data::Ptr{Cvoid})::Cvoid
end

function lws_get_opaque_user_data(wsi)
    @ccall libwebsockets.lws_get_opaque_user_data(wsi::Ptr{lws})::Ptr{Cvoid}
end

function lws_set_opaque_user_data(wsi, data)
    @ccall libwebsockets.lws_set_opaque_user_data(wsi::Ptr{lws}, data::Ptr{Cvoid})::Cvoid
end

function lws_get_child_pending_on_writable(wsi)
    @ccall libwebsockets.lws_get_child_pending_on_writable(wsi::Ptr{lws})::Cint
end

function lws_clear_child_pending_on_writable(wsi)
    @ccall libwebsockets.lws_clear_child_pending_on_writable(wsi::Ptr{lws})::Cvoid
end

function lws_get_close_length(wsi)
    @ccall libwebsockets.lws_get_close_length(wsi::Ptr{lws})::Cint
end

function lws_get_close_payload(wsi)
    @ccall libwebsockets.lws_get_close_payload(wsi::Ptr{lws})::Ptr{Cuchar}
end

function lws_get_network_wsi(wsi)
    @ccall libwebsockets.lws_get_network_wsi(wsi::Ptr{lws})::Ptr{lws}
end

function lws_set_allocator(realloc)
    @ccall libwebsockets.lws_set_allocator(realloc::Ptr{Cvoid})::Cvoid
end

const __JL_Ctag_20 = UInt32
const LWS_RXFLOW_REASON_USER_BOOL = 1 % UInt32
const LWS_RXFLOW_REASON_HTTP_RXBUFFER = 64 % UInt32
const LWS_RXFLOW_REASON_H2_PPS_PENDING = 128 % UInt32
const LWS_RXFLOW_REASON_APPLIES = 16384 % UInt32
const LWS_RXFLOW_REASON_APPLIES_ENABLE_BIT = 8192 % UInt32
const LWS_RXFLOW_REASON_APPLIES_ENABLE = 24576 % UInt32
const LWS_RXFLOW_REASON_APPLIES_DISABLE = 16384 % UInt32
const LWS_RXFLOW_REASON_FLAG_PROCESS_NOW = 4096 % UInt32

function lws_rx_flow_control(wsi, enable)
    @ccall libwebsockets.lws_rx_flow_control(wsi::Ptr{lws}, enable::Cint)::Cint
end

function lws_rx_flow_allow_all_protocol(context, protocol)
    @ccall libwebsockets.lws_rx_flow_allow_all_protocol(context::Ptr{lws_context}, protocol::Ptr{lws_protocols})::Cvoid
end

function lws_remaining_packet_payload(wsi)
    @ccall libwebsockets.lws_remaining_packet_payload(wsi::Ptr{lws})::Csize_t
end

const lws_dir_obj_type_t = UInt32
const LDOT_UNKNOWN = 0 % UInt32
const LDOT_FILE = 1 % UInt32
const LDOT_DIR = 2 % UInt32
const LDOT_LINK = 3 % UInt32
const LDOT_FIFO = 4 % UInt32
const LDOTT_SOCKET = 5 % UInt32
const LDOT_CHAR = 6 % UInt32
const LDOT_BLOCK = 7 % UInt32

struct lws_dir_entry
    name::Ptr{Cchar}
    type::lws_dir_obj_type_t
end

# typedef int lws_dir_callback_function ( const char * dirpath , void * user , struct lws_dir_entry * lde )
const lws_dir_callback_function = Cvoid

function lws_dir(dirpath, user, cb)
    @ccall libwebsockets.lws_dir(dirpath::Ptr{Cchar}, user::Ptr{Cvoid}, cb::lws_dir_callback_function)::Cint
end

function lws_dir_rm_rf_cb(dirpath, user, lde)
    @ccall libwebsockets.lws_dir_rm_rf_cb(dirpath::Ptr{Cchar}, user::Ptr{Cvoid}, lde::Ptr{lws_dir_entry})::Cint
end

# typedef int ( * lws_dir_glob_cb_t ) ( void * data , const char * path )
const lws_dir_glob_cb_t = Ptr{Cvoid}

struct lws_dir_glob
    filter::Ptr{Cchar}
    cb::lws_dir_glob_cb_t
    user::Ptr{Cvoid}
end

const lws_dir_glob_t = lws_dir_glob

function lws_dir_glob_cb(dirpath, user, lde)
    @ccall libwebsockets.lws_dir_glob_cb(dirpath::Ptr{Cchar}, user::Ptr{Cvoid}, lde::Ptr{lws_dir_entry})::Cint
end

function lws_get_allocated_heap()
    @ccall libwebsockets.lws_get_allocated_heap()::Csize_t
end

function lws_get_tsi(wsi)
    @ccall libwebsockets.lws_get_tsi(wsi::Ptr{lws})::Cint
end

function lws_is_ssl(wsi)
    @ccall libwebsockets.lws_is_ssl(wsi::Ptr{lws})::Cint
end

function lws_is_cgi(wsi)
    @ccall libwebsockets.lws_is_cgi(wsi::Ptr{lws})::Cint
end

function lws_tls_jit_trust_blob_queury_skid(_blob, blen, skid, skid_len, prpder, prder_len)
    @ccall libwebsockets.lws_tls_jit_trust_blob_queury_skid(_blob::Ptr{Cvoid}, blen::Csize_t, skid::Ptr{UInt8}, skid_len::Csize_t, prpder::Ptr{Ptr{UInt8}}, prder_len::Ptr{Csize_t})::Cint
end

# automatic type deduction for variadic arguments may not be what you want, please use with caution
@generated function lws_open(__file, __oflag, va_list...)
        :(@ccall(libwebsockets.lws_open(__file::Ptr{Cchar}, __oflag::Cint; $(to_c_type_pairs(va_list)...))::Cint))
    end

struct lws_wifi_scan
    next::Ptr{lws_wifi_scan}
    ssid::NTuple{32, Cchar}
    rssi::Int32
    bssid::NTuple{6, UInt8}
    count::UInt8
    channel::UInt8
    authmode::UInt8
end

function lws_get_ssl(wsi)
    @ccall libwebsockets.lws_get_ssl(wsi::Ptr{lws})::Ptr{Cint}
end

function lws_explicit_bzero(p, len)
    @ccall libwebsockets.lws_explicit_bzero(p::Ptr{Cvoid}, len::Csize_t)::Cvoid
end

struct lws_humanize_unit
    name::Ptr{Cchar}
    factor::UInt64
end

const lws_humanize_unit_t = lws_humanize_unit

function lws_humanize(buf, len, value, schema)
    @ccall libwebsockets.lws_humanize(buf::Ptr{Cchar}, len::Csize_t, value::UInt64, schema::Ptr{lws_humanize_unit_t})::Cint
end

function lws_ser_wu16be(b, u)
    @ccall libwebsockets.lws_ser_wu16be(b::Ptr{UInt8}, u::UInt16)::Cvoid
end

function lws_ser_wu32be(b, u32)
    @ccall libwebsockets.lws_ser_wu32be(b::Ptr{UInt8}, u32::UInt32)::Cvoid
end

function lws_ser_wu64be(b, u64)
    @ccall libwebsockets.lws_ser_wu64be(b::Ptr{UInt8}, u64::UInt64)::Cvoid
end

function lws_ser_ru16be(b)
    @ccall libwebsockets.lws_ser_ru16be(b::Ptr{UInt8})::UInt16
end

function lws_ser_ru32be(b)
    @ccall libwebsockets.lws_ser_ru32be(b::Ptr{UInt8})::UInt32
end

function lws_ser_ru64be(b)
    @ccall libwebsockets.lws_ser_ru64be(b::Ptr{UInt8})::UInt64
end

function lws_vbi_encode(value, buf)
    @ccall libwebsockets.lws_vbi_encode(value::UInt64, buf::Ptr{Cvoid})::Cint
end

function lws_vbi_decode(buf, value, len)
    @ccall libwebsockets.lws_vbi_decode(buf::Ptr{Cvoid}, value::Ptr{UInt64}, len::Csize_t)::Cint
end

struct lws_fsmount
    layers_path::Ptr{Cchar}
    overlay_path::Ptr{Cchar}
    mp::NTuple{256, Cchar}
    ovname::NTuple{64, Cchar}
    distro::NTuple{64, Cchar}
end

function lws_fsmount_mount(fsm)
    @ccall libwebsockets.lws_fsmount_mount(fsm::Ptr{lws_fsmount})::Cint
end

function lws_fsmount_unmount(fsm)
    @ccall libwebsockets.lws_fsmount_unmount(fsm::Ptr{lws_fsmount})::Cint
end

function lws_dsh_create(owner, buffer_size, count_kinds)
    @ccall libwebsockets.lws_dsh_create(owner::Ptr{lws_dll2_owner_t}, buffer_size::Csize_t, count_kinds::Cint)::Ptr{lws_dsh}
end

function lws_dsh_destroy(pdsh)
    @ccall libwebsockets.lws_dsh_destroy(pdsh::Ptr{Ptr{lws_dsh}})::Cvoid
end

function lws_dsh_alloc_tail(dsh, kind, src1, size1, src2, size2)
    @ccall libwebsockets.lws_dsh_alloc_tail(dsh::Ptr{lws_dsh}, kind::Cint, src1::Ptr{Cvoid}, size1::Csize_t, src2::Ptr{Cvoid}, size2::Csize_t)::Cint
end

function lws_dsh_free(obj)
    @ccall libwebsockets.lws_dsh_free(obj::Ptr{Ptr{Cvoid}})::Cvoid
end

function lws_dsh_get_size(dsh, kind)
    @ccall libwebsockets.lws_dsh_get_size(dsh::Ptr{lws_dsh}, kind::Cint)::Csize_t
end

function lws_dsh_get_head(dsh, kind, obj, size)
    @ccall libwebsockets.lws_dsh_get_head(dsh::Ptr{lws_dsh}, kind::Cint, obj::Ptr{Ptr{Cvoid}}, size::Ptr{Csize_t})::Cint
end

function lws_dsh_describe(dsh, desc)
    @ccall libwebsockets.lws_dsh_describe(dsh::Ptr{lws_dsh}, desc::Ptr{Cchar})::Cvoid
end

function lws_service(context, timeout_ms)
    @ccall libwebsockets.lws_service(context::Ptr{lws_context}, timeout_ms::Cint)::Cint
end

function lws_cancel_service_pt(wsi)
    @ccall libwebsockets.lws_cancel_service_pt(wsi::Ptr{lws})::Cvoid
end

function lws_cancel_service(context)
    @ccall libwebsockets.lws_cancel_service(context::Ptr{lws_context})::Cvoid
end

function lws_service_fd(context, pollfd)
    @ccall libwebsockets.lws_service_fd(context::Ptr{lws_context}, pollfd::Ptr{Cvoid})::Cint
end

function lws_service_fd_tsi(context, pollfd, tsi)
    @ccall libwebsockets.lws_service_fd_tsi(context::Ptr{lws_context}, pollfd::Ptr{Cvoid}, tsi::Cint)::Cint
end

function lws_service_adjust_timeout(context, timeout_ms, tsi)
    @ccall libwebsockets.lws_service_adjust_timeout(context::Ptr{lws_context}, timeout_ms::Cint, tsi::Cint)::Cint
end

function lws_handle_POLLOUT_event(wsi, pollfd)
    @ccall libwebsockets.lws_handle_POLLOUT_event(wsi::Ptr{lws}, pollfd::Ptr{Cvoid})::Cint
end

struct lws_write_passthru
    wsi::Ptr{lws}
    buf::Ptr{Cuchar}
    len::Csize_t
    wp::lws_write_protocol
end

function lws_write_ws_flags(initial, is_start, is_end)
    @ccall libwebsockets.lws_write_ws_flags(initial::Cint, is_start::Cint, is_end::Cint)::Cint
end

function lws_raw_transaction_completed(wsi)
    @ccall libwebsockets.lws_raw_transaction_completed(wsi::Ptr{lws})::Cint
end

function lws_callback_on_writable(wsi)
    @ccall libwebsockets.lws_callback_on_writable(wsi::Ptr{lws})::Cint
end

function lws_callback_on_writable_all_protocol(context, protocol)
    @ccall libwebsockets.lws_callback_on_writable_all_protocol(context::Ptr{lws_context}, protocol::Ptr{lws_protocols})::Cint
end

function lws_callback_on_writable_all_protocol_vhost(vhost, protocol)
    @ccall libwebsockets.lws_callback_on_writable_all_protocol_vhost(vhost::Ptr{lws_vhost}, protocol::Ptr{lws_protocols})::Cint
end

function lws_callback_all_protocol(context, protocol, reason)
    @ccall libwebsockets.lws_callback_all_protocol(context::Ptr{lws_context}, protocol::Ptr{lws_protocols}, reason::Cint)::Cint
end

function lws_callback_all_protocol_vhost(vh, protocol, reason)
    @ccall libwebsockets.lws_callback_all_protocol_vhost(vh::Ptr{lws_vhost}, protocol::Ptr{lws_protocols}, reason::Cint)::Cint
end

function lws_callback_all_protocol_vhost_args(vh, protocol, reason, argp, len)
    @ccall libwebsockets.lws_callback_all_protocol_vhost_args(vh::Ptr{lws_vhost}, protocol::Ptr{lws_protocols}, reason::Cint, argp::Ptr{Cvoid}, len::Csize_t)::Cint
end

function lws_callback_vhost_protocols(wsi, reason, in, len)
    @ccall libwebsockets.lws_callback_vhost_protocols(wsi::Ptr{lws}, reason::Cint, in::Ptr{Cvoid}, len::Csize_t)::Cint
end

function lws_callback_vhost_protocols_vhost(vh, reason, in, len)
    @ccall libwebsockets.lws_callback_vhost_protocols_vhost(vh::Ptr{lws_vhost}, reason::Cint, in::Ptr{Cvoid}, len::Csize_t)::Cint
end

function lws_callback_http_dummy(wsi, reason, user, in, len)
    @ccall libwebsockets.lws_callback_http_dummy(wsi::Ptr{lws}, reason::lws_callback_reasons, user::Ptr{Cvoid}, in::Ptr{Cvoid}, len::Csize_t)::Cint
end

function lws_get_socket_fd(wsi)
    @ccall libwebsockets.lws_get_socket_fd(wsi::Ptr{lws})::lws_sockfd_type
end

function lws_get_peer_write_allowance(wsi)
    @ccall libwebsockets.lws_get_peer_write_allowance(wsi::Ptr{lws})::lws_fileofs_t
end

function lws_wsi_tx_credit(wsi, peer_to_us, add)
    @ccall libwebsockets.lws_wsi_tx_credit(wsi::Ptr{lws}, peer_to_us::Cchar, add::Cint)::Cint
end

function lws_ring_create(element_len, count, destroy_element)
    @ccall libwebsockets.lws_ring_create(element_len::Csize_t, count::Csize_t, destroy_element::Ptr{Cvoid})::Ptr{lws_ring}
end

function lws_ring_destroy(ring)
    @ccall libwebsockets.lws_ring_destroy(ring::Ptr{lws_ring})::Cvoid
end

function lws_ring_get_count_free_elements(ring)
    @ccall libwebsockets.lws_ring_get_count_free_elements(ring::Ptr{lws_ring})::Csize_t
end

function lws_ring_insert(ring, src, max_count)
    @ccall libwebsockets.lws_ring_insert(ring::Ptr{lws_ring}, src::Ptr{Cvoid}, max_count::Csize_t)::Csize_t
end

function lws_ring_get_element(ring, tail)
    @ccall libwebsockets.lws_ring_get_element(ring::Ptr{lws_ring}, tail::Ptr{UInt32})::Ptr{Cvoid}
end

function lws_ring_next_linear_insert_range(ring, start, bytes)
    @ccall libwebsockets.lws_ring_next_linear_insert_range(ring::Ptr{lws_ring}, start::Ptr{Ptr{Cvoid}}, bytes::Ptr{Csize_t})::Cint
end

function lws_ring_bump_head(ring, bytes)
    @ccall libwebsockets.lws_ring_bump_head(ring::Ptr{lws_ring}, bytes::Csize_t)::Cvoid
end

function lws_ring_dump(ring, tail)
    @ccall libwebsockets.lws_ring_dump(ring::Ptr{lws_ring}, tail::Ptr{UInt32})::Cvoid
end

function lws_SHA1(d, n, md)
    @ccall libwebsockets.lws_SHA1(d::Ptr{Cuchar}, n::Csize_t, md::Ptr{Cuchar})::Ptr{Cuchar}
end

function lws_b64_encode_string(in, in_len, out, out_size)
    @ccall libwebsockets.lws_b64_encode_string(in::Ptr{Cchar}, in_len::Cint, out::Ptr{Cchar}, out_size::Cint)::Cint
end

function lws_b64_encode_string_url(in, in_len, out, out_size)
    @ccall libwebsockets.lws_b64_encode_string_url(in::Ptr{Cchar}, in_len::Cint, out::Ptr{Cchar}, out_size::Cint)::Cint
end

function lws_b64_decode_string(in, out, out_size)
    @ccall libwebsockets.lws_b64_decode_string(in::Ptr{Cchar}, out::Ptr{Cchar}, out_size::Cint)::Cint
end

function lws_b64_decode_string_len(in, in_len, out, out_size)
    @ccall libwebsockets.lws_b64_decode_string_len(in::Ptr{Cchar}, in_len::Cint, out::Ptr{Cchar}, out_size::Cint)::Cint
end

struct lws_b64state
    quad::NTuple{4, Cuchar}
    done::Csize_t
    len::Csize_t
    i::Cint
    c::Cint
end

function lws_b64_decode_state_init(state)
    @ccall libwebsockets.lws_b64_decode_state_init(state::Ptr{lws_b64state})::Cvoid
end

function lws_b64_decode_stateful(s, in, in_len, out, out_size, final)
    @ccall libwebsockets.lws_b64_decode_stateful(s::Ptr{lws_b64state}, in::Ptr{Cchar}, in_len::Ptr{Csize_t}, out::Ptr{UInt8}, out_size::Ptr{Csize_t}, final::Cint)::Cint
end

const lws_tls_cert_info = UInt32
const LWS_TLS_CERT_INFO_VALIDITY_FROM = 0 % UInt32
const LWS_TLS_CERT_INFO_VALIDITY_TO = 1 % UInt32
const LWS_TLS_CERT_INFO_COMMON_NAME = 2 % UInt32
const LWS_TLS_CERT_INFO_ISSUER_NAME = 3 % UInt32
const LWS_TLS_CERT_INFO_USAGE = 4 % UInt32
const LWS_TLS_CERT_INFO_VERIFIED = 5 % UInt32
const LWS_TLS_CERT_INFO_OPAQUE_PUBLIC_KEY = 6 % UInt32
const LWS_TLS_CERT_INFO_DER_RAW = 7 % UInt32
const LWS_TLS_CERT_INFO_AUTHORITY_KEY_ID = 8 % UInt32
const LWS_TLS_CERT_INFO_AUTHORITY_KEY_ID_ISSUER = 9 % UInt32
const LWS_TLS_CERT_INFO_AUTHORITY_KEY_ID_SERIAL = 10 % UInt32
const LWS_TLS_CERT_INFO_SUBJECT_KEY_ID = 11 % UInt32

struct lws_tls_cert_info_results
    data::NTuple{72, UInt8}
end

function Base.getproperty(x::Ptr{lws_tls_cert_info_results}, f::Symbol)
    f === :verified && return Ptr{Cuint}(x + 0)
    f === :time && return Ptr{time_t}(x + 0)
    f === :usage && return Ptr{Cuint}(x + 0)
    f === :ns && return Ptr{__JL_Ctag_73}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::lws_tls_cert_info_results, f::Symbol)
    r = Ref{lws_tls_cert_info_results}(x)
    ptr = Base.unsafe_convert(Ptr{lws_tls_cert_info_results}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{lws_tls_cert_info_results}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

mutable struct lws_x509_cert end

function lws_x509_create(x509)
    @ccall libwebsockets.lws_x509_create(x509::Ptr{Ptr{lws_x509_cert}})::Cint
end

function lws_x509_parse_from_pem(x509, pem, len)
    @ccall libwebsockets.lws_x509_parse_from_pem(x509::Ptr{lws_x509_cert}, pem::Ptr{Cvoid}, len::Csize_t)::Cint
end

function lws_x509_verify(x509, trusted, common_name)
    @ccall libwebsockets.lws_x509_verify(x509::Ptr{lws_x509_cert}, trusted::Ptr{lws_x509_cert}, common_name::Ptr{Cchar})::Cint
end

struct lws_gencrypto_keyelem
    buf::Ptr{UInt8}
    len::UInt32
end

struct lws_jwk
    e::NTuple{12, lws_gencrypto_keyelem}
    meta::NTuple{6, lws_gencrypto_keyelem}
    kty::Cint
    private_key::Cchar
end

function lws_x509_public_to_jwk(jwk, x509, curves, rsabits)
    @ccall libwebsockets.lws_x509_public_to_jwk(jwk::Ptr{lws_jwk}, x509::Ptr{lws_x509_cert}, curves::Ptr{Cchar}, rsabits::Cint)::Cint
end

function lws_x509_jwk_privkey_pem(cx, jwk, pem, len, passphrase)
    @ccall libwebsockets.lws_x509_jwk_privkey_pem(cx::Ptr{lws_context}, jwk::Ptr{lws_jwk}, pem::Ptr{Cvoid}, len::Csize_t, passphrase::Ptr{Cchar})::Cint
end

function lws_x509_destroy(x509)
    @ccall libwebsockets.lws_x509_destroy(x509::Ptr{Ptr{lws_x509_cert}})::Cvoid
end

function lws_x509_info(x509, type, buf, len)
    @ccall libwebsockets.lws_x509_info(x509::Ptr{lws_x509_cert}, type::lws_tls_cert_info, buf::Ptr{lws_tls_cert_info_results}, len::Csize_t)::Cint
end

function lws_tls_peer_cert_info(wsi, type, buf, len)
    @ccall libwebsockets.lws_tls_peer_cert_info(wsi::Ptr{lws}, type::lws_tls_cert_info, buf::Ptr{lws_tls_cert_info_results}, len::Csize_t)::Cint
end

function lws_tls_vhost_cert_info(vhost, type, buf, len)
    @ccall libwebsockets.lws_tls_vhost_cert_info(vhost::Ptr{lws_vhost}, type::lws_tls_cert_info, buf::Ptr{lws_tls_cert_info_results}, len::Csize_t)::Cint
end

function lws_tls_acme_sni_cert_create(vhost, san_a, san_b)
    @ccall libwebsockets.lws_tls_acme_sni_cert_create(vhost::Ptr{lws_vhost}, san_a::Ptr{Cchar}, san_b::Ptr{Cchar})::Cint
end

function lws_tls_acme_sni_csr_create(context, elements, csr, csr_len, privkey_pem, privkey_len)
    @ccall libwebsockets.lws_tls_acme_sni_csr_create(context::Ptr{lws_context}, elements::Ptr{Ptr{Cchar}}, csr::Ptr{UInt8}, csr_len::Csize_t, privkey_pem::Ptr{Ptr{Cchar}}, privkey_len::Ptr{Csize_t})::Cint
end

function lws_tls_cert_updated(context, certpath, keypath, mem_cert, len_mem_cert, mem_privkey, len_mem_privkey)
    @ccall libwebsockets.lws_tls_cert_updated(context::Ptr{lws_context}, certpath::Ptr{Cchar}, keypath::Ptr{Cchar}, mem_cert::Ptr{Cchar}, len_mem_cert::Csize_t, mem_privkey::Ptr{Cchar}, len_mem_privkey::Csize_t)::Cint
end

const lws_enum_stdinouterr = UInt32
const LWS_STDIN = 0 % UInt32
const LWS_STDOUT = 1 % UInt32
const LWS_STDERR = 2 % UInt32

const lws_cgi_hdr_state = UInt32
const LCHS_HEADER = 0 % UInt32
const LCHS_CR1 = 1 % UInt32
const LCHS_LF1 = 2 % UInt32
const LCHS_CR2 = 3 % UInt32
const LCHS_LF2 = 4 % UInt32
const LHCS_RESPONSE = 5 % UInt32
const LHCS_DUMP_HEADERS = 6 % UInt32
const LHCS_PAYLOAD = 7 % UInt32
const LCHS_SINGLE_0A = 8 % UInt32

struct lws_cgi_args
    stdwsi::Ptr{Ptr{lws}}
    ch::lws_enum_stdinouterr
    data::Ptr{Cuchar}
    hdr_state::lws_cgi_hdr_state
    len::Cint
end

struct lws_fops_index
    sig::Ptr{Cchar}
    len::UInt8
end

struct lws_plat_file_ops
    open::Ptr{Cvoid}
    close::Ptr{Cvoid}
    seek_cur::Ptr{Cvoid}
    read::Ptr{Cvoid}
    write::Ptr{Cvoid}
    fi::NTuple{3, lws_fops_index}
    next::Ptr{lws_plat_file_ops}
end

struct lws_fop_fd
    fd::lws_filefd_type
    fops::Ptr{lws_plat_file_ops}
    filesystem_priv::Ptr{Cvoid}
    pos::lws_filepos_t
    len::lws_filepos_t
    flags::lws_fop_flags_t
    mod_time::UInt32
end

const lws_fop_fd_t = Ptr{lws_fop_fd}

function lws_get_fops(context)
    @ccall libwebsockets.lws_get_fops(context::Ptr{lws_context})::Ptr{lws_plat_file_ops}
end

function lws_set_fops(context, fops)
    @ccall libwebsockets.lws_set_fops(context::Ptr{lws_context}, fops::Ptr{lws_plat_file_ops})::Cvoid
end

function lws_vfs_tell(fop_fd)
    @ccall libwebsockets.lws_vfs_tell(fop_fd::lws_fop_fd_t)::lws_filepos_t
end

function lws_vfs_get_length(fop_fd)
    @ccall libwebsockets.lws_vfs_get_length(fop_fd::lws_fop_fd_t)::lws_filepos_t
end

function lws_vfs_get_mod_time(fop_fd)
    @ccall libwebsockets.lws_vfs_get_mod_time(fop_fd::lws_fop_fd_t)::UInt32
end

function lws_vfs_file_seek_set(fop_fd, offset)
    @ccall libwebsockets.lws_vfs_file_seek_set(fop_fd::lws_fop_fd_t, offset::lws_fileofs_t)::lws_fileofs_t
end

function lws_vfs_file_seek_end(fop_fd, offset)
    @ccall libwebsockets.lws_vfs_file_seek_end(fop_fd::lws_fop_fd_t, offset::lws_fileofs_t)::lws_fileofs_t
end

function lws_vfs_file_open(fops, vfs_path, flags)
    @ccall libwebsockets.lws_vfs_file_open(fops::Ptr{lws_plat_file_ops}, vfs_path::Ptr{Cchar}, flags::Ptr{lws_fop_flags_t})::lws_fop_fd_t
end

function lws_vfs_file_close(fop_fd)
    @ccall libwebsockets.lws_vfs_file_close(fop_fd::Ptr{lws_fop_fd_t})::Cint
end

function lws_vfs_file_seek_cur(fop_fd, offset)
    @ccall libwebsockets.lws_vfs_file_seek_cur(fop_fd::lws_fop_fd_t, offset::lws_fileofs_t)::lws_fileofs_t
end

function lws_vfs_file_read(fop_fd, amount, buf, len)
    @ccall libwebsockets.lws_vfs_file_read(fop_fd::lws_fop_fd_t, amount::Ptr{lws_filepos_t}, buf::Ptr{UInt8}, len::lws_filepos_t)::Cint
end

function lws_vfs_file_write(fop_fd, amount, buf, len)
    @ccall libwebsockets.lws_vfs_file_write(fop_fd::lws_fop_fd_t, amount::Ptr{lws_filepos_t}, buf::Ptr{UInt8}, len::lws_filepos_t)::Cint
end

function _lws_plat_file_open(fops, filename, vpath, flags)
    @ccall libwebsockets._lws_plat_file_open(fops::Ptr{lws_plat_file_ops}, filename::Ptr{Cchar}, vpath::Ptr{Cchar}, flags::Ptr{lws_fop_flags_t})::lws_fop_fd_t
end

function _lws_plat_file_close(fop_fd)
    @ccall libwebsockets._lws_plat_file_close(fop_fd::Ptr{lws_fop_fd_t})::Cint
end

function _lws_plat_file_seek_cur(fop_fd, offset)
    @ccall libwebsockets._lws_plat_file_seek_cur(fop_fd::lws_fop_fd_t, offset::lws_fileofs_t)::lws_fileofs_t
end

function _lws_plat_file_read(fop_fd, amount, buf, len)
    @ccall libwebsockets._lws_plat_file_read(fop_fd::lws_fop_fd_t, amount::Ptr{lws_filepos_t}, buf::Ptr{UInt8}, len::lws_filepos_t)::Cint
end

function _lws_plat_file_write(fop_fd, amount, buf, len)
    @ccall libwebsockets._lws_plat_file_write(fop_fd::lws_fop_fd_t, amount::Ptr{lws_filepos_t}, buf::Ptr{UInt8}, len::lws_filepos_t)::Cint
end

function lws_alloc_vfs_file(context, filename, buf, amount)
    @ccall libwebsockets.lws_alloc_vfs_file(context::Ptr{lws_context}, filename::Ptr{Cchar}, buf::Ptr{Ptr{UInt8}}, amount::Ptr{lws_filepos_t})::Cint
end

const lws_gencrypto_kty = UInt32
const LWS_GENCRYPTO_KTY_UNKNOWN = 0 % UInt32
const LWS_GENCRYPTO_KTY_OCT = 1 % UInt32
const LWS_GENCRYPTO_KTY_RSA = 2 % UInt32
const LWS_GENCRYPTO_KTY_EC = 3 % UInt32

const lws_gencrypto_oct_tok = UInt32
const LWS_GENCRYPTO_OCT_KEYEL_K = 0 % UInt32
const LWS_GENCRYPTO_OCT_KEYEL_COUNT = 1 % UInt32

const lws_gencrypto_rsa_tok = UInt32
const LWS_GENCRYPTO_RSA_KEYEL_E = 0 % UInt32
const LWS_GENCRYPTO_RSA_KEYEL_N = 1 % UInt32
const LWS_GENCRYPTO_RSA_KEYEL_D = 2 % UInt32
const LWS_GENCRYPTO_RSA_KEYEL_P = 3 % UInt32
const LWS_GENCRYPTO_RSA_KEYEL_Q = 4 % UInt32
const LWS_GENCRYPTO_RSA_KEYEL_DP = 5 % UInt32
const LWS_GENCRYPTO_RSA_KEYEL_DQ = 6 % UInt32
const LWS_GENCRYPTO_RSA_KEYEL_QI = 7 % UInt32
const LWS_GENCRYPTO_RSA_KEYEL_OTHER = 8 % UInt32
const LWS_GENCRYPTO_RSA_KEYEL_RI = 9 % UInt32
const LWS_GENCRYPTO_RSA_KEYEL_DI = 10 % UInt32
const LWS_GENCRYPTO_RSA_KEYEL_TI = 11 % UInt32
const LWS_GENCRYPTO_RSA_KEYEL_COUNT = 12 % UInt32

const lws_gencrypto_ec_tok = UInt32
const LWS_GENCRYPTO_EC_KEYEL_CRV = 0 % UInt32
const LWS_GENCRYPTO_EC_KEYEL_X = 1 % UInt32
const LWS_GENCRYPTO_EC_KEYEL_D = 2 % UInt32
const LWS_GENCRYPTO_EC_KEYEL_Y = 3 % UInt32
const LWS_GENCRYPTO_EC_KEYEL_COUNT = 4 % UInt32

const lws_gencrypto_aes_tok = UInt32
const LWS_GENCRYPTO_AES_KEYEL_K = 0 % UInt32
const LWS_GENCRYPTO_AES_KEYEL_COUNT = 1 % UInt32

const lws_gc_elem_t = lws_gencrypto_keyelem

function lws_gencrypto_bits_to_bytes(bits)
    @ccall libwebsockets.lws_gencrypto_bits_to_bytes(bits::Cint)::Cint
end

function lws_base64_size(bytes)
    @ccall libwebsockets.lws_base64_size(bytes::Cint)::Cint
end

function lws_gencrypto_padded_length(block_size, len)
    @ccall libwebsockets.lws_gencrypto_padded_length(block_size::Csize_t, len::Csize_t)::Csize_t
end

const lejp_states = UInt32
const LEJP_IDLE = 0 % UInt32
const LEJP_MEMBERS = 1 % UInt32
const LEJP_M_P = 2 % UInt32
const LEJP_MP_STRING = 67 % UInt32
const LEJP_MP_STRING_ESC = 68 % UInt32
const LEJP_MP_STRING_ESC_U1 = 69 % UInt32
const LEJP_MP_STRING_ESC_U2 = 70 % UInt32
const LEJP_MP_STRING_ESC_U3 = 71 % UInt32
const LEJP_MP_STRING_ESC_U4 = 72 % UInt32
const LEJP_MP_DELIM = 9 % UInt32
const LEJP_MP_VALUE = 10 % UInt32
const LEJP_MP_VALUE_NUM_INT = 75 % UInt32
const LEJP_MP_VALUE_NUM_EXP = 76 % UInt32
const LEJP_MP_VALUE_TOK = 77 % UInt32
const LEJP_MP_COMMA_OR_END = 14 % UInt32
const LEJP_MP_ARRAY_END = 15 % UInt32

const lejp_reasons = Int32
const LEJP_CONTINUE = -1 % Int32
const LEJP_REJECT_IDLE_NO_BRACE = -2 % Int32
const LEJP_REJECT_MEMBERS_NO_CLOSE = -3 % Int32
const LEJP_REJECT_MP_NO_OPEN_QUOTE = -4 % Int32
const LEJP_REJECT_MP_STRING_UNDERRUN = -5 % Int32
const LEJP_REJECT_MP_ILLEGAL_CTRL = -6 % Int32
const LEJP_REJECT_MP_STRING_ESC_ILLEGAL_ESC = -7 % Int32
const LEJP_REJECT_ILLEGAL_HEX = -8 % Int32
const LEJP_REJECT_MP_DELIM_MISSING_COLON = -9 % Int32
const LEJP_REJECT_MP_DELIM_BAD_VALUE_START = -10 % Int32
const LEJP_REJECT_MP_VAL_NUM_INT_NO_FRAC = -11 % Int32
const LEJP_REJECT_MP_VAL_NUM_FORMAT = -12 % Int32
const LEJP_REJECT_MP_VAL_NUM_EXP_BAD_EXP = -13 % Int32
const LEJP_REJECT_MP_VAL_TOK_UNKNOWN = -14 % Int32
const LEJP_REJECT_MP_C_OR_E_UNDERF = -15 % Int32
const LEJP_REJECT_MP_C_OR_E_NOTARRAY = -16 % Int32
const LEJP_REJECT_MP_ARRAY_END_MISSING = -17 % Int32
const LEJP_REJECT_STACK_OVERFLOW = -18 % Int32
const LEJP_REJECT_MP_DELIM_ISTACK = -19 % Int32
const LEJP_REJECT_NUM_TOO_LONG = -20 % Int32
const LEJP_REJECT_MP_C_OR_E_NEITHER = -21 % Int32
const LEJP_REJECT_UNKNOWN = -22 % Int32
const LEJP_REJECT_CALLBACK = -23 % Int32

const lejp_callbacks = UInt32
const LEJPCB_CONSTRUCTED = 0 % UInt32
const LEJPCB_DESTRUCTED = 1 % UInt32
const LEJPCB_START = 2 % UInt32
const LEJPCB_COMPLETE = 3 % UInt32
const LEJPCB_FAILED = 4 % UInt32
const LEJPCB_PAIR_NAME = 5 % UInt32
const LEJPCB_VAL_TRUE = 70 % UInt32
const LEJPCB_VAL_FALSE = 71 % UInt32
const LEJPCB_VAL_NULL = 72 % UInt32
const LEJPCB_VAL_NUM_INT = 73 % UInt32
const LEJPCB_VAL_NUM_FLOAT = 74 % UInt32
const LEJPCB_VAL_STR_START = 11 % UInt32
const LEJPCB_VAL_STR_CHUNK = 76 % UInt32
const LEJPCB_VAL_STR_END = 77 % UInt32
const LEJPCB_ARRAY_START = 14 % UInt32
const LEJPCB_ARRAY_END = 15 % UInt32
const LEJPCB_OBJECT_START = 16 % UInt32
const LEJPCB_OBJECT_END = 17 % UInt32

struct _lejp_parsing_stack
    user::Ptr{Cvoid}
    callback::Ptr{Cvoid}
    paths::Ptr{Ptr{Cchar}}
    count_paths::UInt8
    ppos::UInt8
    path_match::UInt8
end

struct _lejp_stack
    s::Cchar
    p::Cchar
    i::Cchar
    b::Cchar
end

struct lejp_ctx
    user::Ptr{Cvoid}
    pst::NTuple{5, _lejp_parsing_stack}
    st::NTuple{12, _lejp_stack}
    i::NTuple{8, UInt16}
    wild::NTuple{8, UInt16}
    path::NTuple{128, Cchar}
    buf::NTuple{255, Cchar}
    path_stride::Csize_t
    line::UInt32
    uni::UInt16
    npos::UInt8
    dcount::UInt8
    f::UInt8
    sp::UInt8
    ipos::UInt8
    count_paths::UInt8
    path_match::UInt8
    path_match_len::UInt8
    wildcount::UInt8
    pst_sp::UInt8
    outer_array::UInt8
end

function _lejp_callback(ctx, reason)
    @ccall libwebsockets._lejp_callback(ctx::Ptr{lejp_ctx}, reason::Cchar)::Int8
end

# typedef signed char ( * lejp_callback ) ( struct lejp_ctx * ctx , char reason )
const lejp_callback = Ptr{Cvoid}

const num_flags = UInt32
const LEJP_SEEN_MINUS = 1 % UInt32
const LEJP_SEEN_POINT = 2 % UInt32
const LEJP_SEEN_POST_POINT = 4 % UInt32
const LEJP_SEEN_EXP = 8 % UInt32

function lejp_construct(ctx, callback, user, paths, paths_count)
    @ccall libwebsockets.lejp_construct(ctx::Ptr{lejp_ctx}, callback::Ptr{Cvoid}, user::Ptr{Cvoid}, paths::Ptr{Ptr{Cchar}}, paths_count::Cuchar)::Cvoid
end

function lejp_destruct(ctx)
    @ccall libwebsockets.lejp_destruct(ctx::Ptr{lejp_ctx})::Cvoid
end

function lejp_parse(ctx, json, len)
    @ccall libwebsockets.lejp_parse(ctx::Ptr{lejp_ctx}, json::Ptr{Cuchar}, len::Cint)::Cint
end

function lejp_change_callback(ctx, callback)
    @ccall libwebsockets.lejp_change_callback(ctx::Ptr{lejp_ctx}, callback::Ptr{Cvoid})::Cvoid
end

function lejp_parser_push(ctx, user, paths, paths_count, lejp_cb)
    @ccall libwebsockets.lejp_parser_push(ctx::Ptr{lejp_ctx}, user::Ptr{Cvoid}, paths::Ptr{Ptr{Cchar}}, paths_count::Cuchar, lejp_cb::lejp_callback)::Cint
end

function lejp_parser_pop(ctx)
    @ccall libwebsockets.lejp_parser_pop(ctx::Ptr{lejp_ctx})::Cint
end

function lejp_check_path_match(ctx)
    @ccall libwebsockets.lejp_check_path_match(ctx::Ptr{lejp_ctx})::Cvoid
end

function lejp_get_wildcard(ctx, wildcard, dest, len)
    @ccall libwebsockets.lejp_get_wildcard(ctx::Ptr{lejp_ctx}, wildcard::Cint, dest::Ptr{Cchar}, len::Cint)::Cint
end

function lejp_error_to_string(e)
    @ccall libwebsockets.lejp_error_to_string(e::Cint)::Ptr{Cchar}
end

const __JL_Ctag_22 = UInt32
const LWS_CBOR_MAJTYP_UINT = 0 % UInt32
const LWS_CBOR_MAJTYP_INT_NEG = 32 % UInt32
const LWS_CBOR_MAJTYP_BSTR = 64 % UInt32
const LWS_CBOR_MAJTYP_TSTR = 96 % UInt32
const LWS_CBOR_MAJTYP_ARRAY = 128 % UInt32
const LWS_CBOR_MAJTYP_MAP = 160 % UInt32
const LWS_CBOR_MAJTYP_TAG = 192 % UInt32
const LWS_CBOR_MAJTYP_FLOAT = 224 % UInt32
const LWS_CBOR_MAJTYP_MASK = 224 % UInt32
const LWS_CBOR_1 = 24 % UInt32
const LWS_CBOR_2 = 25 % UInt32
const LWS_CBOR_4 = 26 % UInt32
const LWS_CBOR_8 = 27 % UInt32
const LWS_CBOR_RESERVED = 28 % UInt32
const LWS_CBOR_SUBMASK = 31 % UInt32
const LWS_CBOR_SWK_FALSE = 20 % UInt32
const LWS_CBOR_SWK_TRUE = 21 % UInt32
const LWS_CBOR_SWK_NULL = 22 % UInt32
const LWS_CBOR_SWK_UNDEFINED = 23 % UInt32
const LWS_CBOR_M7_SUBTYP_SIMPLE_X8 = 24 % UInt32
const LWS_CBOR_M7_SUBTYP_FLOAT16 = 25 % UInt32
const LWS_CBOR_M7_SUBTYP_FLOAT32 = 26 % UInt32
const LWS_CBOR_M7_SUBTYP_FLOAT64 = 27 % UInt32
const LWS_CBOR_M7_BREAK = 31 % UInt32
const LWS_CBOR_INDETERMINITE = 31 % UInt32
const LWS_CBOR_WKTAG_DATETIME_STD = 0 % UInt32
const LWS_CBOR_WKTAG_DATETIME_EPOCH = 1 % UInt32
const LWS_CBOR_WKTAG_BIGNUM_UNSIGNED = 2 % UInt32
const LWS_CBOR_WKTAG_BIGNUM_NEGATIVE = 3 % UInt32
const LWS_CBOR_WKTAG_DECIMAL_FRAC = 4 % UInt32
const LWS_CBOR_WKTAG_BIGFLOAT = 5 % UInt32
const LWS_CBOR_WKTAG_COSE_ENC0 = 16 % UInt32
const LWS_CBOR_WKTAG_COSE_MAC0 = 17 % UInt32
const LWS_CBOR_WKTAG_COSE_SIGN1 = 18 % UInt32
const LWS_CBOR_WKTAG_TO_B64U = 21 % UInt32
const LWS_CBOR_WKTAG_TO_B64 = 22 % UInt32
const LWS_CBOR_WKTAG_TO_B16 = 23 % UInt32
const LWS_CBOR_WKTAG_CBOR = 24 % UInt32
const LWS_CBOR_WKTAG_URI = 32 % UInt32
const LWS_CBOR_WKTAG_B64U = 33 % UInt32
const LWS_CBOR_WKTAG_B64 = 34 % UInt32
const LWS_CBOR_WKTAG_MIME = 36 % UInt32
const LWS_CBOR_WKTAG_COSE_ENC = 96 % UInt32
const LWS_CBOR_WKTAG_COSE_MAC = 97 % UInt32
const LWS_CBOR_WKTAG_COSE_SIGN = 98 % UInt32
const LWS_CBOR_WKTAG_SELFDESCCBOR = 55799 % UInt32

const lecp_callbacks = UInt32
const LECPCB_CONSTRUCTED = 0 % UInt32
const LECPCB_DESTRUCTED = 1 % UInt32
const LECPCB_COMPLETE = 3 % UInt32
const LECPCB_FAILED = 4 % UInt32
const LECPCB_PAIR_NAME = 5 % UInt32
const LECPCB_VAL_TRUE = 70 % UInt32
const LECPCB_VAL_FALSE = 71 % UInt32
const LECPCB_VAL_NULL = 72 % UInt32
const LECPCB_VAL_NUM_INT = 73 % UInt32
const LECPCB_VAL_RESERVED = 74 % UInt32
const LECPCB_VAL_STR_START = 11 % UInt32
const LECPCB_VAL_STR_CHUNK = 76 % UInt32
const LECPCB_VAL_STR_END = 77 % UInt32
const LECPCB_ARRAY_START = 14 % UInt32
const LECPCB_ARRAY_END = 15 % UInt32
const LECPCB_OBJECT_START = 16 % UInt32
const LECPCB_OBJECT_END = 17 % UInt32
const LECPCB_TAG_START = 18 % UInt32
const LECPCB_TAG_END = 19 % UInt32
const LECPCB_VAL_NUM_UINT = 84 % UInt32
const LECPCB_VAL_UNDEFINED = 85 % UInt32
const LECPCB_VAL_FLOAT16 = 86 % UInt32
const LECPCB_VAL_FLOAT32 = 87 % UInt32
const LECPCB_VAL_FLOAT64 = 88 % UInt32
const LECPCB_VAL_SIMPLE = 89 % UInt32
const LECPCB_VAL_BLOB_START = 26 % UInt32
const LECPCB_VAL_BLOB_CHUNK = 91 % UInt32
const LECPCB_VAL_BLOB_END = 92 % UInt32
const LECPCB_ARRAY_ITEM_START = 29 % UInt32
const LECPCB_ARRAY_ITEM_END = 30 % UInt32
const LECPCB_LITERAL_CBOR = 31 % UInt32

const lecp_reasons = Int32
const LECP_CONTINUE = -1 % Int32
const LECP_REJECT_BAD_CODING = -2 % Int32
const LECP_REJECT_UNKNOWN = -3 % Int32
const LECP_REJECT_CALLBACK = -4 % Int32
const LECP_STACK_OVERFLOW = -5 % Int32

struct __JL_Ctag_63
    data::NTuple{8, UInt8}
end

function Base.getproperty(x::Ptr{__JL_Ctag_63}, f::Symbol)
    f === :u64 && return Ptr{UInt64}(x + 0)
    f === :i64 && return Ptr{Int64}(x + 0)
    f === :u32 && return Ptr{UInt64}(x + 0)
    f === :hf && return Ptr{UInt16}(x + 0)
    f === :f && return Ptr{Cfloat}(x + 0)
    f === :d && return Ptr{Cdouble}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_63, f::Symbol)
    r = Ref{__JL_Ctag_63}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_63}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_63}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

struct lecp_item
    data::NTuple{16, UInt8}
end

function Base.getproperty(x::Ptr{lecp_item}, f::Symbol)
    f === :u && return Ptr{__JL_Ctag_63}(x + 0)
    f === :opcode && return Ptr{UInt8}(x + 8)
    return getfield(x, f)
end

function Base.getproperty(x::lecp_item, f::Symbol)
    r = Ref{lecp_item}(x)
    ptr = Base.unsafe_convert(Ptr{lecp_item}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{lecp_item}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

# typedef signed char ( * lecp_callback ) ( struct lecp_ctx * ctx , char reason )
const lecp_callback = Ptr{Cvoid}

struct _lecp_stack
    s::Cchar
    p::UInt8
    i::Cchar
    indet::Cchar
    intermediate::Cchar
    pop_iss::Cchar
    tag::UInt64
    collect_rem::UInt64
    ordinal::UInt32
    opcode::UInt8
    send_new_array_item::UInt8
    barrier::UInt8
end

struct _lecp_parsing_stack
    user::Ptr{Cvoid}
    cb::lecp_callback
    paths::Ptr{Ptr{Cchar}}
    count_paths::UInt8
    ppos::UInt8
    path_match::UInt8
end

struct lecp_ctx
    data::NTuple{1088, UInt8}
end

function Base.getproperty(x::Ptr{lecp_ctx}, f::Symbol)
    f === :user && return Ptr{Ptr{Cvoid}}(x + 0)
    f === :collect_tgt && return Ptr{Ptr{UInt8}}(x + 8)
    f === :pst && return Ptr{NTuple{5, _lecp_parsing_stack}}(x + 16)
    f === :st && return Ptr{NTuple{12, _lecp_stack}}(x + 176)
    f === :i && return Ptr{NTuple{8, UInt16}}(x + 560)
    f === :wild && return Ptr{NTuple{8, UInt16}}(x + 576)
    f === :path && return Ptr{NTuple{128, Cchar}}(x + 592)
    f === :cbor && return Ptr{NTuple{64, UInt8}}(x + 720)
    f === :item && return Ptr{lecp_item}(x + 784)
    f === :path_stride && return Ptr{Csize_t}(x + 800)
    f === :used_in && return Ptr{Csize_t}(x + 808)
    f === :uni && return Ptr{UInt16}(x + 816)
    f === :npos && return Ptr{UInt8}(x + 818)
    f === :dcount && return Ptr{UInt8}(x + 819)
    f === :f && return Ptr{UInt8}(x + 820)
    f === :sp && return Ptr{UInt8}(x + 821)
    f === :ipos && return Ptr{UInt8}(x + 822)
    f === :count_paths && return Ptr{UInt8}(x + 823)
    f === :path_match && return Ptr{UInt8}(x + 824)
    f === :path_match_len && return Ptr{UInt8}(x + 825)
    f === :wildcount && return Ptr{UInt8}(x + 826)
    f === :pst_sp && return Ptr{UInt8}(x + 827)
    f === :outer_array && return Ptr{UInt8}(x + 828)
    f === :cbor_pos && return Ptr{UInt8}(x + 829)
    f === :literal_cbor_report && return Ptr{UInt8}(x + 830)
    f === :present && return Ptr{Cchar}(x + 831)
    f === :be && return Ptr{UInt8}(x + 832)
    f === :buf && return Ptr{NTuple{255, Cchar}}(x + 833)
    return getfield(x, f)
end

function Base.getproperty(x::lecp_ctx, f::Symbol)
    r = Ref{lecp_ctx}(x)
    ptr = Base.unsafe_convert(Ptr{lecp_ctx}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{lecp_ctx}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

const lws_lec_pctx_ret = UInt32
const LWS_LECPCTX_RET_FINISHED = 0 % UInt32
const LWS_LECPCTX_RET_AGAIN = 1 % UInt32
const LWS_LECPCTX_RET_FAIL = 2 % UInt32

const cbp_state = UInt32
const CBPS_IDLE = 0 % UInt32
const CBPS_PC1 = 1 % UInt32
const CBPS_PC2 = 2 % UInt32
const CBPS_PC3 = 3 % UInt32
const CBPS_STRING_BODY = 4 % UInt32
const CBPS_NUM_LIT = 5 % UInt32
const CBPS_STRING_LIT = 6 % UInt32
const CBPS_CONTYPE = 7 % UInt32

struct lws_lec_pctx
    data::NTuple{176, UInt8}
end

function Base.getproperty(x::Ptr{lws_lec_pctx}, f::Symbol)
    f === :stack && return Ptr{NTuple{16, UInt8}}(x + 0)
    f === :vaa && return Ptr{NTuple{16, UInt8}}(x + 16)
    f === :indet && return Ptr{NTuple{16, UInt8}}(x + 32)
    f === :scratch && return Ptr{NTuple{24, UInt8}}(x + 48)
    f === :start && return Ptr{Ptr{UInt8}}(x + 72)
    f === :buf && return Ptr{Ptr{UInt8}}(x + 80)
    f === :_end && return Ptr{Ptr{UInt8}}(x + 88)
    f === :ongoing_src && return Ptr{Ptr{UInt8}}(x + 96)
    f === :ongoing_len && return Ptr{UInt64}(x + 104)
    f === :ongoing_done && return Ptr{UInt64}(x + 112)
    f === :item && return Ptr{lecp_item}(x + 120)
    f === :used && return Ptr{Csize_t}(x + 136)
    f === :opaque && return Ptr{NTuple{4, Cint}}(x + 144)
    f === :state && return Ptr{cbp_state}(x + 160)
    f === :fmt_pos && return Ptr{Cuint}(x + 164)
    f === :sp && return Ptr{UInt8}(x + 168)
    f === :scratch_len && return Ptr{UInt8}(x + 169)
    f === :escflag && return Ptr{UInt8}(x + 170)
    f === :_long && return Ptr{UInt8}(x + 171)
    f === :vaa_pos && return Ptr{UInt8}(x + 172)
    f === :dotstar && return Ptr{UInt8}(x + 173)
    return getfield(x, f)
end

function Base.getproperty(x::lws_lec_pctx, f::Symbol)
    r = Ref{lws_lec_pctx}(x)
    ptr = Base.unsafe_convert(Ptr{lws_lec_pctx}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{lws_lec_pctx}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

const lws_lec_pctx_t = lws_lec_pctx

function lws_lec_int(ctx, opcode, indet, num)
    @ccall libwebsockets.lws_lec_int(ctx::Ptr{lws_lec_pctx_t}, opcode::UInt8, indet::UInt8, num::UInt64)::Cvoid
end

function lws_lec_scratch(ctx)
    @ccall libwebsockets.lws_lec_scratch(ctx::Ptr{lws_lec_pctx_t})::Cint
end

function lws_lec_init(ctx, buf, len)
    @ccall libwebsockets.lws_lec_init(ctx::Ptr{lws_lec_pctx_t}, buf::Ptr{UInt8}, len::Csize_t)::Cvoid
end

function lws_lec_setbuf(ctx, buf, len)
    @ccall libwebsockets.lws_lec_setbuf(ctx::Ptr{lws_lec_pctx_t}, buf::Ptr{UInt8}, len::Csize_t)::Cvoid
end

# automatic type deduction for variadic arguments may not be what you want, please use with caution
@generated function lws_lec_printf(ctx, format, va_list...)
        :(@ccall(libwebsockets.lws_lec_printf(ctx::Ptr{lws_lec_pctx_t}, format::Ptr{Cchar}; $(to_c_type_pairs(va_list)...))::lws_lec_pctx_ret))
    end

function lecp_construct(ctx, cb, user, paths, paths_count)
    @ccall libwebsockets.lecp_construct(ctx::Ptr{lecp_ctx}, cb::lecp_callback, user::Ptr{Cvoid}, paths::Ptr{Ptr{Cchar}}, paths_count::Cuchar)::Cvoid
end

function lecp_destruct(ctx)
    @ccall libwebsockets.lecp_destruct(ctx::Ptr{lecp_ctx})::Cvoid
end

function lecp_parse(ctx, cbor, len)
    @ccall libwebsockets.lecp_parse(ctx::Ptr{lecp_ctx}, cbor::Ptr{UInt8}, len::Csize_t)::Cint
end

function lecp_change_callback(ctx, cb)
    @ccall libwebsockets.lecp_change_callback(ctx::Ptr{lecp_ctx}, cb::lecp_callback)::Cvoid
end

function lecp_error_to_string(e)
    @ccall libwebsockets.lecp_error_to_string(e::Cint)::Ptr{Cchar}
end

function lecp_parse_report_raw(ctx, on)
    @ccall libwebsockets.lecp_parse_report_raw(ctx::Ptr{lecp_ctx}, on::Cint)::Cvoid
end

function lecp_parse_map_is_key(ctx)
    @ccall libwebsockets.lecp_parse_map_is_key(ctx::Ptr{lecp_ctx})::Cint
end

function lecp_parse_subtree(ctx, in, len)
    @ccall libwebsockets.lecp_parse_subtree(ctx::Ptr{lecp_ctx}, in::Ptr{UInt8}, len::Csize_t)::Cint
end

function lws_singles2halfp(hp, x)
    @ccall libwebsockets.lws_singles2halfp(hp::Ptr{UInt16}, x::UInt32)::Cvoid
end

function lws_halfp2singles(xp, h)
    @ccall libwebsockets.lws_halfp2singles(xp::Ptr{UInt32}, h::UInt16)::Cvoid
end

const __JL_Ctag_23 = Int32
const LWSCOSE_WKL_ALG = 1 % Int32
const LWSCOSE_WKL_CRIT = 2 % Int32
const LWSCOSE_WKL_CONTENT_TYPE = 3 % Int32
const LWSCOSE_WKL_KID = 4 % Int32
const LWSCOSE_WKL_IV = 5 % Int32
const LWSCOSE_WKL_IV_PARTIAL = 6 % Int32
const LWSCOSE_WKL_COUNTERSIG = 7 % Int32
const LWSCOSE_WKL_COUNTERSIG0 = 9 % Int32
const LWSCOSE_WKL_KID_CONTEXT = 10 % Int32
const LWSCOSE_WKL_CUPH_NONCE = 256 % Int32
const LWSCOSE_WKL_CUPH_OWNER_PUBKEY = 257 % Int32
const LWSCOSE_WKK_KTY = 1 % Int32
const LWSCOSE_WKK_KID = 2 % Int32
const LWSCOSE_WKK_ALG = 3 % Int32
const LWSCOSE_WKK_KEY_OPS = 4 % Int32
const LWSCOSE_WKK_BASE_IV = 5 % Int32
const LWSCOSE_WKKO_SIGN = 1 % Int32
const LWSCOSE_WKKO_VERIFY = 2 % Int32
const LWSCOSE_WKKO_ENCRYPT = 3 % Int32
const LWSCOSE_WKKO_DECRYPT = 4 % Int32
const LWSCOSE_WKKO_WRAP_KEY = 5 % Int32
const LWSCOSE_WKKO_UNWRAP_KEY = 6 % Int32
const LWSCOSE_WKKO_DERIVE_KEY = 7 % Int32
const LWSCOSE_WKKO_DERIVE_BITS = 8 % Int32
const LWSCOSE_WKKO_MAC_CREATE = 9 % Int32
const LWSCOSE_WKKO_MAC_VERIFY = 10 % Int32
const LWSCOSE_WKAECDSA_ALG_ES256 = -7 % Int32
const LWSCOSE_WKAECDSA_ALG_ES384 = -35 % Int32
const LWSCOSE_WKAECDSA_ALG_ES512 = -36 % Int32
const LWSCOSE_WKAEDDSA_ALG_EDDSA = -8 % Int32
const LWSCOSE_WKAHMAC_256_64 = 4 % Int32
const LWSCOSE_WKAHMAC_256_256 = 5 % Int32
const LWSCOSE_WKAHMAC_384_384 = 6 % Int32
const LWSCOSE_WKAHMAC_512_512 = 7 % Int32
const LWSCOSE_WKAAES_128_64 = 14 % Int32
const LWSCOSE_WKAAES_256_64 = 15 % Int32
const LWSCOSE_WKAAES_128_128 = 25 % Int32
const LWSCOSE_WKAAES_256_128 = 26 % Int32
const LWSCOSE_WKAAESGCM_128 = 1 % Int32
const LWSCOSE_WKAAESGCM_192 = 2 % Int32
const LWSCOSE_WKAAESGCM_256 = 3 % Int32
const LWSCOSE_WKAAESCCM_16_64_128 = 10 % Int32
const LWSCOSE_WKAAESCCM_16_64_256 = 11 % Int32
const LWSCOSE_WKAAESCCM_64_64_128 = 12 % Int32
const LWSCOSE_WKAAESCCM_64_64_256 = 13 % Int32
const LWSCOSE_WKAAESCCM_16_128_128 = 14 % Int32
const LWSCOSE_WKAAESCCM_16_128_256 = 15 % Int32
const LWSCOSE_WKAAESCCM_64_128_128 = 16 % Int32
const LWSCOSE_WKAAESCCM_64_128_256 = 17 % Int32
const LWSCOSE_WKACHACHA_POLY1305 = 24 % Int32
const LWSCOSE_WKAPHKDF_SALT = -20 % Int32
const LWSCOSE_WKAPCTX_PARTY_U_IDENTITY = -21 % Int32
const LWSCOSE_WKAPCTX_PARTY_U_NONCE = -22 % Int32
const LWSCOSE_WKAPCTX_PARTY_U_OTHER = -23 % Int32
const LWSCOSE_WKAPCTX_PARTY_V_IDENTITY = -24 % Int32
const LWSCOSE_WKAPCTX_PARTY_V_NONCE = -25 % Int32
const LWSCOSE_WKAPCTX_PARTY_V_OTHER = -26 % Int32
const LWSCOSE_WKK_DIRECT_CEK = -6 % Int32
const LWSCOSE_WKK_DIRECT_HKDF_SHA_256 = -10 % Int32
const LWSCOSE_WKK_DIRECT_HKDF_SHA_512 = -11 % Int32
const LWSCOSE_WKK_DIRECT_HKDF_AES_128 = -12 % Int32
const LWSCOSE_WKK_DIRECT_HKDF_AES_256 = -13 % Int32
const LWSCOSE_WKK_DIRECT_HKDFKW_SHA_256 = -3 % Int32
const LWSCOSE_WKK_DIRECT_HKDFKW_SHA_512 = -4 % Int32
const LWSCOSE_WKK_DIRECT_HKDFKW_AES_128 = -5 % Int32
const LWSCOSE_WKAECDH_ALG_ES_HKDF_256 = -25 % Int32
const LWSCOSE_WKAECDH_ALG_ES_HKDF_512 = -26 % Int32
const LWSCOSE_WKAECDH_ALG_SS_HKDF_256 = -27 % Int32
const LWSCOSE_WKAECDH_ALG_SS_HKDF_512 = -28 % Int32
const LWSCOSE_WKAPECDH_EPHEMERAL_KEY = -1 % Int32
const LWSCOSE_WKAPECDH_STATIC_KEY = -2 % Int32
const LWSCOSE_WKAPECDH_STATIC_KEY_ID = -3 % Int32
const LWSCOSE_WKAPECDH_ES_A128KW = -29 % Int32
const LWSCOSE_WKAPECDH_ES_A192KW = -30 % Int32
const LWSCOSE_WKAPECDH_ES_A256KW = -31 % Int32
const LWSCOSE_WKAPECDH_SS_A128KW = -32 % Int32
const LWSCOSE_WKAPECDH_SS_A192KW = -33 % Int32
const LWSCOSE_WKAPECDH_SS_A256KW = -34 % Int32
const LWSCOSE_WKKTV_OKP = 1 % Int32
const LWSCOSE_WKKTV_EC2 = 2 % Int32
const LWSCOSE_WKKTV_RSA = 3 % Int32
const LWSCOSE_WKKTV_SYMMETRIC = 4 % Int32
const LWSCOSE_WKKTV_HSS_LMS = 5 % Int32
const LWSCOSE_WKKTV_WALNUTDSA = 6 % Int32
const LWSCOSE_WKEC_P256 = 1 % Int32
const LWSCOSE_WKEC_P384 = 2 % Int32
const LWSCOSE_WKEC_P521 = 3 % Int32
const LWSCOSE_WKEC_X25519 = 4 % Int32
const LWSCOSE_WKEC_X448 = 5 % Int32
const LWSCOSE_WKEC_ED25519 = 6 % Int32
const LWSCOSE_WKEC_ED448 = 7 % Int32
const LWSCOSE_WKEC_SECP256K1 = 8 % Int32
const LWSCOSE_WKECKP_CRV = -1 % Int32
const LWSCOSE_WKECKP_X = -2 % Int32
const LWSCOSE_WKECKP_Y = -3 % Int32
const LWSCOSE_WKECKP_D = -4 % Int32
const LWSCOSE_WKOKP_CRV = -1 % Int32
const LWSCOSE_WKOKP_X = -2 % Int32
const LWSCOSE_WKOKP_D = -4 % Int32
const LWSCOSE_WKKPRSA_N = -1 % Int32
const LWSCOSE_WKKPRSA_E = -2 % Int32
const LWSCOSE_WKKPRSA_D = -3 % Int32
const LWSCOSE_WKKPRSA_P = -4 % Int32
const LWSCOSE_WKKPRSA_Q = -5 % Int32
const LWSCOSE_WKKPRSA_DP = -6 % Int32
const LWSCOSE_WKKPRSA_DQ = -7 % Int32
const LWSCOSE_WKKPRSA_QINV = -8 % Int32
const LWSCOSE_WKKPRSA_OTHER = -9 % Int32
const LWSCOSE_WKKPRSA_RI = -10 % Int32
const LWSCOSE_WKKPRSA_DI = -11 % Int32
const LWSCOSE_WKKPRSA_TI = -12 % Int32
const LWSCOSE_WKSYMKP_KEY_VALUE = 4 % Int32
const LWSCOAP_CONTENTFORMAT_COSE_SIGN = 98 % Int32
const LWSCOAP_CONTENTFORMAT_COSE_SIGN1 = 18 % Int32
const LWSCOAP_CONTENTFORMAT_COSE_ENCRYPT = 96 % Int32
const LWSCOAP_CONTENTFORMAT_COSE_ENCRYPT0 = 16 % Int32
const LWSCOAP_CONTENTFORMAT_COSE_MAC = 97 % Int32
const LWSCOAP_CONTENTFORMAT_COSE_MAC0 = 17 % Int32
const LWSCOAP_CONTENTFORMAT_COSE_KEY = 101 % Int32
const LWSCOAP_CONTENTFORMAT_COSE_KEY_SET = 102 % Int32
const LWSCOSE_WKL_COUNTERSIGNATURE0 = 9 % Int32
const LWSCOSE_WKARSA_ALG_RS256 = -257 % Int32
const LWSCOSE_WKARSA_ALG_RS384 = -258 % Int32
const LWSCOSE_WKARSA_ALG_RS512 = -259 % Int32

const enum_cose_key_meta_tok = UInt32
const COSEKEY_META_KTY = 0 % UInt32
const COSEKEY_META_KID = 1 % UInt32
const COSEKEY_META_KEY_OPS = 2 % UInt32
const COSEKEY_META_BASE_IV = 3 % UInt32
const COSEKEY_META_ALG = 4 % UInt32
const LWS_COUNT_COSE_KEY_ELEMENTS = 5 % UInt32

const cose_param_t = Int64

function lws_cose_alg_to_name(alg)
    @ccall libwebsockets.lws_cose_alg_to_name(alg::cose_param_t)::Ptr{Cchar}
end

function lws_cose_name_to_alg(name)
    @ccall libwebsockets.lws_cose_name_to_alg(name::Ptr{Cchar})::cose_param_t
end

struct lws_cose_key
    e::NTuple{12, lws_gencrypto_keyelem}
    meta::NTuple{5, lws_gencrypto_keyelem}
    list::lws_dll2_t
    gencrypto_kty::Cint
    kty::cose_param_t
    cose_alg::cose_param_t
    cose_curve::cose_param_t
    private_key::Cchar
end

const lws_cose_key_t = lws_cose_key

# typedef int ( * lws_cose_key_import_callback ) ( struct lws_cose_key * s , void * user )
const lws_cose_key_import_callback = Ptr{Cvoid}

function lws_cose_key_import(pkey_set, cb, user, in, len)
    @ccall libwebsockets.lws_cose_key_import(pkey_set::Ptr{lws_dll2_owner_t}, cb::lws_cose_key_import_callback, user::Ptr{Cvoid}, in::Ptr{UInt8}, len::Csize_t)::Ptr{lws_cose_key_t}
end

function lws_cose_key_export(ck, ctx, flags)
    @ccall libwebsockets.lws_cose_key_export(ck::Ptr{lws_cose_key_t}, ctx::Ptr{lws_lec_pctx_t}, flags::Cint)::lws_lec_pctx_ret
end

function lws_cose_key_generate(context, cose_kty, use_mask, bits, curve, kid, kl)
    @ccall libwebsockets.lws_cose_key_generate(context::Ptr{lws_context}, cose_kty::cose_param_t, use_mask::Cint, bits::Cint, curve::Ptr{Cchar}, kid::Ptr{UInt8}, kl::Csize_t)::Ptr{lws_cose_key_t}
end

function lws_cose_key_from_set(set, kid, kl)
    @ccall libwebsockets.lws_cose_key_from_set(set::Ptr{lws_dll2_owner_t}, kid::Ptr{UInt8}, kl::Csize_t)::Ptr{lws_cose_key_t}
end

function lws_cose_key_destroy(ck)
    @ccall libwebsockets.lws_cose_key_destroy(ck::Ptr{Ptr{lws_cose_key_t}})::Cvoid
end

function lws_cose_key_set_destroy(o)
    @ccall libwebsockets.lws_cose_key_set_destroy(o::Ptr{lws_dll2_owner_t})::Cvoid
end

function lws_cose_key_dump(ck)
    @ccall libwebsockets.lws_cose_key_dump(ck::Ptr{lws_cose_key_t})::Cvoid
end

const lws_cose_sig_types = UInt32
const SIGTYPE_UNKNOWN = 0 % UInt32
const SIGTYPE_MULTI = 1 % UInt32
const SIGTYPE_SINGLE = 2 % UInt32
const SIGTYPE_COUNTERSIGNED = 3 % UInt32
const SIGTYPE_MAC = 4 % UInt32
const SIGTYPE_MAC0 = 5 % UInt32

struct lws_cose_validate_res_t
    list::lws_dll2_t
    cose_key::Ptr{lws_cose_key_t}
    cose_alg::cose_param_t
    result::Cint
end

const __JL_Ctag_25 = Int32
const LCOSESIGEXTCB_RET_FINISHED = 0 % Int32
const LCOSESIGEXTCB_RET_AGAIN = 1 % Int32
const LCOSESIGEXTCB_RET_ERROR = -1 % Int32

mutable struct lws_cose_validate_context end

struct lws_cose_sig_ext_pay_t
    cps::Ptr{lws_cose_validate_context}
    ext::Ptr{UInt8}
    xl::Csize_t
end

# typedef int ( * lws_cose_sign_ext_pay_cb_t ) ( lws_cose_sig_ext_pay_t * x )
const lws_cose_sign_ext_pay_cb_t = Ptr{Cvoid}

# typedef int ( * lws_cose_validate_pay_cb_t ) ( struct lws_cose_validate_context * cps , void * opaque , const uint8_t * paychunk , size_t paychunk_len )
const lws_cose_validate_pay_cb_t = Ptr{Cvoid}

struct lws_cose_validate_create_info
    cx::Ptr{lws_context}
    keyset::Ptr{lws_dll2_owner_t}
    sigtype::lws_cose_sig_types
    pay_cb::lws_cose_validate_pay_cb_t
    pay_opaque::Ptr{Cvoid}
    ext_cb::lws_cose_sign_ext_pay_cb_t
    ext_opaque::Ptr{Cvoid}
    ext_len::Csize_t
end

const lws_cose_validate_create_info_t = lws_cose_validate_create_info

function lws_cose_validate_create(info)
    @ccall libwebsockets.lws_cose_validate_create(info::Ptr{lws_cose_validate_create_info_t})::Ptr{lws_cose_validate_context}
end

function lws_cose_validate_chunk(cps, in, in_len, used_in)
    @ccall libwebsockets.lws_cose_validate_chunk(cps::Ptr{lws_cose_validate_context}, in::Ptr{UInt8}, in_len::Csize_t, used_in::Ptr{Csize_t})::Cint
end

function lws_cose_validate_results(cps)
    @ccall libwebsockets.lws_cose_validate_results(cps::Ptr{lws_cose_validate_context})::Ptr{lws_dll2_owner_t}
end

function lws_cose_validate_destroy(cps)
    @ccall libwebsockets.lws_cose_validate_destroy(cps::Ptr{Ptr{lws_cose_validate_context}})::Cvoid
end

mutable struct lws_cose_sign_context end

struct lws_cose_sign_create_info
    cx::Ptr{lws_context}
    keyset::Ptr{lws_dll2_owner_t}
    lec::Ptr{lws_lec_pctx_t}
    ext_cb::lws_cose_sign_ext_pay_cb_t
    ext_opaque::Ptr{Cvoid}
    ext_len::Csize_t
    inline_payload_len::Csize_t
    flags::Cint
    sigtype::lws_cose_sig_types
end

const lws_cose_sign_create_info_t = lws_cose_sign_create_info

function lws_cose_sign_create(info)
    @ccall libwebsockets.lws_cose_sign_create(info::Ptr{lws_cose_sign_create_info_t})::Ptr{lws_cose_sign_context}
end

function lws_cose_sign_add(csc, alg, ck)
    @ccall libwebsockets.lws_cose_sign_add(csc::Ptr{lws_cose_sign_context}, alg::cose_param_t, ck::Ptr{lws_cose_key_t})::Cint
end

function lws_cose_sign_payload_chunk(csc, in, in_len)
    @ccall libwebsockets.lws_cose_sign_payload_chunk(csc::Ptr{lws_cose_sign_context}, in::Ptr{UInt8}, in_len::Csize_t)::lws_lec_pctx_ret
end

function lws_cose_sign_destroy(csc)
    @ccall libwebsockets.lws_cose_sign_destroy(csc::Ptr{Ptr{lws_cose_sign_context}})::Cvoid
end

const lws_struct_map_type_eum = UInt32
const LSMT_SIGNED = 0 % UInt32
const LSMT_UNSIGNED = 1 % UInt32
const LSMT_BOOLEAN = 2 % UInt32
const LSMT_STRING_CHAR_ARRAY = 3 % UInt32
const LSMT_STRING_PTR = 4 % UInt32
const LSMT_LIST = 5 % UInt32
const LSMT_CHILD_PTR = 6 % UInt32
const LSMT_SCHEMA = 7 % UInt32
const LSMT_BLOB_PTR = 8 % UInt32

struct lejp_collation
    chunks::lws_dll2
    len::Cint
    buf::NTuple{255, Cchar}
end

const lejp_collation_t = lejp_collation

struct lws_struct_map
    colname::Ptr{Cchar}
    child_map::Ptr{lws_struct_map}
    lejp_cb::lejp_callback
    ofs::Csize_t
    aux::Csize_t
    ofs_clist::Csize_t
    child_map_size::Csize_t
    type::lws_struct_map_type_eum
end

const lws_struct_map_t = lws_struct_map

# typedef int ( * lws_struct_args_cb ) ( void * obj , void * cb_arg )
const lws_struct_args_cb = Ptr{Cvoid}

struct lws_struct_args
    map_st::NTuple{5, Ptr{lws_struct_map_t}}
    cb::lws_struct_args_cb
    ac::Ptr{lwsac}
    cb_arg::Ptr{Cvoid}
    dest::Ptr{Cvoid}
    dest_len::Csize_t
    toplevel_dll2_ofs::Csize_t
    map_entries_st::NTuple{5, Csize_t}
    ac_block_size::Csize_t
    subtype::Cint
    top_schema_index::Cint
    ac_chunks::Ptr{lwsac}
    chunks_owner::lws_dll2_owner
    chunks_length::Csize_t
end

const lws_struct_args_t = lws_struct_args

struct lws_struct_serialize_st
    dllpos::Ptr{lws_dll2}
    map::Ptr{lws_struct_map_t}
    obj::Ptr{Cchar}
    map_entries::Csize_t
    map_entry::Csize_t
    size::Csize_t
    subsequent::Cchar
    idt::Cchar
end

const lws_struct_serialize_st_t = lws_struct_serialize_st

const __JL_Ctag_28 = UInt32
const LSSERJ_FLAG_PRETTY = 1 % UInt32
const LSSERJ_FLAG_OMIT_SCHEMA = 2 % UInt32

struct lws_struct_serialize
    st::NTuple{5, lws_struct_serialize_st_t}
    offset::Csize_t
    remaining::Csize_t
    sp::Cint
    flags::Cint
end

const lws_struct_serialize_t = lws_struct_serialize

const lws_struct_json_serialize_result_t = UInt32
const LSJS_RESULT_CONTINUE = 0 % UInt32
const LSJS_RESULT_FINISH = 1 % UInt32
const LSJS_RESULT_ERROR = 2 % UInt32

function lws_struct_json_init_parse(ctx, cb, user)
    @ccall libwebsockets.lws_struct_json_init_parse(ctx::Ptr{lejp_ctx}, cb::lejp_callback, user::Ptr{Cvoid})::Cint
end

function lws_struct_schema_only_lejp_cb(ctx, reason)
    @ccall libwebsockets.lws_struct_schema_only_lejp_cb(ctx::Ptr{lejp_ctx}, reason::Cchar)::Int8
end

function lws_struct_default_lejp_cb(ctx, reason)
    @ccall libwebsockets.lws_struct_default_lejp_cb(ctx::Ptr{lejp_ctx}, reason::Cchar)::Int8
end

function lws_struct_json_serialize_create(map, map_entries, flags, ptoplevel)
    @ccall libwebsockets.lws_struct_json_serialize_create(map::Ptr{lws_struct_map_t}, map_entries::Csize_t, flags::Cint, ptoplevel::Ptr{Cvoid})::Ptr{lws_struct_serialize_t}
end

function lws_struct_json_serialize_destroy(pjs)
    @ccall libwebsockets.lws_struct_json_serialize_destroy(pjs::Ptr{Ptr{lws_struct_serialize_t}})::Cvoid
end

function lws_struct_json_serialize(js, buf, len, written)
    @ccall libwebsockets.lws_struct_json_serialize(js::Ptr{lws_struct_serialize_t}, buf::Ptr{UInt8}, len::Csize_t, written::Ptr{Csize_t})::lws_struct_json_serialize_result_t
end

mutable struct sqlite3 end

function lws_struct_sq3_serialize(pdb, schema, owner, manual_idx)
    @ccall libwebsockets.lws_struct_sq3_serialize(pdb::Ptr{sqlite3}, schema::Ptr{lws_struct_map_t}, owner::Ptr{lws_dll2_owner_t}, manual_idx::UInt32)::Cint
end

function lws_struct_sq3_deserialize(pdb, filter, order, schema, o, ac, start, limit)
    @ccall libwebsockets.lws_struct_sq3_deserialize(pdb::Ptr{sqlite3}, filter::Ptr{Cchar}, order::Ptr{Cchar}, schema::Ptr{lws_struct_map_t}, o::Ptr{lws_dll2_owner_t}, ac::Ptr{Ptr{lwsac}}, start::Cint, limit::Cint)::Cint
end

function lws_struct_sq3_create_table(pdb, schema)
    @ccall libwebsockets.lws_struct_sq3_create_table(pdb::Ptr{sqlite3}, schema::Ptr{lws_struct_map_t})::Cint
end

function lws_struct_sq3_open(context, sqlite3_path, create_if_missing, pdb)
    @ccall libwebsockets.lws_struct_sq3_open(context::Ptr{lws_context}, sqlite3_path::Ptr{Cchar}, create_if_missing::Cchar, pdb::Ptr{Ptr{sqlite3}})::Cint
end

function lws_struct_sq3_close(pdb)
    @ccall libwebsockets.lws_struct_sq3_close(pdb::Ptr{Ptr{sqlite3}})::Cint
end

mutable struct lws_threadpool end

mutable struct lws_threadpool_task end

const lws_threadpool_task_status = UInt32
const LWS_TP_STATUS_QUEUED = 0 % UInt32
const LWS_TP_STATUS_RUNNING = 1 % UInt32
const LWS_TP_STATUS_SYNCING = 2 % UInt32
const LWS_TP_STATUS_STOPPING = 3 % UInt32
const LWS_TP_STATUS_FINISHED = 4 % UInt32
const LWS_TP_STATUS_STOPPED = 5 % UInt32

const lws_threadpool_task_return = UInt32
const LWS_TP_RETURN_CHECKING_IN = 0 % UInt32
const LWS_TP_RETURN_SYNC = 1 % UInt32
const LWS_TP_RETURN_FINISHED = 2 % UInt32
const LWS_TP_RETURN_STOPPED = 3 % UInt32
const LWS_TP_RETURN_FLAG_OUTLIVE = 64 % UInt32

struct lws_threadpool_create_args
    threads::Cint
    max_queue_depth::Cint
end

struct lws_threadpool_task_args
    wsi::Ptr{lws}
    user::Ptr{Cvoid}
    name::Ptr{Cchar}
    async_task::Cchar
    task::Ptr{Cvoid}
    cleanup::Ptr{Cvoid}
end

# automatic type deduction for variadic arguments may not be what you want, please use with caution
@generated function lws_threadpool_create(context, args, format, va_list...)
        :(@ccall(libwebsockets.lws_threadpool_create(context::Ptr{lws_context}, args::Ptr{lws_threadpool_create_args}, format::Ptr{Cchar}; $(to_c_type_pairs(va_list)...))::Ptr{lws_threadpool}))
    end

function lws_threadpool_finish(tp)
    @ccall libwebsockets.lws_threadpool_finish(tp::Ptr{lws_threadpool})::Cvoid
end

function lws_threadpool_destroy(tp)
    @ccall libwebsockets.lws_threadpool_destroy(tp::Ptr{lws_threadpool})::Cvoid
end

# automatic type deduction for variadic arguments may not be what you want, please use with caution
@generated function lws_threadpool_enqueue(tp, args, format, va_list...)
        :(@ccall(libwebsockets.lws_threadpool_enqueue(tp::Ptr{lws_threadpool}, args::Ptr{lws_threadpool_task_args}, format::Ptr{Cchar}; $(to_c_type_pairs(va_list)...))::Ptr{lws_threadpool_task}))
    end

function lws_threadpool_dequeue(wsi)
    @ccall libwebsockets.lws_threadpool_dequeue(wsi::Ptr{lws})::Cint
end

function lws_threadpool_dequeue_task(task)
    @ccall libwebsockets.lws_threadpool_dequeue_task(task::Ptr{lws_threadpool_task})::Cint
end

function lws_threadpool_task_status_wsi(wsi, task, user)
    @ccall libwebsockets.lws_threadpool_task_status_wsi(wsi::Ptr{lws}, task::Ptr{Ptr{lws_threadpool_task}}, user::Ptr{Ptr{Cvoid}})::lws_threadpool_task_status
end

function lws_threadpool_task_status(task, user)
    @ccall libwebsockets.lws_threadpool_task_status(task::Ptr{lws_threadpool_task}, user::Ptr{Ptr{Cvoid}})::lws_threadpool_task_status
end

function lws_threadpool_task_status_noreap(task)
    @ccall libwebsockets.lws_threadpool_task_status_noreap(task::Ptr{lws_threadpool_task})::lws_threadpool_task_status
end

function lws_threadpool_task_sync(task, stop)
    @ccall libwebsockets.lws_threadpool_task_sync(task::Ptr{lws_threadpool_task}, stop::Cint)::Cvoid
end

function lws_threadpool_dump(tp)
    @ccall libwebsockets.lws_threadpool_dump(tp::Ptr{lws_threadpool})::Cvoid
end

function lws_threadpool_get_task_wsi(wsi)
    @ccall libwebsockets.lws_threadpool_get_task_wsi(wsi::Ptr{lws})::Ptr{lws_threadpool_task}
end

function lws_threadpool_foreach_task_wsi(wsi, user, cb)
    @ccall libwebsockets.lws_threadpool_foreach_task_wsi(wsi::Ptr{lws}, user::Ptr{Cvoid}, cb::Ptr{Cvoid})::Cint
end

const lws_tokenize_elem = Int32
const LWS_TOKZE_ERRS = 5 % Int32
const LWS_TOKZE_ERR_BROKEN_UTF8 = -5 % Int32
const LWS_TOKZE_ERR_UNTERM_STRING = -4 % Int32
const LWS_TOKZE_ERR_MALFORMED_FLOAT = -3 % Int32
const LWS_TOKZE_ERR_NUM_ON_LHS = -2 % Int32
const LWS_TOKZE_ERR_COMMA_LIST = -1 % Int32
const LWS_TOKZE_ENDED = 0 % Int32
const LWS_TOKZE_DELIMITER = 1 % Int32
const LWS_TOKZE_TOKEN = 2 % Int32
const LWS_TOKZE_INTEGER = 3 % Int32
const LWS_TOKZE_FLOAT = 4 % Int32
const LWS_TOKZE_TOKEN_NAME_EQUALS = 5 % Int32
const LWS_TOKZE_TOKEN_NAME_COLON = 6 % Int32
const LWS_TOKZE_QUOTED_STRING = 7 % Int32

const lws_tokenize_delimiter_tracking = UInt32
const LWSTZ_DT_NEED_FIRST_CONTENT = 0 % UInt32
const LWSTZ_DT_NEED_DELIM = 1 % UInt32
const LWSTZ_DT_NEED_NEXT_CONTENT = 2 % UInt32

struct lws_tokenize
    start::Ptr{Cchar}
    token::Ptr{Cchar}
    len::Csize_t
    token_len::Csize_t
    flags::UInt16
    delim::UInt8
    e::Int8
end

const lws_tokenize_t = lws_tokenize

function lws_tokenize_init(ts, start, flags)
    @ccall libwebsockets.lws_tokenize_init(ts::Ptr{lws_tokenize}, start::Ptr{Cchar}, flags::Cint)::Cvoid
end

function lws_tokenize(ts)
    @ccall libwebsockets.lws_tokenize(ts::Ptr{lws_tokenize})::lws_tokenize_elem
end

function lws_tokenize_cstr(ts, str, max)
    @ccall libwebsockets.lws_tokenize_cstr(ts::Ptr{lws_tokenize}, str::Ptr{Cchar}, max::Csize_t)::Cint
end

# typedef int ( * lws_strexp_expand_cb ) ( void * priv , const char * name , char * out , size_t * pos , size_t olen , size_t * exp_ofs )
const lws_strexp_expand_cb = Ptr{Cvoid}

struct lws_strexp
    name::NTuple{32, Cchar}
    cb::lws_strexp_expand_cb
    priv::Ptr{Cvoid}
    out::Ptr{Cchar}
    olen::Csize_t
    pos::Csize_t
    exp_ofs::Csize_t
    name_pos::UInt8
    state::Cchar
end

const lws_strexp_t = lws_strexp

const __JL_Ctag_31 = Int32
const LSTRX_DONE = 0 % Int32
const LSTRX_FILLED_OUT = 1 % Int32
const LSTRX_FATAL_NAME_TOO_LONG = -1 % Int32
const LSTRX_FATAL_NAME_UNKNOWN = -2 % Int32

function lws_strexp_init(exp, priv, cb, out, olen)
    @ccall libwebsockets.lws_strexp_init(exp::Ptr{lws_strexp_t}, priv::Ptr{Cvoid}, cb::lws_strexp_expand_cb, out::Ptr{Cchar}, olen::Csize_t)::Cvoid
end

function lws_strexp_reset_out(exp, out, olen)
    @ccall libwebsockets.lws_strexp_reset_out(exp::Ptr{lws_strexp_t}, out::Ptr{Cchar}, olen::Csize_t)::Cvoid
end

function lws_strexp_expand(exp, in, len, pused_in, pused_out)
    @ccall libwebsockets.lws_strexp_expand(exp::Ptr{lws_strexp_t}, in::Ptr{Cchar}, len::Csize_t, pused_in::Ptr{Csize_t}, pused_out::Ptr{Csize_t})::Cint
end

function lws_strcmp_wildcard(wildcard, wlen, check, clen)
    @ccall libwebsockets.lws_strcmp_wildcard(wildcard::Ptr{Cchar}, wlen::Csize_t, check::Ptr{Cchar}, clen::Csize_t)::Cint
end

const lwsac_cached_file_t = Ptr{Cuchar}

const lws_list_ptr = Ptr{Cvoid}

# typedef int ( * lws_list_ptr_sort_func_t ) ( lws_list_ptr a , lws_list_ptr b )
const lws_list_ptr_sort_func_t = Ptr{Cvoid}

function lws_list_ptr_insert(phead, add, sort)
    @ccall libwebsockets.lws_list_ptr_insert(phead::Ptr{lws_list_ptr}, add::Ptr{lws_list_ptr}, sort::lws_list_ptr_sort_func_t)::Cvoid
end

function lwsac_use(head, ensure, chunk_size)
    @ccall libwebsockets.lwsac_use(head::Ptr{Ptr{lwsac}}, ensure::Csize_t, chunk_size::Csize_t)::Ptr{Cvoid}
end

function lwsac_use_backfill(head, ensure, chunk_size)
    @ccall libwebsockets.lwsac_use_backfill(head::Ptr{Ptr{lwsac}}, ensure::Csize_t, chunk_size::Csize_t)::Ptr{Cvoid}
end

function lwsac_free(head)
    @ccall libwebsockets.lwsac_free(head::Ptr{Ptr{lwsac}})::Cvoid
end

function lwsac_detach(head)
    @ccall libwebsockets.lwsac_detach(head::Ptr{Ptr{lwsac}})::Cvoid
end

function lwsac_reference(head)
    @ccall libwebsockets.lwsac_reference(head::Ptr{lwsac})::Cvoid
end

function lwsac_unreference(head)
    @ccall libwebsockets.lwsac_unreference(head::Ptr{Ptr{lwsac}})::Cvoid
end

function lwsac_extend(head, amount)
    @ccall libwebsockets.lwsac_extend(head::Ptr{lwsac}, amount::Csize_t)::Cint
end

function lwsac_use_cached_file_start(cache)
    @ccall libwebsockets.lwsac_use_cached_file_start(cache::lwsac_cached_file_t)::Cvoid
end

function lwsac_use_cached_file_end(cache)
    @ccall libwebsockets.lwsac_use_cached_file_end(cache::Ptr{lwsac_cached_file_t})::Cvoid
end

function lwsac_use_cached_file_detach(cache)
    @ccall libwebsockets.lwsac_use_cached_file_detach(cache::Ptr{lwsac_cached_file_t})::Cvoid
end

function lwsac_cached_file(filepath, cache, len)
    @ccall libwebsockets.lwsac_cached_file(filepath::Ptr{Cchar}, cache::Ptr{lwsac_cached_file_t}, len::Ptr{Csize_t})::Cint
end

function lwsac_sizeof(first)
    @ccall libwebsockets.lwsac_sizeof(first::Cint)::Csize_t
end

function lwsac_get_tail_pos(lac)
    @ccall libwebsockets.lwsac_get_tail_pos(lac::Ptr{lwsac})::Csize_t
end

function lwsac_get_next(lac)
    @ccall libwebsockets.lwsac_get_next(lac::Ptr{lwsac})::Ptr{lwsac}
end

function lwsac_align(length)
    @ccall libwebsockets.lwsac_align(length::Csize_t)::Csize_t
end

function lwsac_info(head)
    @ccall libwebsockets.lwsac_info(head::Ptr{lwsac})::Cvoid
end

function lwsac_total_alloc(head)
    @ccall libwebsockets.lwsac_total_alloc(head::Ptr{lwsac})::UInt64
end

function lwsac_total_overhead(head)
    @ccall libwebsockets.lwsac_total_overhead(head::Ptr{lwsac})::UInt64
end

function lwsac_scan_extant(head, find, len, nul)
    @ccall libwebsockets.lwsac_scan_extant(head::Ptr{lwsac}, find::Ptr{UInt8}, len::Csize_t, nul::Cint)::Ptr{UInt8}
end

mutable struct lws_fts end

mutable struct lws_fts_file end

struct lws_fts_result_filepath
    next::Ptr{lws_fts_result_filepath}
    matches::Cint
    matches_length::Cint
    lines_in_file::Cint
    filepath_length::Cint
end

struct lws_fts_result_autocomplete
    next::Ptr{lws_fts_result_autocomplete}
    instances::Cint
    agg_instances::Cint
    ac_length::Cint
    elided::Cchar
    has_children::Cchar
end

struct lws_fts_result
    filepath_head::Ptr{lws_fts_result_filepath}
    autocomplete_head::Ptr{lws_fts_result_autocomplete}
    duration_ms::Cint
    effective_flags::Cint
end

function lws_fts_create(fd)
    @ccall libwebsockets.lws_fts_create(fd::Cint)::Ptr{lws_fts}
end

function lws_fts_destroy(trie)
    @ccall libwebsockets.lws_fts_destroy(trie::Ptr{Ptr{lws_fts}})::Cvoid
end

function lws_fts_file_index(t, filepath, filepath_len, priority)
    @ccall libwebsockets.lws_fts_file_index(t::Ptr{lws_fts}, filepath::Ptr{Cchar}, filepath_len::Cint, priority::Cint)::Cint
end

function lws_fts_fill(t, file_index, buf, len)
    @ccall libwebsockets.lws_fts_fill(t::Ptr{lws_fts}, file_index::UInt32, buf::Ptr{Cchar}, len::Csize_t)::Cint
end

function lws_fts_serialize(t)
    @ccall libwebsockets.lws_fts_serialize(t::Ptr{lws_fts})::Cint
end

function lws_fts_open(filepath)
    @ccall libwebsockets.lws_fts_open(filepath::Ptr{Cchar})::Ptr{lws_fts_file}
end

struct lws_fts_search_params
    needle::Ptr{Cchar}
    only_filepath::Ptr{Cchar}
    results_head::Ptr{lwsac}
    flags::Cint
    max_autocomplete::Cint
    max_files::Cint
    max_lines::Cint
end

function lws_fts_search(jtf, ftsp)
    @ccall libwebsockets.lws_fts_search(jtf::Ptr{lws_fts_file}, ftsp::Ptr{lws_fts_search_params})::Ptr{lws_fts_result}
end

function lws_fts_close(jtf)
    @ccall libwebsockets.lws_fts_close(jtf::Ptr{lws_fts_file})::Cvoid
end

mutable struct lws_diskcache_scan end

function lws_diskcache_create(cache_dir_base, cache_size_limit)
    @ccall libwebsockets.lws_diskcache_create(cache_dir_base::Ptr{Cchar}, cache_size_limit::UInt64)::Ptr{lws_diskcache_scan}
end

function lws_diskcache_destroy(lds)
    @ccall libwebsockets.lws_diskcache_destroy(lds::Ptr{Ptr{lws_diskcache_scan}})::Cvoid
end

function lws_diskcache_prepare(cache_base_dir, mode, uid)
    @ccall libwebsockets.lws_diskcache_prepare(cache_base_dir::Ptr{Cchar}, mode::Cint, uid::uid_t)::Cint
end

function lws_diskcache_query(lds, is_bot, hash_hex, _fd, cache, cache_len, extant_cache_len)
    @ccall libwebsockets.lws_diskcache_query(lds::Ptr{lws_diskcache_scan}, is_bot::Cint, hash_hex::Ptr{Cchar}, _fd::Ptr{Cint}, cache::Ptr{Cchar}, cache_len::Cint, extant_cache_len::Ptr{Csize_t})::Cint
end

function lws_diskcache_finalize_name(cache)
    @ccall libwebsockets.lws_diskcache_finalize_name(cache::Ptr{Cchar})::Cint
end

function lws_diskcache_trim(lds)
    @ccall libwebsockets.lws_diskcache_trim(lds::Ptr{lws_diskcache_scan})::Cint
end

function lws_diskcache_secs_to_idle(lds)
    @ccall libwebsockets.lws_diskcache_secs_to_idle(lds::Ptr{lws_diskcache_scan})::Cint
end

const lws_seq_events_t = UInt32
const LWSSEQ_CREATED = 0 % UInt32
const LWSSEQ_DESTROYED = 1 % UInt32
const LWSSEQ_TIMED_OUT = 2 % UInt32
const LWSSEQ_HEARTBEAT = 3 % UInt32
const LWSSEQ_WSI_CONNECTED = 4 % UInt32
const LWSSEQ_WSI_CONN_FAIL = 5 % UInt32
const LWSSEQ_WSI_CONN_CLOSE = 6 % UInt32
const LWSSEQ_SS_STATE_BASE = 7 % UInt32
const LWSSEQ_USER_BASE = 100 % UInt32

const lws_seq_cb_return = UInt32
const LWSSEQ_RET_CONTINUE = 0 % UInt32
const LWSSEQ_RET_DESTROY = 1 % UInt32

const lws_seq_cb_return_t = lws_seq_cb_return

# typedef lws_seq_cb_return_t ( * lws_seq_event_cb ) ( struct lws_sequencer * seq , void * user , int event , void * data , void * aux )
const lws_seq_event_cb = Ptr{Cvoid}

struct lws_seq_info
    data::NTuple{64, UInt8}
end

function Base.getproperty(x::Ptr{lws_seq_info}, f::Symbol)
    f === :context && return Ptr{Ptr{lws_context}}(x + 0)
    f === :tsi && return Ptr{Cint}(x + 8)
    f === :user_size && return Ptr{Csize_t}(x + 16)
    f === :puser && return Ptr{Ptr{Ptr{Cvoid}}}(x + 24)
    f === :cb && return Ptr{lws_seq_event_cb}(x + 32)
    f === :name && return Ptr{Ptr{Cchar}}(x + 40)
    f === :retry && return Ptr{Ptr{lws_retry_bo_t}}(x + 48)
    f === :wakesuspend && return (Ptr{UInt8}(x + 56), 0, 1)
    return getfield(x, f)
end

function Base.getproperty(x::lws_seq_info, f::Symbol)
    r = Ref{lws_seq_info}(x)
    ptr = Base.unsafe_convert(Ptr{lws_seq_info}, r)
    fptr = getproperty(ptr, f)
    begin
        if fptr isa Ptr
            return GC.@preserve(r, unsafe_load(fptr))
        else
            (baseptr, offset, width) = fptr
            ty = eltype(baseptr)
            baseptr32 = convert(Ptr{UInt32}, baseptr)
            u64 = GC.@preserve(r, unsafe_load(baseptr32))
            if offset + width > 32
                u64 |= GC.@preserve(r, unsafe_load(baseptr32 + 4)) << 32
            end
            u64 = u64 >> offset & (1 << width - 1)
            return u64 % ty
        end
    end
end

function Base.setproperty!(x::Ptr{lws_seq_info}, f::Symbol, v)
    fptr = getproperty(x, f)
    if fptr isa Ptr
        unsafe_store!(getproperty(x, f), v)
    else
        (baseptr, offset, width) = fptr
        baseptr32 = convert(Ptr{UInt32}, baseptr)
        u64 = unsafe_load(baseptr32)
        straddle = offset + width > 32
        if straddle
            u64 |= unsafe_load(baseptr32 + 4) << 32
        end
        mask = 1 << width - 1
        u64 &= ~(mask << offset)
        u64 |= (unsigned(v) & mask) << offset
        unsafe_store!(baseptr32, u64 & typemax(UInt32))
        if straddle
            unsafe_store!(baseptr32 + 4, u64 >> 32)
        end
    end
end

const lws_seq_info_t = lws_seq_info

function lws_seq_create(info)
    @ccall libwebsockets.lws_seq_create(info::Ptr{lws_seq_info_t})::Ptr{lws_sequencer}
end

function lws_seq_destroy(seq)
    @ccall libwebsockets.lws_seq_destroy(seq::Ptr{Ptr{lws_sequencer}})::Cvoid
end

function lws_seq_queue_event(seq, e, data, aux)
    @ccall libwebsockets.lws_seq_queue_event(seq::Ptr{lws_sequencer}, e::lws_seq_events_t, data::Ptr{Cvoid}, aux::Ptr{Cvoid})::Cint
end

function lws_seq_check_wsi(seq, wsi)
    @ccall libwebsockets.lws_seq_check_wsi(seq::Ptr{lws_sequencer}, wsi::Ptr{lws})::Cint
end

function lws_seq_timeout_us(seq, us)
    @ccall libwebsockets.lws_seq_timeout_us(seq::Ptr{lws_sequencer}, us::lws_usec_t)::Cint
end

function lws_seq_from_user(u)
    @ccall libwebsockets.lws_seq_from_user(u::Ptr{Cvoid})::Ptr{lws_sequencer}
end

function lws_seq_us_since_creation(seq)
    @ccall libwebsockets.lws_seq_us_since_creation(seq::Ptr{lws_sequencer})::lws_usec_t
end

function lws_seq_name(seq)
    @ccall libwebsockets.lws_seq_name(seq::Ptr{lws_sequencer})::Ptr{Cchar}
end

function lws_seq_get_context(seq)
    @ccall libwebsockets.lws_seq_get_context(seq::Ptr{lws_sequencer})::Ptr{lws_context}
end

const lws_ss_tx_ordinal_t = UInt32

const lws_ss_constate_t = UInt32
const LWSSSCS_CREATING = 1 % UInt32
const LWSSSCS_DISCONNECTED = 2 % UInt32
const LWSSSCS_UNREACHABLE = 3 % UInt32
const LWSSSCS_AUTH_FAILED = 4 % UInt32
const LWSSSCS_CONNECTED = 5 % UInt32
const LWSSSCS_CONNECTING = 6 % UInt32
const LWSSSCS_DESTROYING = 7 % UInt32
const LWSSSCS_POLL = 8 % UInt32
const LWSSSCS_ALL_RETRIES_FAILED = 9 % UInt32
const LWSSSCS_QOS_ACK_REMOTE = 10 % UInt32
const LWSSSCS_QOS_NACK_REMOTE = 11 % UInt32
const LWSSSCS_QOS_ACK_LOCAL = 12 % UInt32
const LWSSSCS_QOS_NACK_LOCAL = 13 % UInt32
const LWSSSCS_TIMEOUT = 14 % UInt32
const LWSSSCS_SERVER_TXN = 15 % UInt32
const LWSSSCS_SERVER_UPGRADE = 16 % UInt32
const LWSSSCS_EVENT_WAIT_CANCELLED = 17 % UInt32
const LWSSSCS_UPSTREAM_LINK_RETRY = 18 % UInt32
const LWSSSCS_SINK_JOIN = 19 % UInt32
const LWSSSCS_SINK_PART = 20 % UInt32
const LWSSSCS_USER_BASE = 1000 % UInt32

const __JL_Ctag_34 = UInt32
const LWSSS_FLAG_SOM = 1 % UInt32
const LWSSS_FLAG_EOM = 2 % UInt32
const LWSSS_FLAG_POLL = 4 % UInt32
const LWSSS_FLAG_RELATED_START = 8 % UInt32
const LWSSS_FLAG_RELATED_END = 16 % UInt32
const LWSSS_FLAG_RIDESHARE = 32 % UInt32
const LWSSS_FLAG_PERF_JSON = 64 % UInt32
const LWSSS_SER_RXPRE_RX_PAYLOAD = 85 % UInt32
const LWSSS_SER_RXPRE_CREATE_RESULT = 86 % UInt32
const LWSSS_SER_RXPRE_CONNSTATE = 87 % UInt32
const LWSSS_SER_RXPRE_TXCR_UPDATE = 88 % UInt32
const LWSSS_SER_RXPRE_METADATA = 89 % UInt32
const LWSSS_SER_RXPRE_TLSNEG_ENCLAVE_SIGN = 90 % UInt32
const LWSSS_SER_RXPRE_PERF = 91 % UInt32
const LWSSS_SER_TXPRE_STREAMTYPE = 170 % UInt32
const LWSSS_SER_TXPRE_ONWARD_CONNECT = 171 % UInt32
const LWSSS_SER_TXPRE_DESTROYING = 172 % UInt32
const LWSSS_SER_TXPRE_TX_PAYLOAD = 173 % UInt32
const LWSSS_SER_TXPRE_METADATA = 174 % UInt32
const LWSSS_SER_TXPRE_TXCR_UPDATE = 175 % UInt32
const LWSSS_SER_TXPRE_TIMEOUT_UPDATE = 176 % UInt32
const LWSSS_SER_TXPRE_PAYLOAD_LENGTH_HINT = 177 % UInt32
const LWSSS_SER_TXPRE_TLSNEG_ENCLAVE_SIGNED = 178 % UInt32

const lws_ss_conn_states_t = UInt32
const LPCSPROX_WAIT_INITIAL_TX = 1 % UInt32
const LPCSPROX_REPORTING_FAIL = 2 % UInt32
const LPCSPROX_REPORTING_OK = 3 % UInt32
const LPCSPROX_OPERATIONAL = 4 % UInt32
const LPCSPROX_DESTROYED = 5 % UInt32
const LPCSCLI_SENDING_INITIAL_TX = 6 % UInt32
const LPCSCLI_WAITING_CREATE_RESULT = 7 % UInt32
const LPCSCLI_LOCAL_CONNECTED = 8 % UInt32
const LPCSCLI_ONWARD_CONNECT = 9 % UInt32
const LPCSCLI_OPERATIONAL = 10 % UInt32

const lws_ss_state_return = Int32
const LWSSSSRET_TX_DONT_SEND = 1 % Int32
const LWSSSSRET_OK = 0 % Int32
const LWSSSSRET_DISCONNECT_ME = -1 % Int32
const LWSSSSRET_DESTROY_ME = -2 % Int32

const lws_ss_state_return_t = lws_ss_state_return

const __JL_Ctag_36 = UInt32
const LWSSSINFLAGS_REGISTER_SINK = 1 % UInt32
const LWSSSINFLAGS_PROXIED = 2 % UInt32
const LWSSSINFLAGS_SERVER = 4 % UInt32
const LWSSSINFLAGS_ACCEPTED = 8 % UInt32

# typedef lws_ss_state_return_t ( * lws_sscb_rx ) ( void * userobj , const uint8_t * buf , size_t len , int flags )
const lws_sscb_rx = Ptr{Cvoid}

# typedef lws_ss_state_return_t ( * lws_sscb_tx ) ( void * userobj , lws_ss_tx_ordinal_t ord , uint8_t * buf , size_t * len , int * flags )
const lws_sscb_tx = Ptr{Cvoid}

# typedef lws_ss_state_return_t ( * lws_sscb_state ) ( void * userobj , void * h_src , lws_ss_constate_t state , lws_ss_tx_ordinal_t ack )
const lws_sscb_state = Ptr{Cvoid}

struct lws_ss_info
    streamtype::Ptr{Cchar}
    user_alloc::Csize_t
    handle_offset::Csize_t
    opaque_user_data_offset::Csize_t
    rx::lws_sscb_rx
    tx::lws_sscb_tx
    state::lws_sscb_state
    manual_initial_tx_credit::Cint
    client_pid::UInt32
    flags::UInt8
    sss_protocol_version::UInt8
end

const lws_ss_info_t = lws_ss_info

function lws_ss_create(context, tsi, ssi, opaque_user_data, ppss, seq_owner, ppayload_fmt)
    @ccall libwebsockets.lws_ss_create(context::Ptr{lws_context}, tsi::Cint, ssi::Ptr{lws_ss_info_t}, opaque_user_data::Ptr{Cvoid}, ppss::Ptr{Ptr{lws_ss_handle}}, seq_owner::Ptr{lws_sequencer}, ppayload_fmt::Ptr{Ptr{Cchar}})::Cint
end

function lws_ss_destroy(ppss)
    @ccall libwebsockets.lws_ss_destroy(ppss::Ptr{Ptr{lws_ss_handle}})::Cvoid
end

function lws_ss_request_tx(pss)
    @ccall libwebsockets.lws_ss_request_tx(pss::Ptr{lws_ss_handle})::lws_ss_state_return_t
end

function lws_ss_request_tx_len(pss, len)
    @ccall libwebsockets.lws_ss_request_tx_len(pss::Ptr{lws_ss_handle}, len::Culong)::lws_ss_state_return_t
end

function lws_ss_client_connect(h)
    @ccall libwebsockets.lws_ss_client_connect(h::Ptr{lws_ss_handle})::lws_ss_state_return_t
end

function lws_ss_get_sequencer(h)
    @ccall libwebsockets.lws_ss_get_sequencer(h::Ptr{lws_ss_handle})::Ptr{lws_sequencer}
end

function lws_ss_proxy_create(context, bind, port)
    @ccall libwebsockets.lws_ss_proxy_create(context::Ptr{lws_context}, bind::Ptr{Cchar}, port::Cint)::Cint
end

function lws_ss_state_name(state)
    @ccall libwebsockets.lws_ss_state_name(state::Cint)::Ptr{Cchar}
end

function lws_ss_get_context(h)
    @ccall libwebsockets.lws_ss_get_context(h::Ptr{lws_ss_handle})::Ptr{lws_context}
end

function lws_ss_start_timeout(h, timeout_ms)
    @ccall libwebsockets.lws_ss_start_timeout(h::Ptr{lws_ss_handle}, timeout_ms::Cuint)::Cvoid
end

function lws_ss_cancel_timeout(h)
    @ccall libwebsockets.lws_ss_cancel_timeout(h::Ptr{lws_ss_handle})::Cvoid
end

function lws_ss_to_user_object(h)
    @ccall libwebsockets.lws_ss_to_user_object(h::Ptr{lws_ss_handle})::Ptr{Cvoid}
end

function lws_ss_rideshare(h)
    @ccall libwebsockets.lws_ss_rideshare(h::Ptr{lws_ss_handle})::Ptr{Cchar}
end

function lws_ss_set_metadata(h, name, value, len)
    @ccall libwebsockets.lws_ss_set_metadata(h::Ptr{lws_ss_handle}, name::Ptr{Cchar}, value::Ptr{Cvoid}, len::Csize_t)::Cint
end

function lws_ss_alloc_set_metadata(h, name, value, len)
    @ccall libwebsockets.lws_ss_alloc_set_metadata(h::Ptr{lws_ss_handle}, name::Ptr{Cchar}, value::Ptr{Cvoid}, len::Csize_t)::Cint
end

function lws_ss_get_metadata(h, name, value, len)
    @ccall libwebsockets.lws_ss_get_metadata(h::Ptr{lws_ss_handle}, name::Ptr{Cchar}, value::Ptr{Ptr{Cvoid}}, len::Ptr{Csize_t})::Cint
end

function lws_ss_server_ack(h, nack)
    @ccall libwebsockets.lws_ss_server_ack(h::Ptr{lws_ss_handle}, nack::Cint)::Cvoid
end

# typedef void ( * lws_sssfec_cb ) ( struct lws_ss_handle * h , void * arg )
const lws_sssfec_cb = Ptr{Cvoid}

function lws_ss_server_foreach_client(h, cb, arg)
    @ccall libwebsockets.lws_ss_server_foreach_client(h::Ptr{lws_ss_handle}, cb::lws_sssfec_cb, arg::Ptr{Cvoid})::Cvoid
end

function lws_ss_change_handlers(h, rx, tx, state)
    @ccall libwebsockets.lws_ss_change_handlers(h::Ptr{lws_ss_handle}, rx::lws_sscb_rx, tx::lws_sscb_tx, state::lws_sscb_state)::Cvoid
end

function lws_ss_add_peer_tx_credit(h, add)
    @ccall libwebsockets.lws_ss_add_peer_tx_credit(h::Ptr{lws_ss_handle}, add::Int32)::Cint
end

function lws_ss_get_est_peer_tx_credit(h)
    @ccall libwebsockets.lws_ss_get_est_peer_tx_credit(h::Ptr{lws_ss_handle})::Cint
end

function lws_ss_tag(h)
    @ccall libwebsockets.lws_ss_tag(h::Ptr{lws_ss_handle})::Ptr{Cchar}
end

# typedef int ( * plugin_auth_status_cb ) ( struct lws_ss_handle * ss , int status )
const plugin_auth_status_cb = Ptr{Cvoid}

struct lws_metric_policy
    next::Ptr{lws_metric_policy}
    name::Ptr{Cchar}
    report::Ptr{Cchar}
    us_schedule::UInt64
    us_decay_unit::UInt32
    min_contributors::UInt8
end

const lws_metric_policy_t = lws_metric_policy

struct lws_ss_x509
    data::NTuple{40, UInt8}
end

function Base.getproperty(x::Ptr{lws_ss_x509}, f::Symbol)
    f === :next && return Ptr{Ptr{lws_ss_x509}}(x + 0)
    f === :vhost_name && return Ptr{Ptr{Cchar}}(x + 8)
    f === :ca_der && return Ptr{Ptr{UInt8}}(x + 16)
    f === :ca_der_len && return Ptr{Csize_t}(x + 24)
    f === :keep && return (Ptr{UInt8}(x + 32), 0, 1)
    return getfield(x, f)
end

function Base.getproperty(x::lws_ss_x509, f::Symbol)
    r = Ref{lws_ss_x509}(x)
    ptr = Base.unsafe_convert(Ptr{lws_ss_x509}, r)
    fptr = getproperty(ptr, f)
    begin
        if fptr isa Ptr
            return GC.@preserve(r, unsafe_load(fptr))
        else
            (baseptr, offset, width) = fptr
            ty = eltype(baseptr)
            baseptr32 = convert(Ptr{UInt32}, baseptr)
            u64 = GC.@preserve(r, unsafe_load(baseptr32))
            if offset + width > 32
                u64 |= GC.@preserve(r, unsafe_load(baseptr32 + 4)) << 32
            end
            u64 = u64 >> offset & (1 << width - 1)
            return u64 % ty
        end
    end
end

function Base.setproperty!(x::Ptr{lws_ss_x509}, f::Symbol, v)
    fptr = getproperty(x, f)
    if fptr isa Ptr
        unsafe_store!(getproperty(x, f), v)
    else
        (baseptr, offset, width) = fptr
        baseptr32 = convert(Ptr{UInt32}, baseptr)
        u64 = unsafe_load(baseptr32)
        straddle = offset + width > 32
        if straddle
            u64 |= unsafe_load(baseptr32 + 4) << 32
        end
        mask = 1 << width - 1
        u64 &= ~(mask << offset)
        u64 |= (unsigned(v) & mask) << offset
        unsafe_store!(baseptr32, u64 & typemax(UInt32))
        if straddle
            unsafe_store!(baseptr32 + 4, u64 >> 32)
        end
    end
end

const lws_ss_x509_t = lws_ss_x509

const __JL_Ctag_37 = UInt32
const LWSSSPOLF_OPPORTUNISTIC = 1 % UInt32
const LWSSSPOLF_NAILED_UP = 2 % UInt32
const LWSSSPOLF_URGENT_TX = 4 % UInt32
const LWSSSPOLF_URGENT_RX = 8 % UInt32
const LWSSSPOLF_TLS = 16 % UInt32
const LWSSSPOLF_LONG_POLL = 32 % UInt32
const LWSSSPOLF_AUTH_BEARER = 64 % UInt32
const LWSSSPOLF_HTTP_NO_CONTENT_LENGTH = 128 % UInt32
const LWSSSPOLF_QUIRK_NGHTTP2_END_STREAM = 256 % UInt32
const LWSSSPOLF_H2_QUIRK_OVERFLOWS_TXCR = 512 % UInt32
const LWSSSPOLF_H2_QUIRK_UNCLEAN_HPACK_STATE = 1024 % UInt32
const LWSSSPOLF_HTTP_MULTIPART = 2048 % UInt32
const LWSSSPOLF_HTTP_X_WWW_FORM_URLENCODED = 4096 % UInt32
const LWSSSPOLF_LOCAL_SINK = 8192 % UInt32
const LWSSSPOLF_WAKE_SUSPEND__VALIDITY = 16384 % UInt32
const LWSSSPOLF_SERVER = 32768 % UInt32
const LWSSSPOLF_ALLOW_REDIRECTS = 65536 % UInt32
const LWSSSPOLF_HTTP_MULTIPART_IN = 131072 % UInt32
const LWSSSPOLF_ATTR_LOW_LATENCY = 262144 % UInt32
const LWSSSPOLF_ATTR_HIGH_THROUGHPUT = 524288 % UInt32
const LWSSSPOLF_ATTR_HIGH_RELIABILITY = 1048576 % UInt32
const LWSSSPOLF_ATTR_LOW_COST = 2097152 % UInt32
const LWSSSPOLF_PERF = 4194304 % UInt32
const LWSSSPOLF_DIRECT_PROTO_STR = 8388608 % UInt32
const LWSSSPOLF_HTTP_CACHE_COOKIES = 16777216 % UInt32
const LWSSSPOLF_PRIORITIZE_READS = 33554432 % UInt32

struct lws_ss_trust_store
    next::Ptr{lws_ss_trust_store}
    name::Ptr{Cchar}
    ssx509::NTuple{6, Ptr{lws_ss_x509_t}}
    count::Cint
end

const lws_ss_trust_store_t = lws_ss_trust_store

const __JL_Ctag_38 = UInt32
const LWSSSP_H1 = 0 % UInt32
const LWSSSP_H2 = 1 % UInt32
const LWSSSP_WS = 2 % UInt32
const LWSSSP_MQTT = 3 % UInt32
const LWSSSP_RAW = 4 % UInt32
const LWSSS_HBI_AUTH = 0 % UInt32
const LWSSS_HBI_DSN = 1 % UInt32
const LWSSS_HBI_FWV = 2 % UInt32
const LWSSS_HBI_TYPE = 3 % UInt32
const _LWSSS_HBI_COUNT = 4 % UInt32

struct lws_ss_metadata
    data::NTuple{40, UInt8}
end

function Base.getproperty(x::Ptr{lws_ss_metadata}, f::Symbol)
    f === :next && return Ptr{Ptr{lws_ss_metadata}}(x + 0)
    f === :name && return Ptr{Ptr{Cchar}}(x + 8)
    f === :value__may_own_heap && return Ptr{Ptr{Cvoid}}(x + 16)
    f === :length && return Ptr{Csize_t}(x + 24)
    f === :value_length && return Ptr{UInt8}(x + 32)
    f === :value_is_http_token && return Ptr{UInt8}(x + 33)
    f === :value_on_lws_heap && return (Ptr{UInt8}(x + 32), 16, 1)
    return getfield(x, f)
end

function Base.getproperty(x::lws_ss_metadata, f::Symbol)
    r = Ref{lws_ss_metadata}(x)
    ptr = Base.unsafe_convert(Ptr{lws_ss_metadata}, r)
    fptr = getproperty(ptr, f)
    begin
        if fptr isa Ptr
            return GC.@preserve(r, unsafe_load(fptr))
        else
            (baseptr, offset, width) = fptr
            ty = eltype(baseptr)
            baseptr32 = convert(Ptr{UInt32}, baseptr)
            u64 = GC.@preserve(r, unsafe_load(baseptr32))
            if offset + width > 32
                u64 |= GC.@preserve(r, unsafe_load(baseptr32 + 4)) << 32
            end
            u64 = u64 >> offset & (1 << width - 1)
            return u64 % ty
        end
    end
end

function Base.setproperty!(x::Ptr{lws_ss_metadata}, f::Symbol, v)
    fptr = getproperty(x, f)
    if fptr isa Ptr
        unsafe_store!(getproperty(x, f), v)
    else
        (baseptr, offset, width) = fptr
        baseptr32 = convert(Ptr{UInt32}, baseptr)
        u64 = unsafe_load(baseptr32)
        straddle = offset + width > 32
        if straddle
            u64 |= unsafe_load(baseptr32 + 4) << 32
        end
        mask = 1 << width - 1
        u64 &= ~(mask << offset)
        u64 |= (unsigned(v) & mask) << offset
        unsafe_store!(baseptr32, u64 & typemax(UInt32))
        if straddle
            unsafe_store!(baseptr32 + 4, u64 >> 32)
        end
    end
end

const lws_ss_metadata_t = lws_ss_metadata

struct lws_ss_http_respmap
    resp::UInt16
    state::UInt16
end

const lws_ss_http_respmap_t = lws_ss_http_respmap

struct lws_ss_auth
    next::Ptr{lws_ss_auth}
    name::Ptr{Cchar}
    type::Ptr{Cchar}
    streamtype::Ptr{Cchar}
    blob_index::UInt8
end

const lws_ss_auth_t = lws_ss_auth

struct __JL_Ctag_57
    data::NTuple{112, UInt8}
end

function Base.getproperty(x::Ptr{__JL_Ctag_57}, f::Symbol)
    f === :http && return Ptr{Cvoid}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_57, f::Symbol)
    r = Ref{__JL_Ctag_57}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_57}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_57}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

struct __JL_Ctag_61
    data::NTuple{16, UInt8}
end

function Base.getproperty(x::Ptr{__JL_Ctag_61}, f::Symbol)
    f === :store && return Ptr{Ptr{lws_ss_trust_store_t}}(x + 0)
    f === :server && return Ptr{__JL_Ctag_62}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_61, f::Symbol)
    r = Ref{__JL_Ctag_61}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_61}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_61}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

struct lws_ss_policy
    data::NTuple{248, UInt8}
end

function Base.getproperty(x::Ptr{lws_ss_policy}, f::Symbol)
    f === :next && return Ptr{Ptr{lws_ss_policy}}(x + 0)
    f === :streamtype && return Ptr{Ptr{Cchar}}(x + 8)
    f === :endpoint && return Ptr{Ptr{Cchar}}(x + 16)
    f === :rideshare_streamtype && return Ptr{Ptr{Cchar}}(x + 24)
    f === :payload_fmt && return Ptr{Ptr{Cchar}}(x + 32)
    f === :socks5_proxy && return Ptr{Ptr{Cchar}}(x + 40)
    f === :metadata && return Ptr{Ptr{lws_ss_metadata_t}}(x + 48)
    f === :metrics && return Ptr{Ptr{lws_metric_policy_t}}(x + 56)
    f === :auth && return Ptr{Ptr{lws_ss_auth_t}}(x + 64)
    f === :u && return Ptr{__JL_Ctag_57}(x + 72)
    f === :trust && return Ptr{__JL_Ctag_61}(x + 184)
    f === :retry_bo && return Ptr{Ptr{lws_retry_bo_t}}(x + 200)
    f === :proxy_buflen && return Ptr{UInt32}(x + 208)
    f === :proxy_buflen_rxflow_on_above && return Ptr{UInt32}(x + 212)
    f === :proxy_buflen_rxflow_off_below && return Ptr{UInt32}(x + 216)
    f === :client_buflen && return Ptr{UInt32}(x + 220)
    f === :client_buflen_rxflow_on_above && return Ptr{UInt32}(x + 224)
    f === :client_buflen_rxflow_off_below && return Ptr{UInt32}(x + 228)
    f === :timeout_ms && return Ptr{UInt32}(x + 232)
    f === :flags && return Ptr{UInt32}(x + 236)
    f === :port && return Ptr{UInt16}(x + 240)
    f === :metadata_count && return Ptr{UInt8}(x + 242)
    f === :protocol && return Ptr{UInt8}(x + 243)
    f === :client_cert && return Ptr{UInt8}(x + 244)
    f === :priority && return Ptr{UInt8}(x + 245)
    return getfield(x, f)
end

function Base.getproperty(x::lws_ss_policy, f::Symbol)
    r = Ref{lws_ss_policy}(x)
    ptr = Base.unsafe_convert(Ptr{lws_ss_policy}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{lws_ss_policy}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

const lws_ss_policy_t = lws_ss_policy

function lws_ss_policy_parse_begin(context, overlay)
    @ccall libwebsockets.lws_ss_policy_parse_begin(context::Ptr{lws_context}, overlay::Cint)::Cint
end

function lws_ss_policy_parse_abandon(context)
    @ccall libwebsockets.lws_ss_policy_parse_abandon(context::Ptr{lws_context})::Cint
end

function lws_ss_policy_parse(context, buf, len)
    @ccall libwebsockets.lws_ss_policy_parse(context::Ptr{lws_context}, buf::Ptr{UInt8}, len::Csize_t)::Cint
end

function lws_ss_policy_overlay(context, overlay)
    @ccall libwebsockets.lws_ss_policy_overlay(context::Ptr{lws_context}, overlay::Ptr{Cchar})::Cint
end

function lws_ss_policy_get(context)
    @ccall libwebsockets.lws_ss_policy_get(context::Ptr{lws_context})::Ptr{lws_ss_policy_t}
end

function lws_ss_auth_get(context)
    @ccall libwebsockets.lws_ss_auth_get(context::Ptr{lws_context})::Ptr{lws_ss_auth_t}
end

mutable struct lws_sspc_handle end

function lws_sspc_create(context, tsi, ssi, opaque_user_data, ppss, seq_owner, ppayload_fmt)
    @ccall libwebsockets.lws_sspc_create(context::Ptr{lws_context}, tsi::Cint, ssi::Ptr{lws_ss_info_t}, opaque_user_data::Ptr{Cvoid}, ppss::Ptr{Ptr{lws_sspc_handle}}, seq_owner::Ptr{lws_sequencer}, ppayload_fmt::Ptr{Ptr{Cchar}})::Cint
end

function lws_sspc_destroy(ppss)
    @ccall libwebsockets.lws_sspc_destroy(ppss::Ptr{Ptr{lws_sspc_handle}})::Cvoid
end

function lws_sspc_request_tx(pss)
    @ccall libwebsockets.lws_sspc_request_tx(pss::Ptr{lws_sspc_handle})::lws_ss_state_return_t
end

function lws_sspc_request_tx_len(h, len)
    @ccall libwebsockets.lws_sspc_request_tx_len(h::Ptr{lws_sspc_handle}, len::Culong)::lws_ss_state_return_t
end

function lws_sspc_client_connect(h)
    @ccall libwebsockets.lws_sspc_client_connect(h::Ptr{lws_sspc_handle})::lws_ss_state_return_t
end

function lws_sspc_get_sequencer(h)
    @ccall libwebsockets.lws_sspc_get_sequencer(h::Ptr{lws_sspc_handle})::Ptr{lws_sequencer}
end

function lws_sspc_proxy_create(context)
    @ccall libwebsockets.lws_sspc_proxy_create(context::Ptr{lws_context})::Cint
end

function lws_sspc_get_context(h)
    @ccall libwebsockets.lws_sspc_get_context(h::Ptr{lws_sspc_handle})::Ptr{lws_context}
end

function lws_sspc_rideshare(h)
    @ccall libwebsockets.lws_sspc_rideshare(h::Ptr{lws_sspc_handle})::Ptr{Cchar}
end

function lws_sspc_set_metadata(h, name, value, len)
    @ccall libwebsockets.lws_sspc_set_metadata(h::Ptr{lws_sspc_handle}, name::Ptr{Cchar}, value::Ptr{Cvoid}, len::Csize_t)::Cint
end

function lws_sspc_get_metadata(h, name, value, len)
    @ccall libwebsockets.lws_sspc_get_metadata(h::Ptr{lws_sspc_handle}, name::Ptr{Cchar}, value::Ptr{Ptr{Cvoid}}, len::Ptr{Csize_t})::Cint
end

function lws_sspc_add_peer_tx_credit(h, add)
    @ccall libwebsockets.lws_sspc_add_peer_tx_credit(h::Ptr{lws_sspc_handle}, add::Int32)::Cint
end

function lws_sspc_get_est_peer_tx_credit(h)
    @ccall libwebsockets.lws_sspc_get_est_peer_tx_credit(h::Ptr{lws_sspc_handle})::Cint
end

function lws_sspc_start_timeout(h, timeout_ms)
    @ccall libwebsockets.lws_sspc_start_timeout(h::Ptr{lws_sspc_handle}, timeout_ms::Cuint)::Cvoid
end

function lws_sspc_cancel_timeout(h)
    @ccall libwebsockets.lws_sspc_cancel_timeout(h::Ptr{lws_sspc_handle})::Cvoid
end

function lws_sspc_to_user_object(h)
    @ccall libwebsockets.lws_sspc_to_user_object(h::Ptr{lws_sspc_handle})::Ptr{Cvoid}
end

function lws_sspc_change_handlers(h, rx, tx, state)
    @ccall libwebsockets.lws_sspc_change_handlers(h::Ptr{lws_sspc_handle}, rx::Ptr{Cvoid}, tx::Ptr{Cvoid}, state::Ptr{Cvoid})::Cvoid
end

function lws_sspc_tag(h)
    @ccall libwebsockets.lws_sspc_tag(h::Ptr{lws_sspc_handle})::Ptr{Cchar}
end

struct __JL_Ctag_56
    data::NTuple{8, UInt8}
end

function Base.getproperty(x::Ptr{__JL_Ctag_56}, f::Symbol)
    f === :value && return Ptr{Ptr{Cchar}}(x + 0)
    f === :bvalue && return Ptr{Ptr{UInt8}}(x + 0)
    f === :lvalue && return Ptr{Culong}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_56, f::Symbol)
    r = Ref{__JL_Ctag_56}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_56}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_56}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

struct lws_token_map
    data::NTuple{16, UInt8}
end

function Base.getproperty(x::Ptr{lws_token_map}, f::Symbol)
    f === :u && return Ptr{__JL_Ctag_56}(x + 0)
    f === :name_index && return Ptr{Cshort}(x + 8)
    f === :length_or_zero && return Ptr{Cshort}(x + 10)
    return getfield(x, f)
end

function Base.getproperty(x::lws_token_map, f::Symbol)
    r = Ref{lws_token_map}(x)
    ptr = Base.unsafe_convert(Ptr{lws_token_map}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{lws_token_map}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

const lws_token_map_t = lws_token_map

const __JL_Ctag_39 = UInt32
const LTMI_END_OF_ARRAY = 0 % UInt32
const LTMI_PROTOCOL_BASE = 2048 % UInt32
const LTMI_TRANSPORT_BASE = 4096 % UInt32

mutable struct lws_abs end

const lws_abs_t = lws_abs

function lws_abs_get_token(token_map, name_index)
    @ccall libwebsockets.lws_abs_get_token(token_map::Ptr{lws_token_map_t}, name_index::Cshort)::Ptr{lws_token_map_t}
end

const lws_abs_transport_inst_t = Cvoid

const lws_abs_protocol_inst_t = Cvoid

function lws_abstract_alloc(vhost, user, abstract_path, ap_tokens, at_tokens, seq, opaque_user_data)
    @ccall libwebsockets.lws_abstract_alloc(vhost::Ptr{lws_vhost}, user::Ptr{Cvoid}, abstract_path::Ptr{Cchar}, ap_tokens::Ptr{lws_token_map_t}, at_tokens::Ptr{lws_token_map_t}, seq::Ptr{lws_sequencer}, opaque_user_data::Ptr{Cvoid})::Ptr{lws_abs_t}
end

function lws_abstract_free(pabs)
    @ccall libwebsockets.lws_abstract_free(pabs::Ptr{Ptr{lws_abs_t}})::Cvoid
end

function lws_abs_bind_and_create_instance(ai)
    @ccall libwebsockets.lws_abs_bind_and_create_instance(ai::Ptr{lws_abs_t})::Ptr{lws_abs_t}
end

function lws_abs_destroy_instance(ai)
    @ccall libwebsockets.lws_abs_destroy_instance(ai::Ptr{Ptr{lws_abs_t}})::Cvoid
end

const __JL_Ctag_40 = UInt32
const LWS_AP_FLAG_PIPELINE_TRANSACTIONS = 1 % UInt32
const LWS_AP_FLAG_MUXABLE_STREAM = 2 % UInt32

struct lws_abs_protocol
    name::Ptr{Cchar}
    alloc::Cint
    flags::Cint
    create::Ptr{Cvoid}
    destroy::Ptr{Cvoid}
    compare::Ptr{Cvoid}
    accept::Ptr{Cvoid}
    rx::Ptr{Cvoid}
    writeable::Ptr{Cvoid}
    closed::Ptr{Cvoid}
    heartbeat::Ptr{Cvoid}
    child_bind::Ptr{Cvoid}
end

const lws_abs_protocol_t = lws_abs_protocol

function lws_abs_protocol_get_by_name(name)
    @ccall libwebsockets.lws_abs_protocol_get_by_name(name::Ptr{Cchar})::Ptr{lws_abs_protocol_t}
end

struct lws_abs_transport
    name::Ptr{Cchar}
    alloc::Cint
    create::Ptr{Cvoid}
    destroy::Ptr{Cvoid}
    compare::Ptr{Cvoid}
    tx::Ptr{Cvoid}
    client_conn::Ptr{Cvoid}
    close::Ptr{Cvoid}
    ask_for_writeable::Ptr{Cvoid}
    set_timeout::Ptr{Cvoid}
    state::Ptr{Cvoid}
end

const lws_abs_transport_t = lws_abs_transport

function lws_abs_transport_get_by_name(name)
    @ccall libwebsockets.lws_abs_transport_get_by_name(name::Ptr{Cchar})::Ptr{lws_abs_transport_t}
end

const __JL_Ctag_41 = UInt32
const LTMI_PEER_V_DNS_ADDRESS = 4096 % UInt32
const LTMI_PEER_LV_PORT = 4097 % UInt32
const LTMI_PEER_LV_TLS_FLAGS = 4098 % UInt32

const __JL_Ctag_42 = UInt32
const LWS_AUT_EXPECT_TEST_END = 1 % UInt32
const LWS_AUT_EXPECT_LOCAL_CLOSE = 2 % UInt32
const LWS_AUT_EXPECT_DO_REMOTE_CLOSE = 4 % UInt32
const LWS_AUT_EXPECT_TX = 8 % UInt32
const LWS_AUT_EXPECT_RX = 16 % UInt32
const LWS_AUT_EXPECT_SHOULD_FAIL = 32 % UInt32
const LWS_AUT_EXPECT_SHOULD_TIMEOUT = 64 % UInt32

const lws_unit_test_packet_disposition = UInt32
const LPE_CONTINUE = 0 % UInt32
const LPE_SUCCEEDED = 1 % UInt32
const LPE_FAILED = 2 % UInt32
const LPE_FAILED_UNEXPECTED_TIMEOUT = 3 % UInt32
const LPE_FAILED_UNEXPECTED_PASS = 4 % UInt32
const LPE_FAILED_UNEXPECTED_CLOSE = 5 % UInt32
const LPE_SKIPPED = 6 % UInt32
const LPE_CLOSING = 7 % UInt32

# typedef int ( * lws_unit_test_packet_test_cb ) ( const void * cb_user , int disposition )
const lws_unit_test_packet_test_cb = Ptr{Cvoid}

# typedef int ( * lws_unit_test_packet_cb ) ( lws_abs_t * instance )
const lws_unit_test_packet_cb = Ptr{Cvoid}

struct lws_unit_test_packet
    buffer::Ptr{Cvoid}
    pre::lws_unit_test_packet_cb
    len::Csize_t
    flags::UInt32
end

const lws_unit_test_packet_t = lws_unit_test_packet

struct lws_unit_test
    name::Ptr{Cchar}
    expect_array::Ptr{lws_unit_test_packet_t}
    max_secs::Cint
end

const lws_unit_test_t = lws_unit_test

const __JL_Ctag_44 = UInt32
const LTMI_PEER_V_EXPECT_TEST = 4096 % UInt32
const LTMI_PEER_V_EXPECT_RESULT_CB = 4097 % UInt32
const LTMI_PEER_V_EXPECT_RESULT_CB_ARG = 4098 % UInt32

function lws_unit_test_result_name(in)
    @ccall libwebsockets.lws_unit_test_result_name(in::Cint)::Ptr{Cchar}
end

# typedef void ( * lws_test_sequence_cb ) ( const void * cb_user )
const lws_test_sequence_cb = Ptr{Cvoid}

struct lws_test_sequencer_args
    abs::Ptr{lws_abs_t}
    tests::Ptr{lws_unit_test_t}
    results::Ptr{Cint}
    results_max::Cint
    count_tests::Ptr{Cint}
    count_passes::Ptr{Cint}
    cb::lws_test_sequence_cb
    cb_user::Ptr{Cvoid}
end

const lws_test_sequencer_args_t = lws_test_sequencer_args

function lws_abs_unit_test_sequencer(args)
    @ccall libwebsockets.lws_abs_unit_test_sequencer(args::Ptr{lws_test_sequencer_args_t})::Cint
end

const dns_query_type = UInt32
const LWS_ADNS_RECORD_A = 1 % UInt32
const LWS_ADNS_RECORD_CNAME = 5 % UInt32
const LWS_ADNS_RECORD_MX = 15 % UInt32
const LWS_ADNS_RECORD_AAAA = 28 % UInt32

const adns_query_type_t = dns_query_type

const lws_async_dns_retcode_t = Int32
const LADNS_RET_FAILED_WSI_CLOSED = -4 % Int32
const LADNS_RET_NXDOMAIN = -3 % Int32
const LADNS_RET_TIMEDOUT = -2 % Int32
const LADNS_RET_FAILED = -1 % Int32
const LADNS_RET_FOUND = 0 % Int32
const LADNS_RET_CONTINUING = 1 % Int32

# typedef struct lws * ( * lws_async_dns_cb_t ) ( struct lws * wsi , const char * ads , const struct addrinfo * result , int n , void * opaque )
const lws_async_dns_cb_t = Ptr{Cvoid}

function lws_async_dns_query(context, tsi, name, qtype, cb, wsi, opaque)
    @ccall libwebsockets.lws_async_dns_query(context::Ptr{lws_context}, tsi::Cint, name::Ptr{Cchar}, qtype::adns_query_type_t, cb::lws_async_dns_cb_t, wsi::Ptr{lws}, opaque::Ptr{Cvoid})::lws_async_dns_retcode_t
end

function lws_async_dns_freeaddrinfo(ai)
    @ccall libwebsockets.lws_async_dns_freeaddrinfo(ai::Ptr{Ptr{addrinfo}})::Cvoid
end

struct lws_tls_session_dump
    tag::NTuple{96, Cchar}
    blob::Ptr{Cvoid}
    opaque::Ptr{Cvoid}
    blob_len::Csize_t
end

# typedef int ( * lws_tls_sess_cb_t ) ( struct lws_context * cx , struct lws_tls_session_dump * info )
const lws_tls_sess_cb_t = Ptr{Cvoid}

function lws_tls_session_dump_save(vh, host, port, cb_save, opq)
    @ccall libwebsockets.lws_tls_session_dump_save(vh::Ptr{lws_vhost}, host::Ptr{Cchar}, port::UInt16, cb_save::lws_tls_sess_cb_t, opq::Ptr{Cvoid})::Cint
end

function lws_tls_session_dump_load(vh, host, port, cb_load, opq)
    @ccall libwebsockets.lws_tls_session_dump_load(vh::Ptr{lws_vhost}, host::Ptr{Cchar}, port::UInt16, cb_load::lws_tls_sess_cb_t, opq::Ptr{Cvoid})::Cint
end

const lws_genhash_types = UInt32
const LWS_GENHASH_TYPE_UNKNOWN = 0 % UInt32
const LWS_GENHASH_TYPE_MD5 = 1 % UInt32
const LWS_GENHASH_TYPE_SHA1 = 2 % UInt32
const LWS_GENHASH_TYPE_SHA256 = 3 % UInt32
const LWS_GENHASH_TYPE_SHA384 = 4 % UInt32
const LWS_GENHASH_TYPE_SHA512 = 5 % UInt32

const lws_genhmac_types = UInt32
const LWS_GENHMAC_TYPE_UNKNOWN = 0 % UInt32
const LWS_GENHMAC_TYPE_SHA256 = 1 % UInt32
const LWS_GENHMAC_TYPE_SHA384 = 2 % UInt32
const LWS_GENHMAC_TYPE_SHA512 = 3 % UInt32

struct lws_genhash_ctx
    type::UInt8
    evp_type::Ptr{Cint}
    mdctx::Ptr{Cint}
end

struct lws_genhmac_ctx
    type::UInt8
    evp_type::Ptr{Cint}
    ctx::Ptr{Cint}
    key::Ptr{Cint}
end

function lws_genhash_size(type)
    @ccall libwebsockets.lws_genhash_size(type::lws_genhash_types)::Csize_t
end

function lws_genhmac_size(type)
    @ccall libwebsockets.lws_genhmac_size(type::lws_genhmac_types)::Csize_t
end

function lws_genhash_init(ctx, type)
    @ccall libwebsockets.lws_genhash_init(ctx::Ptr{lws_genhash_ctx}, type::lws_genhash_types)::Cint
end

function lws_genhash_update(ctx, in, len)
    @ccall libwebsockets.lws_genhash_update(ctx::Ptr{lws_genhash_ctx}, in::Ptr{Cvoid}, len::Csize_t)::Cint
end

function lws_genhash_destroy(ctx, result)
    @ccall libwebsockets.lws_genhash_destroy(ctx::Ptr{lws_genhash_ctx}, result::Ptr{Cvoid})::Cint
end

function lws_genhmac_init(ctx, type, key, key_len)
    @ccall libwebsockets.lws_genhmac_init(ctx::Ptr{lws_genhmac_ctx}, type::lws_genhmac_types, key::Ptr{UInt8}, key_len::Csize_t)::Cint
end

function lws_genhmac_update(ctx, in, len)
    @ccall libwebsockets.lws_genhmac_update(ctx::Ptr{lws_genhmac_ctx}, in::Ptr{Cvoid}, len::Csize_t)::Cint
end

function lws_genhmac_destroy(ctx, result)
    @ccall libwebsockets.lws_genhmac_destroy(ctx::Ptr{lws_genhmac_ctx}, result::Ptr{Cvoid})::Cint
end

const enum_genrsa_mode = UInt32
const LGRSAM_PKCS1_1_5 = 0 % UInt32
const LGRSAM_PKCS1_OAEP_PSS = 1 % UInt32
const LGRSAM_COUNT = 2 % UInt32

struct lws_genrsa_ctx
    bn::NTuple{12, Ptr{Cint}}
    ctx::Ptr{Cint}
    rsa::Ptr{Cint}
    context::Ptr{lws_context}
    mode::enum_genrsa_mode
end

function lws_genrsa_create(ctx, el, context, mode, oaep_hashid)
    @ccall libwebsockets.lws_genrsa_create(ctx::Ptr{lws_genrsa_ctx}, el::Ptr{lws_gencrypto_keyelem}, context::Ptr{lws_context}, mode::enum_genrsa_mode, oaep_hashid::lws_genhash_types)::Cint
end

function lws_genrsa_destroy_elements(el)
    @ccall libwebsockets.lws_genrsa_destroy_elements(el::Ptr{lws_gencrypto_keyelem})::Cvoid
end

function lws_genrsa_new_keypair(context, ctx, mode, el, bits)
    @ccall libwebsockets.lws_genrsa_new_keypair(context::Ptr{lws_context}, ctx::Ptr{lws_genrsa_ctx}, mode::enum_genrsa_mode, el::Ptr{lws_gencrypto_keyelem}, bits::Cint)::Cint
end

function lws_genrsa_public_encrypt(ctx, in, in_len, out)
    @ccall libwebsockets.lws_genrsa_public_encrypt(ctx::Ptr{lws_genrsa_ctx}, in::Ptr{UInt8}, in_len::Csize_t, out::Ptr{UInt8})::Cint
end

function lws_genrsa_private_encrypt(ctx, in, in_len, out)
    @ccall libwebsockets.lws_genrsa_private_encrypt(ctx::Ptr{lws_genrsa_ctx}, in::Ptr{UInt8}, in_len::Csize_t, out::Ptr{UInt8})::Cint
end

function lws_genrsa_public_decrypt(ctx, in, in_len, out, out_max)
    @ccall libwebsockets.lws_genrsa_public_decrypt(ctx::Ptr{lws_genrsa_ctx}, in::Ptr{UInt8}, in_len::Csize_t, out::Ptr{UInt8}, out_max::Csize_t)::Cint
end

function lws_genrsa_private_decrypt(ctx, in, in_len, out, out_max)
    @ccall libwebsockets.lws_genrsa_private_decrypt(ctx::Ptr{lws_genrsa_ctx}, in::Ptr{UInt8}, in_len::Csize_t, out::Ptr{UInt8}, out_max::Csize_t)::Cint
end

function lws_genrsa_hash_sig_verify(ctx, in, hash_type, sig, sig_len)
    @ccall libwebsockets.lws_genrsa_hash_sig_verify(ctx::Ptr{lws_genrsa_ctx}, in::Ptr{UInt8}, hash_type::lws_genhash_types, sig::Ptr{UInt8}, sig_len::Csize_t)::Cint
end

function lws_genrsa_hash_sign(ctx, in, hash_type, sig, sig_len)
    @ccall libwebsockets.lws_genrsa_hash_sign(ctx::Ptr{lws_genrsa_ctx}, in::Ptr{UInt8}, hash_type::lws_genhash_types, sig::Ptr{UInt8}, sig_len::Csize_t)::Cint
end

function lws_genrsa_destroy(ctx)
    @ccall libwebsockets.lws_genrsa_destroy(ctx::Ptr{lws_genrsa_ctx})::Cvoid
end

function lws_genrsa_render_pkey_asn1(ctx, _private, pkey_asn1, pkey_asn1_len)
    @ccall libwebsockets.lws_genrsa_render_pkey_asn1(ctx::Ptr{lws_genrsa_ctx}, _private::Cint, pkey_asn1::Ptr{UInt8}, pkey_asn1_len::Csize_t)::Cint
end

const enum_aes_modes = UInt32
const LWS_GAESM_CBC = 0 % UInt32
const LWS_GAESM_CFB128 = 1 % UInt32
const LWS_GAESM_CFB8 = 2 % UInt32
const LWS_GAESM_CTR = 3 % UInt32
const LWS_GAESM_ECB = 4 % UInt32
const LWS_GAESM_OFB = 5 % UInt32
const LWS_GAESM_XTS = 6 % UInt32
const LWS_GAESM_GCM = 7 % UInt32
const LWS_GAESM_KW = 8 % UInt32

const enum_aes_operation = UInt32
const LWS_GAESO_ENC = 0 % UInt32
const LWS_GAESO_DEC = 1 % UInt32

const enum_aes_padding = UInt32
const LWS_GAESP_NO_PADDING = 0 % UInt32
const LWS_GAESP_WITH_PADDING = 1 % UInt32

struct lws_genaes_ctx
    ctx::Ptr{Cint}
    cipher::Ptr{Cint}
    engine::Ptr{Cint}
    init::Cchar
    tag::NTuple{16, Cuchar}
    k::Ptr{lws_gencrypto_keyelem}
    op::enum_aes_operation
    mode::enum_aes_modes
    padding::enum_aes_padding
    taglen::Cint
    underway::Cchar
end

function lws_genaes_create(ctx, op, mode, el, padding, engine)
    @ccall libwebsockets.lws_genaes_create(ctx::Ptr{lws_genaes_ctx}, op::enum_aes_operation, mode::enum_aes_modes, el::Ptr{lws_gencrypto_keyelem}, padding::enum_aes_padding, engine::Ptr{Cvoid})::Cint
end

function lws_genaes_destroy(ctx, tag, tlen)
    @ccall libwebsockets.lws_genaes_destroy(ctx::Ptr{lws_genaes_ctx}, tag::Ptr{Cuchar}, tlen::Csize_t)::Cint
end

function lws_genaes_crypt(ctx, in, len, out, iv_or_nonce_ctr_or_data_unit_16, stream_block_16, nc_or_iv_off, taglen)
    @ccall libwebsockets.lws_genaes_crypt(ctx::Ptr{lws_genaes_ctx}, in::Ptr{UInt8}, len::Csize_t, out::Ptr{UInt8}, iv_or_nonce_ctr_or_data_unit_16::Ptr{UInt8}, stream_block_16::Ptr{UInt8}, nc_or_iv_off::Ptr{Csize_t}, taglen::Cint)::Cint
end

const enum_genec_alg = UInt32
const LEGENEC_UNKNOWN = 0 % UInt32
const LEGENEC_ECDH = 1 % UInt32
const LEGENEC_ECDSA = 2 % UInt32

struct lws_genec_ctx
    ctx::NTuple{2, Ptr{Cint}}
    context::Ptr{lws_context}
    curve_table::Ptr{lws_ec_curves}
    genec_alg::enum_genec_alg
    has_private::Cchar
end

const enum_lws_dh_side = UInt32
const LDHS_OURS = 0 % UInt32
const LDHS_THEIRS = 1 % UInt32

struct lws_ec_curves
    name::Ptr{Cchar}
    tls_lib_nid::Cint
    key_bytes::UInt16
end

function lws_genecdh_create(ctx, context, curve_table)
    @ccall libwebsockets.lws_genecdh_create(ctx::Ptr{lws_genec_ctx}, context::Ptr{lws_context}, curve_table::Ptr{lws_ec_curves})::Cint
end

function lws_genecdh_set_key(ctx, el, side)
    @ccall libwebsockets.lws_genecdh_set_key(ctx::Ptr{lws_genec_ctx}, el::Ptr{lws_gencrypto_keyelem}, side::enum_lws_dh_side)::Cint
end

function lws_genecdh_new_keypair(ctx, side, curve_name, el)
    @ccall libwebsockets.lws_genecdh_new_keypair(ctx::Ptr{lws_genec_ctx}, side::enum_lws_dh_side, curve_name::Ptr{Cchar}, el::Ptr{lws_gencrypto_keyelem})::Cint
end

function lws_genecdh_compute_shared_secret(ctx, ss, ss_len)
    @ccall libwebsockets.lws_genecdh_compute_shared_secret(ctx::Ptr{lws_genec_ctx}, ss::Ptr{UInt8}, ss_len::Ptr{Cint})::Cint
end

function lws_genecdsa_create(ctx, context, curve_table)
    @ccall libwebsockets.lws_genecdsa_create(ctx::Ptr{lws_genec_ctx}, context::Ptr{lws_context}, curve_table::Ptr{lws_ec_curves})::Cint
end

function lws_genecdsa_new_keypair(ctx, curve_name, el)
    @ccall libwebsockets.lws_genecdsa_new_keypair(ctx::Ptr{lws_genec_ctx}, curve_name::Ptr{Cchar}, el::Ptr{lws_gencrypto_keyelem})::Cint
end

function lws_genecdsa_set_key(ctx, el)
    @ccall libwebsockets.lws_genecdsa_set_key(ctx::Ptr{lws_genec_ctx}, el::Ptr{lws_gencrypto_keyelem})::Cint
end

function lws_genecdsa_hash_sig_verify_jws(ctx, in, hash_type, keybits, sig, sig_len)
    @ccall libwebsockets.lws_genecdsa_hash_sig_verify_jws(ctx::Ptr{lws_genec_ctx}, in::Ptr{UInt8}, hash_type::lws_genhash_types, keybits::Cint, sig::Ptr{UInt8}, sig_len::Csize_t)::Cint
end

function lws_genecdsa_hash_sign_jws(ctx, in, hash_type, keybits, sig, sig_len)
    @ccall libwebsockets.lws_genecdsa_hash_sign_jws(ctx::Ptr{lws_genec_ctx}, in::Ptr{UInt8}, hash_type::lws_genhash_types, keybits::Cint, sig::Ptr{UInt8}, sig_len::Csize_t)::Cint
end

function lws_genec_destroy(ctx)
    @ccall libwebsockets.lws_genec_destroy(ctx::Ptr{lws_genec_ctx})::Cvoid
end

function lws_genec_destroy_elements(el)
    @ccall libwebsockets.lws_genec_destroy_elements(el::Ptr{lws_gencrypto_keyelem})::Cvoid
end

function lws_genec_dump(el)
    @ccall libwebsockets.lws_genec_dump(el::Ptr{lws_gencrypto_keyelem})::Cint
end

const enum_jwk_meta_tok = UInt32
const JWK_META_KTY = 0 % UInt32
const JWK_META_KID = 1 % UInt32
const JWK_META_USE = 2 % UInt32
const JWK_META_KEY_OPS = 3 % UInt32
const JWK_META_X5C = 4 % UInt32
const JWK_META_ALG = 5 % UInt32
const LWS_COUNT_JWK_ELEMENTS = 6 % UInt32

# typedef int ( * lws_jwk_key_import_callback ) ( struct lws_jwk * s , void * user )
const lws_jwk_key_import_callback = Ptr{Cvoid}

struct lws_jwk_parse_state
    jwk::Ptr{lws_jwk}
    b64::NTuple{1366, Cchar}
    per_key_cb::lws_jwk_key_import_callback
    user::Ptr{Cvoid}
    pos::Cint
    cose_state::Cint
    seen::Cint
    possible::Cushort
end

function lws_jwk_import(jwk, cb, user, in, len)
    @ccall libwebsockets.lws_jwk_import(jwk::Ptr{lws_jwk}, cb::lws_jwk_key_import_callback, user::Ptr{Cvoid}, in::Ptr{Cchar}, len::Csize_t)::Cint
end

function lws_jwk_destroy(jwk)
    @ccall libwebsockets.lws_jwk_destroy(jwk::Ptr{lws_jwk})::Cvoid
end

function lws_jwk_dup_oct(jwk, key, len)
    @ccall libwebsockets.lws_jwk_dup_oct(jwk::Ptr{lws_jwk}, key::Ptr{Cvoid}, len::Cint)::Cint
end

function lws_jwk_export(jwk, flags, p, len)
    @ccall libwebsockets.lws_jwk_export(jwk::Ptr{lws_jwk}, flags::Cint, p::Ptr{Cchar}, len::Ptr{Cint})::Cint
end

function lws_jwk_load(jwk, filename, cb, user)
    @ccall libwebsockets.lws_jwk_load(jwk::Ptr{lws_jwk}, filename::Ptr{Cchar}, cb::lws_jwk_key_import_callback, user::Ptr{Cvoid})::Cint
end

function lws_jwk_save(jwk, filename)
    @ccall libwebsockets.lws_jwk_save(jwk::Ptr{lws_jwk}, filename::Ptr{Cchar})::Cint
end

function lws_jwk_rfc7638_fingerprint(jwk, digest32)
    @ccall libwebsockets.lws_jwk_rfc7638_fingerprint(jwk::Ptr{lws_jwk}, digest32::Ptr{Cchar})::Cint
end

function lws_jwk_strdup_meta(jwk, idx, in, len)
    @ccall libwebsockets.lws_jwk_strdup_meta(jwk::Ptr{lws_jwk}, idx::enum_jwk_meta_tok, in::Ptr{Cchar}, len::Cint)::Cint
end

function lws_jwk_dump(jwk)
    @ccall libwebsockets.lws_jwk_dump(jwk::Ptr{lws_jwk})::Cint
end

function lws_jwk_generate(context, jwk, kty, bits, curve)
    @ccall libwebsockets.lws_jwk_generate(context::Ptr{lws_context}, jwk::Ptr{lws_jwk}, kty::lws_gencrypto_kty, bits::Cint, curve::Ptr{Cchar})::Cint
end

const lws_jws_jose_hdr_indexes = UInt32
const LJJHI_ALG = 0 % UInt32
const LJJHI_JKU = 1 % UInt32
const LJJHI_JWK = 2 % UInt32
const LJJHI_KID = 3 % UInt32
const LJJHI_X5U = 4 % UInt32
const LJJHI_X5C = 5 % UInt32
const LJJHI_X5T = 6 % UInt32
const LJJHI_X5T_S256 = 7 % UInt32
const LJJHI_TYP = 8 % UInt32
const LJJHI_CTY = 9 % UInt32
const LJJHI_CRIT = 10 % UInt32
const LJJHI_RECIPS_HDR = 11 % UInt32
const LJJHI_RECIPS_HDR_ALG = 12 % UInt32
const LJJHI_RECIPS_HDR_KID = 13 % UInt32
const LJJHI_RECIPS_EKEY = 14 % UInt32
const LJJHI_ENC = 15 % UInt32
const LJJHI_ZIP = 16 % UInt32
const LJJHI_EPK = 17 % UInt32
const LJJHI_APU = 18 % UInt32
const LJJHI_APV = 19 % UInt32
const LJJHI_IV = 20 % UInt32
const LJJHI_TAG = 21 % UInt32
const LJJHI_P2S = 22 % UInt32
const LJJHI_P2C = 23 % UInt32
const LWS_COUNT_JOSE_HDR_ELEMENTS = 24 % UInt32

const lws_jose_algtype = UInt32
const LWS_JOSE_ENCTYPE_NONE = 0 % UInt32
const LWS_JOSE_ENCTYPE_RSASSA_PKCS1_1_5 = 1 % UInt32
const LWS_JOSE_ENCTYPE_RSASSA_PKCS1_OAEP = 2 % UInt32
const LWS_JOSE_ENCTYPE_RSASSA_PKCS1_PSS = 3 % UInt32
const LWS_JOSE_ENCTYPE_ECDSA = 4 % UInt32
const LWS_JOSE_ENCTYPE_ECDHES = 5 % UInt32
const LWS_JOSE_ENCTYPE_AES_CBC = 6 % UInt32
const LWS_JOSE_ENCTYPE_AES_CFB128 = 7 % UInt32
const LWS_JOSE_ENCTYPE_AES_CFB8 = 8 % UInt32
const LWS_JOSE_ENCTYPE_AES_CTR = 9 % UInt32
const LWS_JOSE_ENCTYPE_AES_ECB = 10 % UInt32
const LWS_JOSE_ENCTYPE_AES_OFB = 11 % UInt32
const LWS_JOSE_ENCTYPE_AES_XTS = 12 % UInt32
const LWS_JOSE_ENCTYPE_AES_GCM = 13 % UInt32

struct lws_jose_jwe_alg
    hash_type::lws_genhash_types
    hmac_type::lws_genhmac_types
    algtype_signing::lws_jose_algtype
    algtype_crypto::lws_jose_algtype
    alg::Ptr{Cchar}
    curve_name::Ptr{Cchar}
    keybits_min::Cushort
    keybits_fixed::Cushort
    ivbits::Cushort
end

struct lws_jws_recpient
    unprot::NTuple{24, lws_gencrypto_keyelem}
    jwk_ephemeral::lws_jwk
    jwk::lws_jwk
end

struct lws_jose
    e::NTuple{24, lws_gencrypto_keyelem}
    recipient::NTuple{3, lws_jws_recpient}
    typ::NTuple{32, Cchar}
    alg::Ptr{lws_jose_jwe_alg}
    enc_alg::Ptr{lws_jose_jwe_alg}
    recipients::Cint
end

function lws_jose_init(jose)
    @ccall libwebsockets.lws_jose_init(jose::Ptr{lws_jose})::Cvoid
end

function lws_jose_destroy(jose)
    @ccall libwebsockets.lws_jose_destroy(jose::Ptr{lws_jose})::Cvoid
end

function lws_gencrypto_jws_alg_to_definition(alg, jose)
    @ccall libwebsockets.lws_gencrypto_jws_alg_to_definition(alg::Ptr{Cchar}, jose::Ptr{Ptr{lws_jose_jwe_alg}})::Cint
end

function lws_gencrypto_jwe_alg_to_definition(alg, jose)
    @ccall libwebsockets.lws_gencrypto_jwe_alg_to_definition(alg::Ptr{Cchar}, jose::Ptr{Ptr{lws_jose_jwe_alg}})::Cint
end

function lws_gencrypto_jwe_enc_to_definition(enc, jose)
    @ccall libwebsockets.lws_gencrypto_jwe_enc_to_definition(enc::Ptr{Cchar}, jose::Ptr{Ptr{lws_jose_jwe_alg}})::Cint
end

function lws_jws_parse_jose(jose, buf, len, temp, temp_len)
    @ccall libwebsockets.lws_jws_parse_jose(jose::Ptr{lws_jose}, buf::Ptr{Cchar}, len::Cint, temp::Ptr{Cchar}, temp_len::Ptr{Cint})::Cint
end

function lws_jwe_parse_jose(jose, buf, len, temp, temp_len)
    @ccall libwebsockets.lws_jwe_parse_jose(jose::Ptr{lws_jose}, buf::Ptr{Cchar}, len::Cint, temp::Ptr{Cchar}, temp_len::Ptr{Cint})::Cint
end

const enum_jws_sig_elements = UInt32
const LJWS_JOSE = 0 % UInt32
const LJWS_PYLD = 1 % UInt32
const LJWS_SIG = 2 % UInt32
const LJWS_UHDR = 3 % UInt32
const LJWE_JOSE = 0 % UInt32
const LJWE_EKEY = 1 % UInt32
const LJWE_IV = 2 % UInt32
const LJWE_CTXT = 3 % UInt32
const LJWE_ATAG = 4 % UInt32
const LJWE_AAD = 5 % UInt32
const LWS_JWS_MAX_COMPACT_BLOCKS = 6 % UInt32

struct lws_jws_map
    buf::NTuple{6, Ptr{Cchar}}
    len::NTuple{6, UInt32}
end

struct lws_jws
    jwk::Ptr{lws_jwk}
    context::Ptr{lws_context}
    map::lws_jws_map
    map_b64::lws_jws_map
end

function lws_jws_init(jws, jwk, context)
    @ccall libwebsockets.lws_jws_init(jws::Ptr{lws_jws}, jwk::Ptr{lws_jwk}, context::Ptr{lws_context})::Cvoid
end

function lws_jws_destroy(jws)
    @ccall libwebsockets.lws_jws_destroy(jws::Ptr{lws_jws})::Cvoid
end

function lws_jws_sig_confirm_compact(map, jwk, context, temp, temp_len)
    @ccall libwebsockets.lws_jws_sig_confirm_compact(map::Ptr{lws_jws_map}, jwk::Ptr{lws_jwk}, context::Ptr{lws_context}, temp::Ptr{Cchar}, temp_len::Ptr{Cint})::Cint
end

function lws_jws_sig_confirm_compact_b64_map(map_b64, jwk, context, temp, temp_len)
    @ccall libwebsockets.lws_jws_sig_confirm_compact_b64_map(map_b64::Ptr{lws_jws_map}, jwk::Ptr{lws_jwk}, context::Ptr{lws_context}, temp::Ptr{Cchar}, temp_len::Ptr{Cint})::Cint
end

function lws_jws_sig_confirm_compact_b64(in, len, map, jwk, context, temp, temp_len)
    @ccall libwebsockets.lws_jws_sig_confirm_compact_b64(in::Ptr{Cchar}, len::Csize_t, map::Ptr{lws_jws_map}, jwk::Ptr{lws_jwk}, context::Ptr{lws_context}, temp::Ptr{Cchar}, temp_len::Ptr{Cint})::Cint
end

function lws_jws_sig_confirm(map_b64, map, jwk, context)
    @ccall libwebsockets.lws_jws_sig_confirm(map_b64::Ptr{lws_jws_map}, map::Ptr{lws_jws_map}, jwk::Ptr{lws_jwk}, context::Ptr{lws_context})::Cint
end

function lws_jws_sign_from_b64(jose, jws, b64_sig, sig_len)
    @ccall libwebsockets.lws_jws_sign_from_b64(jose::Ptr{lws_jose}, jws::Ptr{lws_jws}, b64_sig::Ptr{Cchar}, sig_len::Csize_t)::Cint
end

function lws_jws_compact_decode(in, len, map, map_b64, out, out_len)
    @ccall libwebsockets.lws_jws_compact_decode(in::Ptr{Cchar}, len::Cint, map::Ptr{lws_jws_map}, map_b64::Ptr{lws_jws_map}, out::Ptr{Cchar}, out_len::Ptr{Cint})::Cint
end

function lws_jws_compact_encode(map_b64, map, buf, out_len)
    @ccall libwebsockets.lws_jws_compact_encode(map_b64::Ptr{lws_jws_map}, map::Ptr{lws_jws_map}, buf::Ptr{Cchar}, out_len::Ptr{Cint})::Cint
end

function lws_jws_sig_confirm_json(in, len, jws, jwk, context, temp, temp_len)
    @ccall libwebsockets.lws_jws_sig_confirm_json(in::Ptr{Cchar}, len::Csize_t, jws::Ptr{lws_jws}, jwk::Ptr{lws_jwk}, context::Ptr{lws_context}, temp::Ptr{Cchar}, temp_len::Ptr{Cint})::Cint
end

function lws_jws_write_flattened_json(jws, flattened, len)
    @ccall libwebsockets.lws_jws_write_flattened_json(jws::Ptr{lws_jws}, flattened::Ptr{Cchar}, len::Csize_t)::Cint
end

function lws_jws_write_compact(jws, compact, len)
    @ccall libwebsockets.lws_jws_write_compact(jws::Ptr{lws_jws}, compact::Ptr{Cchar}, len::Csize_t)::Cint
end

function lws_jws_dup_element(map, idx, temp, temp_len, in, in_len, actual_alloc)
    @ccall libwebsockets.lws_jws_dup_element(map::Ptr{lws_jws_map}, idx::Cint, temp::Ptr{Cchar}, temp_len::Ptr{Cint}, in::Ptr{Cvoid}, in_len::Csize_t, actual_alloc::Csize_t)::Cint
end

function lws_jws_randomize_element(context, map, idx, temp, temp_len, random_len, actual_alloc)
    @ccall libwebsockets.lws_jws_randomize_element(context::Ptr{lws_context}, map::Ptr{lws_jws_map}, idx::Cint, temp::Ptr{Cchar}, temp_len::Ptr{Cint}, random_len::Csize_t, actual_alloc::Csize_t)::Cint
end

function lws_jws_alloc_element(map, idx, temp, temp_len, len, actual_alloc)
    @ccall libwebsockets.lws_jws_alloc_element(map::Ptr{lws_jws_map}, idx::Cint, temp::Ptr{Cchar}, temp_len::Ptr{Cint}, len::Csize_t, actual_alloc::Csize_t)::Cint
end

function lws_jws_encode_b64_element(map, idx, temp, temp_len, in, in_len)
    @ccall libwebsockets.lws_jws_encode_b64_element(map::Ptr{lws_jws_map}, idx::Cint, temp::Ptr{Cchar}, temp_len::Ptr{Cint}, in::Ptr{Cvoid}, in_len::Csize_t)::Cint
end

function lws_jws_b64_compact_map(in, len, map)
    @ccall libwebsockets.lws_jws_b64_compact_map(in::Ptr{Cchar}, len::Cint, map::Ptr{lws_jws_map})::Cint
end

function lws_jws_base64_enc(in, in_len, out, out_max)
    @ccall libwebsockets.lws_jws_base64_enc(in::Ptr{Cchar}, in_len::Csize_t, out::Ptr{Cchar}, out_max::Csize_t)::Cint
end

function lws_jws_encode_section(in, in_len, first, p, _end)
    @ccall libwebsockets.lws_jws_encode_section(in::Ptr{Cchar}, in_len::Csize_t, first::Cint, p::Ptr{Ptr{Cchar}}, _end::Ptr{Cchar})::Cint
end

function lws_jwt_signed_validate(ctx, jwk, alg_list, com, len, temp, tl, out, out_len)
    @ccall libwebsockets.lws_jwt_signed_validate(ctx::Ptr{lws_context}, jwk::Ptr{lws_jwk}, alg_list::Ptr{Cchar}, com::Ptr{Cchar}, len::Csize_t, temp::Ptr{Cchar}, tl::Cint, out::Ptr{Cchar}, out_len::Ptr{Csize_t})::Cint
end

# automatic type deduction for variadic arguments may not be what you want, please use with caution
@generated function lws_jwt_sign_compact(ctx, jwk, alg, out, out_len, temp, tl, format, va_list...)
        :(@ccall(libwebsockets.lws_jwt_sign_compact(ctx::Ptr{lws_context}, jwk::Ptr{lws_jwk}, alg::Ptr{Cchar}, out::Ptr{Cchar}, out_len::Ptr{Csize_t}, temp::Ptr{Cchar}, tl::Cint, format::Ptr{Cchar}; $(to_c_type_pairs(va_list)...))::Cint))
    end

struct lws_jwt_sign_info
    alg::Ptr{Cchar}
    jose_hdr::Ptr{Cchar}
    jose_hdr_len::Csize_t
    out::Ptr{Cchar}
    out_len::Ptr{Csize_t}
    temp::Ptr{Cchar}
    tl::Cint
end

# automatic type deduction for variadic arguments may not be what you want, please use with caution
@generated function lws_jwt_sign_via_info(ctx, jwk, info, format, va_list...)
        :(@ccall(libwebsockets.lws_jwt_sign_via_info(ctx::Ptr{lws_context}, jwk::Ptr{lws_jwk}, info::Ptr{lws_jwt_sign_info}, format::Ptr{Cchar}; $(to_c_type_pairs(va_list)...))::Cint))
    end

function lws_jwt_token_sanity(in, in_len, iss, aud, csrf_in, sub, sub_len, exp_unix_time)
    @ccall libwebsockets.lws_jwt_token_sanity(in::Ptr{Cchar}, in_len::Csize_t, iss::Ptr{Cchar}, aud::Ptr{Cchar}, csrf_in::Ptr{Cchar}, sub::Ptr{Cchar}, sub_len::Csize_t, exp_unix_time::Ptr{Culong})::Cint
end

struct lws_jwt_sign_set_cookie
    jwk::Ptr{lws_jwk}
    alg::Ptr{Cchar}
    iss::Ptr{Cchar}
    aud::Ptr{Cchar}
    cookie_name::Ptr{Cchar}
    sub::NTuple{33, Cchar}
    extra_json::Ptr{Cchar}
    extra_json_len::Csize_t
    csrf_in::Ptr{Cchar}
    expiry_unix_time::Culong
end

function lws_jwt_sign_token_set_http_cookie(wsi, i, p, _end)
    @ccall libwebsockets.lws_jwt_sign_token_set_http_cookie(wsi::Ptr{lws}, i::Ptr{lws_jwt_sign_set_cookie}, p::Ptr{Ptr{UInt8}}, _end::Ptr{UInt8})::Cint
end

function lws_jwt_get_http_cookie_validate_jwt(wsi, i, out, out_len)
    @ccall libwebsockets.lws_jwt_get_http_cookie_validate_jwt(wsi::Ptr{lws}, i::Ptr{lws_jwt_sign_set_cookie}, out::Ptr{Cchar}, out_len::Ptr{Csize_t})::Cint
end

struct lws_jwe
    data::NTuple{4344, UInt8}
end

function Base.getproperty(x::Ptr{lws_jwe}, f::Symbol)
    f === :jose && return Ptr{lws_jose}(x + 0)
    f === :jws && return Ptr{lws_jws}(x + 3368)
    f === :jwk && return Ptr{lws_jwk}(x + 3528)
    f === :cek && return Ptr{NTuple{512, UInt8}}(x + 3824)
    f === :cek_valid && return (Ptr{Cuint}(x + 4336), 0, 1)
    f === :recip && return Ptr{Cint}(x + 4340)
    return getfield(x, f)
end

function Base.getproperty(x::lws_jwe, f::Symbol)
    r = Ref{lws_jwe}(x)
    ptr = Base.unsafe_convert(Ptr{lws_jwe}, r)
    fptr = getproperty(ptr, f)
    begin
        if fptr isa Ptr
            return GC.@preserve(r, unsafe_load(fptr))
        else
            (baseptr, offset, width) = fptr
            ty = eltype(baseptr)
            baseptr32 = convert(Ptr{UInt32}, baseptr)
            u64 = GC.@preserve(r, unsafe_load(baseptr32))
            if offset + width > 32
                u64 |= GC.@preserve(r, unsafe_load(baseptr32 + 4)) << 32
            end
            u64 = u64 >> offset & (1 << width - 1)
            return u64 % ty
        end
    end
end

function Base.setproperty!(x::Ptr{lws_jwe}, f::Symbol, v)
    fptr = getproperty(x, f)
    if fptr isa Ptr
        unsafe_store!(getproperty(x, f), v)
    else
        (baseptr, offset, width) = fptr
        baseptr32 = convert(Ptr{UInt32}, baseptr)
        u64 = unsafe_load(baseptr32)
        straddle = offset + width > 32
        if straddle
            u64 |= unsafe_load(baseptr32 + 4) << 32
        end
        mask = 1 << width - 1
        u64 &= ~(mask << offset)
        u64 |= (unsigned(v) & mask) << offset
        unsafe_store!(baseptr32, u64 & typemax(UInt32))
        if straddle
            unsafe_store!(baseptr32 + 4, u64 >> 32)
        end
    end
end

function lws_jwe_init(jwe, context)
    @ccall libwebsockets.lws_jwe_init(jwe::Ptr{lws_jwe}, context::Ptr{lws_context})::Cvoid
end

function lws_jwe_destroy(jwe)
    @ccall libwebsockets.lws_jwe_destroy(jwe::Ptr{lws_jwe})::Cvoid
end

function lws_jwe_be64(c, p8)
    @ccall libwebsockets.lws_jwe_be64(c::UInt64, p8::Ptr{UInt8})::Cvoid
end

function lws_jwe_render_compact(jwe, out, out_len)
    @ccall libwebsockets.lws_jwe_render_compact(jwe::Ptr{lws_jwe}, out::Ptr{Cchar}, out_len::Csize_t)::Cint
end

function lws_jwe_render_flattened(jwe, out, out_len)
    @ccall libwebsockets.lws_jwe_render_flattened(jwe::Ptr{lws_jwe}, out::Ptr{Cchar}, out_len::Csize_t)::Cint
end

function lws_jwe_json_parse(jwe, buf, len, temp, temp_len)
    @ccall libwebsockets.lws_jwe_json_parse(jwe::Ptr{lws_jwe}, buf::Ptr{UInt8}, len::Cint, temp::Ptr{Cchar}, temp_len::Ptr{Cint})::Cint
end

function lws_jwe_auth_and_decrypt(jwe, temp, temp_len)
    @ccall libwebsockets.lws_jwe_auth_and_decrypt(jwe::Ptr{lws_jwe}, temp::Ptr{Cchar}, temp_len::Ptr{Cint})::Cint
end

function lws_jwe_encrypt(jwe, temp, temp_len)
    @ccall libwebsockets.lws_jwe_encrypt(jwe::Ptr{lws_jwe}, temp::Ptr{Cchar}, temp_len::Ptr{Cint})::Cint
end

function lws_jwe_create_packet(jwe, payload, len, nonce, out, out_len, context)
    @ccall libwebsockets.lws_jwe_create_packet(jwe::Ptr{lws_jwe}, payload::Ptr{Cchar}, len::Csize_t, nonce::Ptr{Cchar}, out::Ptr{Cchar}, out_len::Csize_t, context::Ptr{lws_context})::Cint
end

function lws_jwe_auth_and_decrypt_cbc_hs(jwe, enc_cek, aad, aad_len)
    @ccall libwebsockets.lws_jwe_auth_and_decrypt_cbc_hs(jwe::Ptr{lws_jwe}, enc_cek::Ptr{UInt8}, aad::Ptr{UInt8}, aad_len::Cint)::Cint
end

function lws_jwa_concat_kdf(jwe, direct, out, shared_secret, sslen)
    @ccall libwebsockets.lws_jwa_concat_kdf(jwe::Ptr{lws_jwe}, direct::Cint, out::Ptr{UInt8}, shared_secret::Ptr{UInt8}, sslen::Cint)::Cint
end

const lws_event_lib_ops_flags = UInt32
const LELOF_ISPOLL = 1 % UInt32
const LELOF_DESTROY_FINAL = 0 % UInt32

const __JL_Ctag_46 = UInt32
const LWS_EV_READ = 1 % UInt32
const LWS_EV_WRITE = 2 % UInt32
const LWS_EV_START = 4 % UInt32
const LWS_EV_STOP = 8 % UInt32

function lws_evlib_wsi_to_evlib_pt(wsi)
    @ccall libwebsockets.lws_evlib_wsi_to_evlib_pt(wsi::Ptr{lws})::Ptr{Cvoid}
end

function lws_evlib_tsi_to_evlib_pt(ctx, tsi)
    @ccall libwebsockets.lws_evlib_tsi_to_evlib_pt(ctx::Ptr{lws_context}, tsi::Cint)::Ptr{Cvoid}
end

function lws_realloc(ptr, size, reason)
    @ccall libwebsockets.lws_realloc(ptr::Ptr{Cvoid}, size::Csize_t, reason::Ptr{Cchar})::Ptr{Cvoid}
end

function lws_vhost_destroy1(vh)
    @ccall libwebsockets.lws_vhost_destroy1(vh::Ptr{lws_vhost})::Cvoid
end

function lws_close_free_wsi(wsi, reason, caller)
    @ccall libwebsockets.lws_close_free_wsi(wsi::Ptr{lws}, reason::lws_close_status, caller::Ptr{Cchar})::Cvoid
end

function lws_vhost_foreach_listen_wsi(cx, arg, cb)
    @ccall libwebsockets.lws_vhost_foreach_listen_wsi(cx::Ptr{lws_context}, arg::Ptr{Cvoid}, cb::lws_dll2_foreach_cb_t)::Cint
end

mutable struct lws_context_per_thread end

function lws_service_do_ripe_rxflow(pt)
    @ccall libwebsockets.lws_service_do_ripe_rxflow(pt::Ptr{lws_context_per_thread})::Cvoid
end

function wsi_from_fd(context, fd)
    @ccall libwebsockets.wsi_from_fd(context::Ptr{lws_context}, fd::Cint)::Ptr{lws}
end

function _lws_plat_service_forced_tsi(context, tsi)
    @ccall libwebsockets._lws_plat_service_forced_tsi(context::Ptr{lws_context}, tsi::Cint)::Cint
end

function lws_context_destroy2(context)
    @ccall libwebsockets.lws_context_destroy2(context::Ptr{lws_context})::Cvoid
end

function lws_destroy_event_pipe(wsi)
    @ccall libwebsockets.lws_destroy_event_pipe(wsi::Ptr{lws})::Cvoid
end

function __lws_close_free_wsi_final(wsi)
    @ccall libwebsockets.__lws_close_free_wsi_final(wsi::Ptr{lws})::Cvoid
end

function lws_i2c_command(ctx, ads7, c)
    @ccall libwebsockets.lws_i2c_command(ctx::Ptr{lws_i2c_ops_t}, ads7::UInt8, c::UInt8)::Cint
end

function lws_i2c_command_list(ctx, ads7, buf, len)
    @ccall libwebsockets.lws_i2c_command_list(ctx::Ptr{lws_i2c_ops_t}, ads7::UInt8, buf::Ptr{UInt8}, len::Csize_t)::Cint
end

const __JL_Ctag_47 = UInt32
const LWSSPIMODE_CPOL = 1 % UInt32
const LWSSPIMODE_CPHA = 2 % UInt32
const LWS_SPI_BUSMODE_CLK_IDLE_LOW_SAMP_RISING = 0 % UInt32
const LWS_SPI_BUSMODE_CLK_IDLE_HIGH_SAMP_RISING = 1 % UInt32
const LWS_SPI_BUSMODE_CLK_IDLE_LOW_SAMP_FALLING = 2 % UInt32
const LWS_SPI_BUSMODE_CLK_IDLE_HIGH_SAMP_FALLING = 3 % UInt32
const LWS_SPI_TXN_HALF_DUPLEX_DISCRETE = 0 % UInt32

const lws_gpio_irq_t = UInt32
const LWSGGPIO_IRQ_NONE = 0 % UInt32
const LWSGGPIO_IRQ_RISING = 1 % UInt32
const LWSGGPIO_IRQ_FALLING = 2 % UInt32
const LWSGGPIO_IRQ_CHANGE = 3 % UInt32
const LWSGGPIO_IRQ_LOW = 4 % UInt32
const LWSGGPIO_IRQ_HIGH = 5 % UInt32

const __JL_Ctag_49 = UInt32
const LWSGGPIO_FL_READ = 1 % UInt32
const LWSGGPIO_FL_WRITE = 2 % UInt32
const LWSGGPIO_FL_PULLUP = 4 % UInt32
const LWSGGPIO_FL_PULLDOWN = 8 % UInt32
const LWSGGPIO_FL_START_LOW = 16 % UInt32

# typedef void ( * lws_gpio_irq_cb_t ) ( void * arg )
const lws_gpio_irq_cb_t = Ptr{Cvoid}

struct lws_gpio_ops
    mode::Ptr{Cvoid}
    read::Ptr{Cvoid}
    set::Ptr{Cvoid}
    irq_mode::Ptr{Cvoid}
end

const lws_gpio_ops_t = lws_gpio_ops

struct lws_bb_i2c
    bb_ops::lws_i2c_ops_t
    scl::_lws_plat_gpio_t
    sda::_lws_plat_gpio_t
    gpio::Ptr{lws_gpio_ops_t}
    delay::Ptr{Cvoid}
end

const lws_bb_i2c_t = lws_bb_i2c

struct lws_bb_spi
    bb_ops::lws_spi_ops_t
    gpio::Ptr{lws_gpio_ops_t}
    clk::_lws_plat_gpio_t
    ncs::NTuple{4, _lws_plat_gpio_t}
    ncmd::NTuple{4, _lws_plat_gpio_t}
    mosi::_lws_plat_gpio_t
    miso::_lws_plat_gpio_t
    flags::UInt8
end

const lws_bb_spi_t = lws_bb_spi

const lws_button_idx_t = UInt16

# typedef void ( * lws_button_cb_t ) ( void * opaque , lws_button_idx_t idx , int state )
const lws_button_cb_t = Ptr{Cvoid}

const __JL_Ctag_50 = UInt32
const LWSBTNRGMFLAG_CLASSIFY_DOUBLECLICK = 1 % UInt32

struct lws_button_regime
    ms_min_down::UInt16
    ms_min_down_longpress::UInt16
    ms_up_settle::UInt16
    ms_doubleclick_grace::UInt16
    ms_repeat_down::UInt16
    flags::UInt8
end

const lws_button_regime_t = lws_button_regime

struct lws_button_map
    gpio::_lws_plat_gpio_t
    smd_interaction_name::Ptr{Cchar}
    regime::Ptr{lws_button_regime_t}
end

const lws_button_map_t = lws_button_map

struct lws_button_controller
    smd_bc_name::Ptr{Cchar}
    gpio_ops::Ptr{lws_gpio_ops_t}
    button_map::Ptr{lws_button_map_t}
    active_state_bitmap::lws_button_idx_t
    count_buttons::UInt8
end

const lws_button_controller_t = lws_button_controller

mutable struct lws_button_state end

function lws_button_controller_create(ctx, controller)
    @ccall libwebsockets.lws_button_controller_create(ctx::Ptr{lws_context}, controller::Ptr{lws_button_controller_t})::Ptr{lws_button_state}
end

function lws_button_controller_destroy(bcs)
    @ccall libwebsockets.lws_button_controller_destroy(bcs::Ptr{lws_button_state})::Cvoid
end

function lws_button_get_bit(bcs, name)
    @ccall libwebsockets.lws_button_get_bit(bcs::Ptr{lws_button_state}, name::Ptr{Cchar})::lws_button_idx_t
end

function lws_button_enable(bcs, _reset, _set)
    @ccall libwebsockets.lws_button_enable(bcs::Ptr{lws_button_state}, _reset::lws_button_idx_t, _set::lws_button_idx_t)::Cvoid
end

const __JL_Ctag_51 = UInt32
const LLSI_CURR = 0 % UInt32
const LLSI_NEXT = 1 % UInt32
const LLSI_TRANS = 2 % UInt32

struct lws_led_state_ch
    seq::Ptr{lws_led_sequence_def_t}
    ph::lws_led_seq_phase_t
    step::lws_led_seq_phase_t
    phase_budget::Cint
    last::lws_led_intensity_t
end

const lws_led_state_ch_t = lws_led_state_ch

struct lws_led_state_chs
    seqs::NTuple{3, lws_led_state_ch_t}
end

const lws_led_state_chs_t = lws_led_state_chs

struct lws_led_gpio_map
    name::Ptr{Cchar}
    gpio::_lws_plat_gpio_t
    intensity_correction::lws_led_lookup_t
    pwm_ops::Ptr{lws_pwm_ops}
    active_level::UInt8
end

const lws_led_gpio_map_t = lws_led_gpio_map

struct lws_led_gpio_controller
    led_ops::lws_led_ops_t
    gpio_ops::Ptr{lws_gpio_ops_t}
    led_map::Ptr{lws_led_gpio_map_t}
    count_leds::UInt8
end

const lws_led_gpio_controller_t = lws_led_gpio_controller

function lws_led_transition(lcs, name, next, trans)
    @ccall libwebsockets.lws_led_transition(lcs::Ptr{lws_led_state}, name::Ptr{Cchar}, next::Ptr{lws_led_sequence_def_t}, trans::Ptr{lws_led_sequence_def_t})::Cint
end

function lws_led_func_linear(n)
    @ccall libwebsockets.lws_led_func_linear(n::lws_led_seq_phase_t)::lws_led_intensity_t
end

function lws_led_func_sine(n)
    @ccall libwebsockets.lws_led_func_sine(n::lws_led_seq_phase_t)::lws_led_intensity_t
end

const lws_display_t = lws_display

const lws_display_controller_state = UInt32
const LWSDISPS_OFF = 0 % UInt32
const LWSDISPS_AUTODIMMED = 1 % UInt32
const LWSDISPS_BECOMING_ACTIVE = 2 % UInt32
const LWSDISPS_ACTIVE = 3 % UInt32
const LWSDISPS_GOING_OFF = 4 % UInt32

struct lws_display_state
    sul_autodim::lws_sorted_usec_list_t
    disp::Ptr{lws_display_t}
    ctx::Ptr{lws_context}
    autodim_ms::Cint
    off_ms::Cint
    bl_lcs::Ptr{lws_led_state}
    chs::lws_led_state_chs_t
    state::lws_display_controller_state
end

const lws_display_state_t = lws_display_state

function lws_display_state_init(lds, ctx, autodim_ms, off_ms, bl_lcs, disp)
    @ccall libwebsockets.lws_display_state_init(lds::Ptr{lws_display_state_t}, ctx::Ptr{lws_context}, autodim_ms::Cint, off_ms::Cint, bl_lcs::Ptr{lws_led_state}, disp::Ptr{lws_display_t})::Cvoid
end

function lws_display_state_set_brightness(lds, pwmseq)
    @ccall libwebsockets.lws_display_state_set_brightness(lds::Ptr{lws_display_state_t}, pwmseq::Ptr{lws_led_sequence_def_t})::Cvoid
end

function lws_display_state_active(lds)
    @ccall libwebsockets.lws_display_state_active(lds::Ptr{lws_display_state_t})::Cvoid
end

function lws_display_state_off(lds)
    @ccall libwebsockets.lws_display_state_off(lds::Ptr{lws_display_state_t})::Cvoid
end

struct lws_display_ssd1306
    disp::lws_display_t
    i2c::Ptr{lws_i2c_ops_t}
    gpio::Ptr{lws_gpio_ops_t}
    reset_gpio::_lws_plat_gpio_t
    i2c7_address::UInt8
end

const lws_display_ssd1306_t = lws_display_ssd1306

struct lws_display_ili9341
    disp::lws_display_t
    spi::Ptr{lws_spi_ops_t}
    gpio::Ptr{lws_gpio_ops_t}
    reset_gpio::_lws_plat_gpio_t
    spi_index::UInt8
end

const lws_display_ili9341_t = lws_display_ili9341

const lws_settings_ops_t = lws_settings_ops

# automatic type deduction for variadic arguments may not be what you want, please use with caution
@generated function lws_settings_plat_printf(si, name, format, va_list...)
        :(@ccall(libwebsockets.lws_settings_plat_printf(si::Ptr{lws_settings_instance_t}, name::Ptr{Cchar}, format::Ptr{Cchar}; $(to_c_type_pairs(va_list)...))::Cint))
    end

function lws_settings_init(so, opaque_plat)
    @ccall libwebsockets.lws_settings_init(so::Ptr{lws_settings_ops_t}, opaque_plat::Ptr{Cvoid})::Ptr{lws_settings_instance_t}
end

function lws_settings_deinit(si)
    @ccall libwebsockets.lws_settings_deinit(si::Ptr{Ptr{lws_settings_instance_t}})::Cvoid
end

const lws_wifi_ch_t = UInt8

const lws_wifi_rssi_t = Int8

const lws_netdev_type_t = UInt32
const LWSNDTYP_UNKNOWN = 0 % UInt32
const LWSNDTYP_WIFI = 1 % UInt32
const LWSNDTYP_ETH = 2 % UInt32

struct lws_netdevs
    owner::lws_dll2_owner_t
    owner_creds::lws_dll2_owner_t
    ac_creds::Ptr{lwsac}
    si::Ptr{lws_settings_instance_t}
    sa46_dns_resolver::lws_sockaddr46
    refcount_creds::UInt8
end

const lws_netdevs_t = lws_netdevs

const __JL_Ctag_54 = UInt32
const LNDIW_ALG_OPEN = 0 % UInt32
const LNDIW_ALG_WPA2 = 1 % UInt32
const LNDIW_MODE_STA = 1 % UInt32
const LNDIW_MODE_AP = 2 % UInt32
const LNDIW_UP = 128 % UInt32
const LNDIW_ACQ_IPv4 = 1 % UInt32
const LNDIW_ACQ_IPv6 = 2 % UInt32

const lws_netdev_wifi_state_t = UInt32
const LWSNDVWIFI_STATE_INITIAL = 0 % UInt32
const LWSNDVWIFI_STATE_SCAN = 1 % UInt32
const LWSNDVWIFI_STATE_AP = 2 % UInt32
const LWSNDVWIFI_STATE_AP_SCAN = 3 % UInt32
const LWSNDVWIFI_STATE_STAT_GRP_AP = 4 % UInt32
const LWSNDVWIFI_STATE_STAT_GRP_AP_SCAN = 5 % UInt32
const LWSNDVWIFI_STATE_STAT = 6 % UInt32
const LWSNDVWIFI_STATE_STAT_HAPPY = 7 % UInt32

struct lws_wifi_creds
    list::lws_dll2_t
    bssid::NTuple{6, UInt8}
    passphrase::NTuple{64, Cchar}
    ssid::NTuple{33, Cchar}
    alg::UInt8
end

const lws_wifi_creds_t = lws_wifi_creds

struct lws_netdev_instance_wifi
    inst::lws_netdev_instance_t
    scan::lws_dll2_owner_t
    sul_scan::lws_sorted_usec_list_t
    ap_cred::Ptr{lws_wifi_creds_t}
    ap_ip::Ptr{Cchar}
    sta_ads::Ptr{Cchar}
    current_attempt_ssid::NTuple{33, Cchar}
    current_attempt_bssid::NTuple{6, UInt8}
    flags::UInt8
    state::UInt8
end

const lws_netdev_instance_wifi_t = lws_netdev_instance_wifi

struct lws_wifi_sta
    list::lws_dll2_t
    last_seen::UInt32
    last_tried::UInt32
    bssid::NTuple{6, UInt8}
    ssid::Ptr{Cchar}
    ssid_len::UInt8
    ch::lws_wifi_ch_t
    rssi::NTuple{8, lws_wifi_rssi_t}
    rssi_avg::Int16
    authmode::UInt8
    rssi_count::UInt8
    rssi_next::UInt8
end

const lws_wifi_sta_t = lws_wifi_sta

function lws_netdevs_from_ctx(ctx)
    @ccall libwebsockets.lws_netdevs_from_ctx(ctx::Ptr{lws_context})::Ptr{lws_netdevs_t}
end

function lws_netdev_credentials_settings_set(nds)
    @ccall libwebsockets.lws_netdev_credentials_settings_set(nds::Ptr{lws_netdevs_t})::Cint
end

function lws_netdev_credentials_settings_get(nds)
    @ccall libwebsockets.lws_netdev_credentials_settings_get(nds::Ptr{lws_netdevs_t})::Cint
end

function lws_netdev_find(netdevs, ifname)
    @ccall libwebsockets.lws_netdev_find(netdevs::Ptr{lws_netdevs_t}, ifname::Ptr{Cchar})::Ptr{lws_netdev_instance_t}
end

function lws_netdev_plat_init()
    @ccall libwebsockets.lws_netdev_plat_init()::Cint
end

function lws_netdev_plat_wifi_init()
    @ccall libwebsockets.lws_netdev_plat_wifi_init()::Cint
end

struct __JL_Ctag_59
    data::NTuple{16, UInt8}
end

function Base.getproperty(x::Ptr{__JL_Ctag_59}, f::Symbol)
    f === :ws && return Ptr{__JL_Ctag_60}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_59, f::Symbol)
    r = Ref{__JL_Ctag_59}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_59}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_59}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

struct __JL_Ctag_58
    data::NTuple{112, UInt8}
end

function Base.getproperty(x::Ptr{__JL_Ctag_58}, f::Symbol)
    f === :method && return Ptr{Ptr{Cchar}}(x + 0)
    f === :url && return Ptr{Ptr{Cchar}}(x + 8)
    f === :multipart_name && return Ptr{Ptr{Cchar}}(x + 16)
    f === :multipart_filename && return Ptr{Ptr{Cchar}}(x + 24)
    f === :multipart_content_type && return Ptr{Ptr{Cchar}}(x + 32)
    f === :blob_header && return Ptr{NTuple{4, Ptr{Cchar}}}(x + 40)
    f === :auth_preamble && return Ptr{Ptr{Cchar}}(x + 72)
    f === :respmap && return Ptr{Ptr{lws_ss_http_respmap_t}}(x + 80)
    f === :u && return Ptr{__JL_Ctag_59}(x + 88)
    f === :resp_expect && return Ptr{UInt16}(x + 104)
    f === :count_respmap && return Ptr{UInt8}(x + 106)
    f === :fail_redirect && return (Ptr{UInt8}(x + 104), 24, 1)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_58, f::Symbol)
    r = Ref{__JL_Ctag_58}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_58}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_58}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end

struct __JL_Ctag_60
    subprotocol::Ptr{Cchar}
    binary::UInt8
end
function Base.getproperty(x::Ptr{__JL_Ctag_60}, f::Symbol)
    f === :subprotocol && return Ptr{Ptr{Cchar}}(x + 0)
    f === :binary && return Ptr{UInt8}(x + 8)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_60, f::Symbol)
    r = Ref{__JL_Ctag_60}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_60}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_60}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end


struct __JL_Ctag_62
    cert::Ptr{lws_ss_x509_t}
    key::Ptr{lws_ss_x509_t}
end
function Base.getproperty(x::Ptr{__JL_Ctag_62}, f::Symbol)
    f === :cert && return Ptr{Ptr{lws_ss_x509_t}}(x + 0)
    f === :key && return Ptr{Ptr{lws_ss_x509_t}}(x + 8)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_62, f::Symbol)
    r = Ref{__JL_Ctag_62}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_62}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_62}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end


struct __JL_Ctag_65
    filepath::Ptr{Cchar}
end
function Base.getproperty(x::Ptr{__JL_Ctag_65}, f::Symbol)
    f === :filepath && return Ptr{Ptr{Cchar}}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_65, f::Symbol)
    r = Ref{__JL_Ctag_65}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_65}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_65}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end


struct __JL_Ctag_68
    response::Cint
end
function Base.getproperty(x::Ptr{__JL_Ctag_68}, f::Symbol)
    f === :response && return Ptr{Cint}(x + 0)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_68, f::Symbol)
    r = Ref{__JL_Ctag_68}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_68}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_68}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end


struct __JL_Ctag_71
    sum::NTuple{2, u_mt_t}
    min::u_mt_t
    max::u_mt_t
    count::NTuple{2, UInt32}
end
function Base.getproperty(x::Ptr{__JL_Ctag_71}, f::Symbol)
    f === :sum && return Ptr{NTuple{2, u_mt_t}}(x + 0)
    f === :min && return Ptr{u_mt_t}(x + 16)
    f === :max && return Ptr{u_mt_t}(x + 24)
    f === :count && return Ptr{NTuple{2, UInt32}}(x + 32)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_71, f::Symbol)
    r = Ref{__JL_Ctag_71}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_71}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_71}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end


struct __JL_Ctag_72
    head::Ptr{lws_metric_bucket_t}
    total_count::UInt64
    list_size::UInt32
end
function Base.getproperty(x::Ptr{__JL_Ctag_72}, f::Symbol)
    f === :head && return Ptr{Ptr{lws_metric_bucket_t}}(x + 0)
    f === :total_count && return Ptr{UInt64}(x + 8)
    f === :list_size && return Ptr{UInt32}(x + 16)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_72, f::Symbol)
    r = Ref{__JL_Ctag_72}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_72}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_72}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end


struct __JL_Ctag_73
    len::Cint
    name::NTuple{64, Cchar}
end
function Base.getproperty(x::Ptr{__JL_Ctag_73}, f::Symbol)
    f === :len && return Ptr{Cint}(x + 0)
    f === :name && return Ptr{NTuple{64, Cchar}}(x + 4)
    return getfield(x, f)
end

function Base.getproperty(x::__JL_Ctag_73, f::Symbol)
    r = Ref{__JL_Ctag_73}(x)
    ptr = Base.unsafe_convert(Ptr{__JL_Ctag_73}, r)
    fptr = getproperty(ptr, f)
    GC.@preserve r unsafe_load(fptr)
end

function Base.setproperty!(x::Ptr{__JL_Ctag_73}, f::Symbol, v)
    unsafe_store!(getproperty(x, f), v)
end


const LWS_INSTALL_DATADIR = "/workspace/destdir/share"

const LWS_INSTALL_LIBDIR = "/workspace/destdir/lib"

const LWS_LIBRARY_VERSION_MAJOR = 4

const LWS_LIBRARY_VERSION_MINOR = 3

# Skipping MacroDefinition: LWS_LIBRARY_VERSION_PATCH_ELABORATED 3 - v4 .3.3 - 42 - g5102a5c8

const LWS_LIBRARY_VERSION_PATCH = 3

const LWS_LIBRARY_VERSION_NUMBER = LWS_LIBRARY_VERSION_MAJOR * 1000000 + LWS_LIBRARY_VERSION_MINOR * 1000 + LWS_LIBRARY_VERSION_PATCH

const LWS_MAX_SMP = 1

const LWS_BUILD_HASH = "v4.3.3-42-g5102a5c8"

const LWS_LIBRARY_VERSION = "4.3.3-v4.3.3-42-g5102a5c8"

const LWS_LOGGING_BITFIELD_CLEAR = 0

const LWS_LOGGING_BITFIELD_SET = 0

const LWS_OPENSSL_CLIENT_CERTS = "../share"

const LWS_US_PER_SEC = lws_usec_t(1000000)

const LWS_MS_PER_SEC = lws_usec_t(1000)

const LWS_US_PER_MS = lws_usec_t(1000)

const LWS_NS_PER_US = lws_usec_t(1000)

const LWS_KI = 1024

const LWS_MI = LWS_KI * 1024

const LWS_GI = LWS_MI * 1024

const LWS_TI = uint64_t(LWS_GI) * 1024

const LWS_PI = uint64_t(LWS_TI) * 1024

const lws_intptr_t = intptr_t

# Skipping MacroDefinition: LWS_INLINE inline

const LWS_INVALID_FILE = -1

const LWS_SOCK_INVALID = -1

# Skipping MacroDefinition: LWS_WARN_UNUSED_RESULT __attribute__ ( ( warn_unused_result ) )

# Skipping MacroDefinition: LWS_WARN_DEPRECATED __attribute__ ( ( deprecated ) )

const CONTEXT_PORT_NO_LISTEN = -1

const CONTEXT_PORT_NO_LISTEN_SERVER = -2

const LLL_ERR = 1 << 0

const LLL_WARN = 1 << 1

const LLL_NOTICE = 1 << 2

const LLL_INFO = 1 << 3

const LLL_DEBUG = 1 << 4

const LLL_PARSER = 1 << 5

const LLL_HEADER = 1 << 6

const LLL_EXT = 1 << 7

const LLL_CLIENT = 1 << 8

const LLL_LATENCY = 1 << 9

const LLL_USER = 1 << 10

const LLL_THREAD = 1 << 11

const LLL_COUNT = 12

const LLLF_SECRECY_PII = 1 << 16

const LLLF_SECRECY_BEARER = 1 << 17

const LLLF_LOG_TIMESTAMP = 1 << 18

const LLLF_LOG_CONTEXT_AWARE = 1 << 30

const _LWS_LINIT = 1 << LLL_COUNT - 1

const _LWS_LBS = LWS_LOGGING_BITFIELD_SET

const _LWS_LBC = LWS_LOGGING_BITFIELD_CLEAR

const _LWS_ENABLED_LOGS = (_LWS_LINIT | _LWS_LBS) & ~_LWS_LBC

const lws_pollfd = pollfd

const LWS_POLLHUP = POLLHUP | POLLERR

const LWS_POLLIN = POLLIN

const LWS_POLLOUT = POLLOUT

const lws_time_in_microseconds = lws_now_usecs

const LWS_TO_KILL_ASYNC = -1

const LWS_TO_KILL_SYNC = -2

const LWS_SET_TIMER_USEC_CANCEL = lws_usec_t(Clonglong(-1))

const LWS_USEC_PER_SEC = lws_usec_t(1000000)

const LWS_COUNT_PT_SUL_OWNERS = 2

const LWSSULLI_MISS_IF_SUSPENDED = 0

const LWSSULLI_WAKE_IF_SUSPENDED = 1

const LWS_SMD_MAX_PAYLOAD = 384

const LWS_SMD_CLASS_BITFIELD_BYTES = 4

const LWS_SMD_STREAMTYPENAME = "_lws_smd"

const LWS_SMD_SS_RX_HEADER_LEN = 16

const LWSSMDREG_FLAG_PROXIED_SS = 1 << 0

const LWS_RETRY_CONCEAL_ALWAYS = 0xffff

const LWS_CAUDP_BIND = 1 << 0

const LWS_CAUDP_BROADCAST = 1 << 1

const LWS_CAUDP_PF_PACKET = 1 << 2

const LWS_ITOSA_USABLE = 0

const LWS_ITOSA_NOT_EXIST = -1

const LWS_ITOSA_NOT_USABLE = -2

const LWS_ITOSA_BUSY = -3

const METRES_GO = 0

const METRES_NOGO = 1

const LWSSYSGAUTH_HEX = 1 << 0

const LWS_CB_REASON_AUX_BF__CGI = 1

const LWS_CB_REASON_AUX_BF__PROXY = 2

const LWS_CB_REASON_AUX_BF__CGI_CHUNK_END = 4

const LWS_CB_REASON_AUX_BF__CGI_HEADERS = 8

const LWS_CB_REASON_AUX_BF__PROXY_TRANS_END = 16

const LWS_CB_REASON_AUX_BF__PROXY_HEADERS = 32

const LWS_PROTOCOL_LIST_TERM = {NULL, NULL, 0, 0, 0, NULL, 0}

const LWS_PLUGIN_API_MAGIC = 191

const LWS_SERVER_OPTION_REQUIRE_VALID_OPENSSL_CLIENT_CERT = Clonglong(1) << 1 | Clonglong(1) << 12

const LWS_SERVER_OPTION_SKIP_SERVER_CANONICAL_NAME = Clonglong(1) << 2

const LWS_SERVER_OPTION_ALLOW_NON_SSL_ON_SSL_PORT = Clonglong(1) << 3 | Clonglong(1) << 12

const LWS_SERVER_OPTION_LIBEV = Clonglong(1) << 4

const LWS_SERVER_OPTION_DISABLE_IPV6 = Clonglong(1) << 5

const LWS_SERVER_OPTION_DISABLE_OS_CA_CERTS = Clonglong(1) << 6

const LWS_SERVER_OPTION_PEER_CERT_NOT_REQUIRED = Clonglong(1) << 7

const LWS_SERVER_OPTION_VALIDATE_UTF8 = Clonglong(1) << 8

const LWS_SERVER_OPTION_SSL_ECDH = Clonglong(1) << 9 | Clonglong(1) << 12

const LWS_SERVER_OPTION_LIBUV = Clonglong(1) << 10

const LWS_SERVER_OPTION_REDIRECT_HTTP_TO_HTTPS = Clonglong(1) << 11 | Clonglong(1) << 12

const LWS_SERVER_OPTION_DO_SSL_GLOBAL_INIT = Clonglong(1) << 12

const LWS_SERVER_OPTION_EXPLICIT_VHOSTS = Clonglong(1) << 13

const LWS_SERVER_OPTION_UNIX_SOCK = Clonglong(1) << 14

const LWS_SERVER_OPTION_STS = Clonglong(1) << 15

const LWS_SERVER_OPTION_IPV6_V6ONLY_MODIFY = Clonglong(1) << 16

const LWS_SERVER_OPTION_IPV6_V6ONLY_VALUE = Clonglong(1) << 17

const LWS_SERVER_OPTION_UV_NO_SIGSEGV_SIGFPE_SPIN = Clonglong(1) << 18

const LWS_SERVER_OPTION_JUST_USE_RAW_ORIGIN = Clonglong(1) << 19

# Skipping MacroDefinition: LWS_SERVER_OPTION_FALLBACK_TO_RAW /* use below name */ ( 1ll << 20 )

const LWS_SERVER_OPTION_FALLBACK_TO_APPLY_LISTEN_ACCEPT_CONFIG = Clonglong(1) << 20

const LWS_SERVER_OPTION_LIBEVENT = Clonglong(1) << 21

# Skipping MacroDefinition: LWS_SERVER_OPTION_ONLY_RAW /* Use below name instead */ ( 1ll << 22 )

const LWS_SERVER_OPTION_ADOPT_APPLY_LISTEN_ACCEPT_CONFIG = Clonglong(1) << 22

const LWS_SERVER_OPTION_ALLOW_LISTEN_SHARE = Clonglong(1) << 23

const LWS_SERVER_OPTION_CREATE_VHOST_SSL_CTX = Clonglong(1) << 24

const LWS_SERVER_OPTION_SKIP_PROTOCOL_INIT = Clonglong(1) << 25

const LWS_SERVER_OPTION_IGNORE_MISSING_CERT = Clonglong(1) << 26

const LWS_SERVER_OPTION_VHOST_UPG_STRICT_HOST_CHECK = Clonglong(1) << 27

const LWS_SERVER_OPTION_HTTP_HEADERS_SECURITY_BEST_PRACTICES_ENFORCE = Clonglong(1) << 28

const LWS_SERVER_OPTION_ALLOW_HTTP_ON_HTTPS_LISTENER = Clonglong(1) << 29

const LWS_SERVER_OPTION_FAIL_UPON_UNABLE_TO_BIND = Clonglong(1) << 30

const LWS_SERVER_OPTION_H2_JUST_FIX_WINDOW_UPDATE_OVERFLOW = Clonglong(1) << 31

const LWS_SERVER_OPTION_VH_H2_HALF_CLOSED_LONG_POLL = Clonglong(1) << 32

const LWS_SERVER_OPTION_GLIB = Clonglong(1) << 33

const LWS_SERVER_OPTION_H2_PRIOR_KNOWLEDGE = Clonglong(1) << 34

const LWS_SERVER_OPTION_NO_LWS_SYSTEM_STATES = Clonglong(1) << 35

const LWS_SERVER_OPTION_SS_PROXY = Clonglong(1) << 36

const LWS_SERVER_OPTION_SDEVENT = Clonglong(1) << 37

const LWS_SERVER_OPTION_ULOOP = Clonglong(1) << 38

const LWS_SERVER_OPTION_DISABLE_TLS_SESSION_CACHE = Clonglong(1) << 39

const AUTH_MODE_MASK = 0xf0000000

const LWS_RECOMMENDED_MIN_HEADER_SPACE = 2048

const LWSAHH_CODE_MASK = 1 << 16 - 1

const LWSAHH_FLAG_NO_SERVER_NAME = 1 << 30

const LWS_ILLEGAL_HTTP_CONTENT_LEN = lws_filepos_t(Clonglong(-1))

const LWS_H2_STREAM_SID = -1

const lws_plat_service_tsi = lws_service_tsi

# Skipping MacroDefinition: LWS_SIZEOFPTR ( ( int ) sizeof ( void * ) )

const LWS_PRE = _LWS_PAD(4 + 10 + 2)

const LWS_SEND_BUFFER_PRE_PADDING = LWS_PRE

const LWS_SEND_BUFFER_POST_PADDING = 0

const LWS_WRITE_RAW = LWS_WRITE_HTTP

const LWSTXCR_US_TO_PEER = 0

const LWSTXCR_PEER_TO_US = 1

const LWS_FOP_OPEN = open

const LWS_FOP_CLOSE = close

const LWS_FOP_SEEK_CUR = seek_cur

const LWS_FOP_READ = read

const LWS_FOP_WRITE = write

const LWS_FOP_FLAGS_MASK = 1 << 23 - 1

const LWS_FOP_FLAG_COMPR_ACCEPTABLE_GZIP = 1 << 24

const LWS_FOP_FLAG_COMPR_IS_GZIP = 1 << 25

const LWS_FOP_FLAG_MOD_TIME_VALID = 1 << 26

const LWS_FOP_FLAG_VIRTUAL = 1 << 27

const LWS_GENCRYPTO_MAX_KEYEL_COUNT = LWS_GENCRYPTO_RSA_KEYEL_COUNT

const LEJP_FLAG_WS_KEEP = 64

const LEJP_FLAG_WS_COMMENTLINE = 32

const LEJP_FLAG_CB_IS_VALUE = 64

const LEJP_MAX_PARSING_STACK_DEPTH = 5

const LEJP_MAX_DEPTH = 12

const LEJP_MAX_INDEX_DEPTH = 8

const LEJP_MAX_PATH = 128

const LEJP_STRING_CHUNK = 254

const LECP_MAX_PARSING_STACK_DEPTH = 5

const LECP_MAX_DEPTH = 12

const LECP_MAX_INDEX_DEPTH = 8

const LECP_MAX_PATH = 128

const LECP_STRING_CHUNK = 254

const LECP_FLAG_CB_IS_VALUE = 64

const LCSC_FL_ADD_CBOR_TAG = 1 << 0

const LCSC_FL_ADD_CBOR_PREFER_MAC0 = 1 << 1

const LWS_TOKENIZE_F_MINUS_NONTERM = 1 << 0

const LWS_TOKENIZE_F_AGG_COLON = 1 << 1

const LWS_TOKENIZE_F_COMMA_SEP_LIST = 1 << 2

const LWS_TOKENIZE_F_RFC7230_DELIMS = 1 << 3

const LWS_TOKENIZE_F_DOT_NONTERM = 1 << 4

const LWS_TOKENIZE_F_NO_FLOATS = 1 << 5

const LWS_TOKENIZE_F_NO_INTEGERS = 1 << 6

const LWS_TOKENIZE_F_HASH_COMMENT = 1 << 7

const LWS_TOKENIZE_F_SLASH_NONTERM = 1 << 8

const LWS_TOKENIZE_F_ASTERISK_NONTERM = 1 << 9

const LWS_TOKENIZE_F_EQUALS_NONTERM = 1 << 10

const lwsac_use_zeroed = lwsac_use_zero

const LWSFTS_F_QUERY_AUTOCOMPLETE = 1 << 0

const LWSFTS_F_QUERY_FILES = 1 << 1

const LWSFTS_F_QUERY_FILE_LINES = 1 << 2

const LWSFTS_F_QUERY_QUOTE_LINE = 1 << 3

const LWS_DISKCACHE_QUERY_NO_CACHE = 0

const LWS_DISKCACHE_QUERY_EXISTS = 1

const LWS_DISKCACHE_QUERY_CREATING = 2

const LWS_DISKCACHE_QUERY_ONGOING = 3

const LWSSEQTO_NONE = 0

const LWS_SS_MTU = 1540

const LWSSS_TIMEOUT_FROM_POLICY = 0

const LWS_SESSION_TAG_LEN = 96

const LWS_GENHASH_LARGEST = 64

const LWS_AES_BLOCKSIZE = 128

const LWS_AES_CBC_BLOCKLEN = 16

const LWSJWKF_EXPORT_PRIVATE = 1 << 0

const LWSJWKF_EXPORT_NOCRLF = 1 << 1

const LWS_JWS_MAX_RECIPIENTS = 3

const LWS_JWS_MAX_SIGS = 3

const LWS_JWE_RFC3394_OVERHEAD_BYTES = 8

const LWS_JWE_AES_IV_BYTES = 16

const LWS_JWE_LIMIT_RSA_KEY_BITS = 4096

const LWS_JWE_LIMIT_AES_KEY_BITS = 512 + 64

const LWS_JWE_LIMIT_EC_KEY_BITS = 528

const LWS_JWE_LIMIT_HASH_BITS = LWS_GENHASH_LARGEST * 8

const LWS_JWE_LIMIT_KEY_ELEMENT_BYTES = LWS_JWE_LIMIT_RSA_KEY_BITS ÷ 8

# Skipping MacroDefinition: lws_bb_i2c_ops { . init = lws_bb_i2c_init , . start = lws_bb_i2c_start , . stop = lws_bb_i2c_stop , . write = lws_bb_i2c_write , . read = lws_bb_i2c_read , . set_ack = lws_bb_i2c_set_ack , }

const LWSBBSPI_FLAG_USE_NCMD3 = 1 << 7

const LWSBBSPI_FLAG_USE_NCMD2 = 1 << 6

const LWSBBSPI_FLAG_USE_NCMD1 = 1 << 5

const LWSBBSPI_FLAG_USE_NCMD0 = 1 << 4

const LWSBBSPI_FLAG_USE_NCS3 = 1 << 3

const LWSBBSPI_FLAG_USE_NCS2 = 1 << 2

const LWSBBSPI_FLAG_USE_NCS1 = 1 << 1

const LWSBBSPI_FLAG_USE_NCS0 = 1 << 0

const LWS_SPI_BB_MAX_CH = 4

# Skipping MacroDefinition: lws_bb_spi_ops . init = lws_bb_spi_init , . queue = lws_bb_spi_queue

const LWS_BUTTON_MON_TIMER_MS = 5

const LWS_LED_MAX_INTENSITY = 0xffff

const LWS_LED_FUNC_PHASE = 65536

const LWS_SEQ_LEDPHASE_TOTAL_ENDLESS = -1

const LWS_LED_SEQUENCER_UPDATE_INTERVAL_MS = 33

# Skipping MacroDefinition: lws_led_gpio_ops { . create = lws_led_gpio_create , . destroy = lws_led_gpio_destroy , . intensity = lws_led_gpio_intensity , }

# Skipping MacroDefinition: lws_pwm_plat_ops . init = lws_pwm_plat_init , . intensity = lws_pwm_plat_intensity

const SSD1306_I2C7_ADS1 = 0x3c

const SSD1306_I2C7_ADS2 = 0x3d

# Skipping MacroDefinition: lws_display_ssd1306_ops . init = lws_display_ssd1306_i2c_init , . contrast = lws_display_ssd1306_i2c_contrast , . blit = lws_display_ssd1306_i2c_blit , . power = lws_display_ssd1306_i2c_power

# Skipping MacroDefinition: lws_display_ili9341_ops . init = lws_display_ili9341_spi_init , . blit = lws_display_ili9341_spi_blit , . power = lws_display_ili9341_spi_power

const LSOOPEN_FLAG_WRITEABLE = 1 << 0

# Skipping MacroDefinition: lws_settings_ops_plat . get = lws_settings_plat_get , . set = lws_settings_plat_set ,

const LWS_WIFI_MAX_SCAN_TRACK = 16

const LWS_ETH_ALEN = 6

# Skipping MacroDefinition: lws_netdev_wifi_plat_ops . create = lws_netdev_wifi_create_plat , . configure = lws_netdev_wifi_configure_plat , . event = lws_netdev_wifi_event_plat , . up = lws_netdev_wifi_up_plat , . down = lws_netdev_wifi_down_plat , . connect = lws_netdev_wifi_connect_plat , . scan = lws_netdev_wifi_scan_plat , . destroy = lws_netdev_wifi_destroy_plat

# exports
const PREFIXES = ["lws_", "LWS"]
for name in names(@__MODULE__; all=true), prefix in PREFIXES
    if startswith(string(name), prefix)
        @eval export $name
    end
end

