/*

Copyright (c) 2019, Arvid Norberg
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef TORRENT_POLYMORPHIC_SOCKET
#define TORRENT_POLYMORPHIC_SOCKET

#include "libtorrent/aux_/disable_warnings_push.hpp"
#include <boost/variant/variant.hpp>
#include <boost/variant/get.hpp>
#include "libtorrent/aux_/disable_warnings_pop.hpp"

namespace libtorrent {
namespace aux {

#define TORRENT_SOCKTYPE_FORWARD(x) \
	return boost::apply_visitor([&](auto& s){ return s.x; }, *this)

	template <typename... Sockets>
	struct TORRENT_EXTRA_EXPORT polymorphic_socket
		: boost::variant<Sockets...>
	{
		using endpoint_type = tcp::socket::endpoint_type;
		using protocol_type = tcp::socket::protocol_type;

		using receive_buffer_size = tcp::socket::receive_buffer_size;
		using send_buffer_size = tcp::socket::send_buffer_size;

		template <typename S>
		explicit polymorphic_socket(S s) : boost::variant<Sockets...>(std::move(s)) {}
		polymorphic_socket(polymorphic_socket&&) = default;
		~polymorphic_socket() = default;

		bool is_open() const
		{ TORRENT_SOCKTYPE_FORWARD(is_open()); }

		void open(protocol_type const& p, error_code& ec)
		{ TORRENT_SOCKTYPE_FORWARD(open(p, ec)); }

		void close(error_code& ec)
		{ TORRENT_SOCKTYPE_FORWARD(close(ec)); }

		endpoint_type local_endpoint(error_code& ec) const
		{ TORRENT_SOCKTYPE_FORWARD(local_endpoint(ec)); }

		endpoint_type remote_endpoint(error_code& ec) const
		{ TORRENT_SOCKTYPE_FORWARD(remote_endpoint(ec)); }

		void bind(endpoint_type const& endpoint, error_code& ec)
		{ TORRENT_SOCKTYPE_FORWARD(bind(endpoint, ec)); }

		std::size_t available(error_code& ec) const
		{ TORRENT_SOCKTYPE_FORWARD(available(ec)); }

#ifndef BOOST_NO_EXCEPTIONS
		void open(protocol_type const& p)
		{ TORRENT_SOCKTYPE_FORWARD(open(p)); }

		void close()
		{ TORRENT_SOCKTYPE_FORWARD(close()); }

		endpoint_type local_endpoint() const
		{ TORRENT_SOCKTYPE_FORWARD(local_endpoint()); }

		endpoint_type remote_endpoint() const
		{ TORRENT_SOCKTYPE_FORWARD(remote_endpoint()); }

		void bind(endpoint_type const& endpoint)
		{ TORRENT_SOCKTYPE_FORWARD(bind(endpoint)); }

		std::size_t available() const
		{ TORRENT_SOCKTYPE_FORWARD(available()); }
#endif

		template <class Mutable_Buffers>
		std::size_t read_some(Mutable_Buffers const& buffers, error_code& ec)
		{ TORRENT_SOCKTYPE_FORWARD(read_some(buffers, ec)); }

		template <class Mutable_Buffers, class Handler>
		void async_read_some(Mutable_Buffers const& buffers, Handler const& handler)
		{ TORRENT_SOCKTYPE_FORWARD(async_read_some(buffers, handler)); }

		template <class Const_Buffers>
		std::size_t write_some(Const_Buffers const& buffers, error_code& ec)
		{ TORRENT_SOCKTYPE_FORWARD(write_some(buffers, ec)); }

		template <class Const_Buffers, class Handler>
		void async_write_some(Const_Buffers const& buffers, Handler const& handler)
		{ TORRENT_SOCKTYPE_FORWARD(async_write_some(buffers, handler)); }

		template <class Handler>
		void async_connect(endpoint_type const& endpoint, Handler const& handler)
		{ TORRENT_SOCKTYPE_FORWARD(async_connect(endpoint, handler)); }

#ifndef BOOST_NO_EXCEPTIONS
		template <class IO_Control_Command>
		void io_control(IO_Control_Command& ioc)
		{ TORRENT_SOCKTYPE_FORWARD(io_control(ioc)); }

		template <class Mutable_Buffers>
		std::size_t read_some(Mutable_Buffers const& buffers)
		{ TORRENT_SOCKTYPE_FORWARD(read_some(buffers)); }
#endif

		template <class IO_Control_Command>
		void io_control(IO_Control_Command& ioc, error_code& ec)
		{ TORRENT_SOCKTYPE_FORWARD(io_control(ioc, ec)); }

#ifndef BOOST_NO_EXCEPTIONS
		template <class SettableSocketOption>
		void set_option(SettableSocketOption const& opt)
		{ TORRENT_SOCKTYPE_FORWARD(set_option(opt)); }
#endif

		template <class SettableSocketOption>
		void set_option(SettableSocketOption const& opt, error_code& ec)
		{ TORRENT_SOCKTYPE_FORWARD(set_option(opt, ec)); }

		void non_blocking(bool b, error_code& ec)
		{ TORRENT_SOCKTYPE_FORWARD(non_blocking(b, ec)); }

#ifndef BOOST_NO_EXCEPTIONS
		void non_blocking(bool b)
		{ TORRENT_SOCKTYPE_FORWARD(non_blocking(b)); }
#endif

#ifndef BOOST_NO_EXCEPTIONS
		template <class GettableSocketOption>
		void get_option(GettableSocketOption& opt)
		{ TORRENT_SOCKTYPE_FORWARD(get_option(opt)); }
#endif

		template <class GettableSocketOption>
		void get_option(GettableSocketOption& opt, error_code& ec)
		{ TORRENT_SOCKTYPE_FORWARD(get_option(opt, ec)); }

		// explicitly disallow assignment, to silence msvc warning
		polymorphic_socket& operator=(polymorphic_socket const&) = delete;
		polymorphic_socket& operator=(polymorphic_socket&&) = default;
	};

#undef TORRENT_SOCKTYPE_FORWARD

}
}

#endif // TORRENT_POLYMORPHIC_SOCKET
