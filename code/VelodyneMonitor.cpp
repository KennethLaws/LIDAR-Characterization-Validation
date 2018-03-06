#include "VelodyneMonitor.h"

#include <iostream>
#include <cstring>

namespace velodyne
{

const uint16_t udp_port = 2368;
const uint16_t position_port = 8308;
const int CLOSED_SOCKET = -1;
const int defaultProtocol = 0;

// This could immediately throw a socket exception.
VelodyneMonitor::VelodyneMonitor(struct in_addr ip_address, SocketErrorHandler socket_error_handler,
        TimeoutHandler timeout_handler)
    : _ip_address(ip_address), _laser_packet_socket(CLOSED_SOCKET), _position_socket(CLOSED_SOCKET), _thread(NULL),
      _socket_error_handler(socket_error_handler), _timeout_handler(timeout_handler)
{
    assert(_socket_error_handler);
    assert(_timeout_handler);
    _laser_packet_socket = socket(PF_INET, SOCK_DGRAM, defaultProtocol);

    if (_laser_packet_socket == -1) {
        throw SocketError(errno);
    }

    int socket_option_value = 1;
    int result = setsockopt(_laser_packet_socket, SOL_SOCKET, SO_REUSEADDR,
                 &socket_option_value, sizeof socket_option_value);

    if (result == -1) {
        close(_laser_packet_socket);
        _laser_packet_socket = CLOSED_SOCKET;
        throw SocketError(errno);
    };

    /*
     * Retrieve the receive buffer default size, for logging purposes,
     * and then increase the size of the receive buffer to at least
     * one second's worth of LaserPackets.
     */
    socket_option_value = 0;

    socklen_t socket_option_length = static_cast<socklen_t>(socket_option_value);

    result = getsockopt(_laser_packet_socket, SOL_SOCKET, SO_RCVBUF,
                        &socket_option_value, &socket_option_length);

    socket_option_value = sizeof(LaserPacket) * 1808;

    result = setsockopt(_laser_packet_socket, SOL_SOCKET, SO_RCVBUF,
                        &socket_option_value, sizeof socket_option_value);

    {
        sockaddr_in my_addr;
        memset(&my_addr, 0, sizeof(my_addr));
        my_addr.sin_family = AF_INET;
        my_addr.sin_port = htons(udp_port);
        my_addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(_laser_packet_socket, reinterpret_cast<sockaddr *>(&my_addr), sizeof(sockaddr)) == -1) {
            close(_laser_packet_socket);
            _laser_packet_socket = CLOSED_SOCKET;
            throw SocketError(errno);
        }
    }

    _position_socket = socket(PF_INET, SOCK_DGRAM, 0);

    if (_position_socket == -1) {
        throw SocketError(errno);
    }

    if (setsockopt(_position_socket, SOL_SOCKET, SO_REUSEADDR,
                   &socket_option_value, sizeof socket_option_value) == -1) {
        close(_position_socket);
        _position_socket = CLOSED_SOCKET;
        throw SocketError(errno);
    }

    {
        sockaddr_in my_addr;
        memset(&my_addr, 0, sizeof(my_addr));
        my_addr.sin_family = AF_INET;
        my_addr.sin_port = htons(position_port);
        my_addr.sin_addr.s_addr = INADDR_ANY;

        if (bind(_position_socket, reinterpret_cast<sockaddr *>(&my_addr), sizeof(sockaddr)) == -1) {
            close(_position_socket);
            _position_socket = CLOSED_SOCKET;
            throw SocketError(errno);
        }
    }
}

VelodyneMonitor::~VelodyneMonitor()
{
    // Make sure the thread is stopped.
    stop();
}

void VelodyneMonitor::start()
{
    assert(!_thread);
    _terminate = false;
    _thread = new std::thread(&VelodyneMonitor::run, this);
}

void VelodyneMonitor::stop()
{
    if (!_thread) {
        return;
    }

    _terminate = true;

    _thread->join();
    close(_laser_packet_socket);
    _laser_packet_socket = CLOSED_SOCKET;
    close(_position_socket);
    _position_socket = CLOSED_SOCKET;
    delete _thread;
    _thread = NULL;
}

void VelodyneMonitor::run()
{
    struct sockaddr source_address;
    socklen_t source_address_length = sizeof source_address;
    struct pollfd fds[2] = {
        { _laser_packet_socket, POLLIN, 0 },
        { _position_socket, POLLIN, 0 }
    };
     // The source of the data for which this function poll()-s spews that data at a rate of
     // 1,808 times each second or once every 553.1 microseconds. In theory, if not in practice,
     // this function should never have to wait one millisecond to retrieve the next message.
    static const int poll_timeout_milliseconds = 1;

    while (!_terminate) {
        try {
            int retval = 0; // Default to no events.

            while (retval <= 0) {
                retval = poll(fds, 2, poll_timeout_milliseconds);

                if (retval < 0) {
                    if (errno != EINTR) {
                        throw SocketError(errno);
                    }
                }

                if (retval == 0) {
                    // Poll timeout
                    throw Timeout();
                }
            }

            if (fds[0].revents & POLLIN) {
                LaserPacket potential_packet;
                ssize_t nbytes = recvfrom(_laser_packet_socket, &potential_packet, sizeof potential_packet, 0,
                                 &source_address, &source_address_length);

                if (source_address.sa_family == AF_INET) {
                    sockaddr_in * socket_address = reinterpret_cast<sockaddr_in *>(&source_address);

                    if (socket_address->sin_addr.s_addr == _ip_address.s_addr) {
                        if (nbytes < 0) {
                            if (errno != EWOULDBLOCK) {
                                throw SocketError(errno);
                            }
                        } else if (nbytes == sizeof potential_packet) {
                            _laser_packet_queue.push(potential_packet);
                        } else {
                            // Implicit here is that we drop incomplete packets.
                        }
                    }
                }
            } else if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) {
                throw SocketError(errno);
            }

            if (fds[1].revents & POLLIN) {
                PositioningPacket potential_packet;
                ssize_t nbytes = recvfrom(_position_socket, &potential_packet, sizeof potential_packet, 0,
                                 &source_address, &source_address_length);

                if (source_address.sa_family == AF_INET) {
                    sockaddr_in * socket_address = reinterpret_cast<sockaddr_in *>(&source_address);

                    if (socket_address->sin_addr.s_addr == _ip_address.s_addr) {
                        if (nbytes < 0) {
                            if (errno != EWOULDBLOCK) {
                                throw SocketError(errno);
                            }
                        } else if (nbytes == sizeof potential_packet) {
                            _position_queue.push(potential_packet);
                        } else {
                            // Implicit here is that we drop incomplete packets.
                        }
                    }
                }
            } else if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) {
                throw SocketError(errno);
            }
        } catch (SocketError & e) {
            LaserPacket p;
            memset(&p, 0, sizeof p);
            _laser_packet_queue.push(p);
            PositioningPacket q;
            memset(&q, 0, sizeof q);
            q.unused1[0] = 1;
            _position_queue.push(q);
            _socket_error_handler(e);
            return;
        } catch (Timeout & e) {
            _timeout_handler(e);
        }
    }

    // Poison the queue to indicate we're done.
    LaserPacket p;
    memset(&p, 0, sizeof p);
    _laser_packet_queue.push(p);
    PositioningPacket q;
    memset(&q, 0, sizeof q);
    q.unused1[0] = 1;
    _position_queue.push(q);
}

};

