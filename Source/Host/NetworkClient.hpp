#ifndef NETWORK_CLIENT_HPP_
#define NETWORK_CLIENT_HPP_

#include <string>
#include <optional>

#include <zmq.hpp>
#include <zmq_addon.hpp>

class NetworkClient
{
private:
    zmq::context_t Context;
    zmq::socket_t Socket;

public:
    NetworkClient(const std::string& URL);

    // Non-copyable
    NetworkClient(const NetworkClient&) = delete;
    NetworkClient& operator=(const NetworkClient&) = delete;

    // Movable
    NetworkClient(NetworkClient&&) = default;
    NetworkClient& operator=(NetworkClient&&) = default;

    // Send (non-blocking)
    bool Send(const std::string& data);

    // Receive (non-blocking) (unused for now)
    std::optional<std::string> Receive();

    ~NetworkClient();
};

#endif // NETWORK_CLIENT_HPP_
