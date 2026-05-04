#include "NetworkClient.hpp"

NetworkClient::NetworkClient(const std::string& URL)
{
    this->Context = zmq::context_t(1);
    this->Socket = zmq::socket_t(this->Context, zmq::socket_type::dealer);

    this->Socket.set(zmq::sockopt::linger, 0);

    this->Socket.connect(URL);
}

bool NetworkClient::Send(const std::string &data)
{
    zmq::message_t Message(data.cbegin(), data.cend());

    auto Result = this->Socket.send(Message, zmq::send_flags::dontwait);

    return Result.has_value();
}

std::optional<std::string> NetworkClient::Receive()
{
    zmq::message_t Message;

    auto Result = this->Socket.recv(Message, zmq::recv_flags::dontwait);

    if (!Result.has_value() || Result.value() == 0)
    {
        // No message received
        return std::nullopt;
    }

    // Drain queue until newest message is fetched
    while(true)
    {
        zmq::message_t NextMessage;

        auto NextResult = this->Socket.recv(NextMessage, zmq::recv_flags::dontwait);

        if (!NextResult.has_value() || NextResult.value() == 0)
        {
            // No more messages
            break;
        }

        Message = std::move(NextMessage);
    }

    return std::string(static_cast<char*>(Message.data()), Message.size());
}

NetworkClient::~NetworkClient()
{
    this->Socket.close();
    this->Context.close();
}
