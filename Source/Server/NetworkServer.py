import zmq
import ControlMessage_pb2

class NetworkServer:
    def __init__(self, url="tcp://*:5556"):
        self.Context = zmq.Context()
        self.Socket = self.Context.socket(zmq.DEALER)
        self.Socket.bind(url)

        print(f"Network server started on {url}")

    def Receive(self):
        try:
            Message = self.Socket.recv(flags=zmq.NOBLOCK)
        except zmq.ZMQError:
            return None
        
        # Loop until message is latest in the queue
        while True:
            try:
                Message = self.Socket.recv(flags=zmq.NOBLOCK)
            except zmq.ZMQError:
                break
        
        return Message

    def Send(self, Message):
        try:
            self.Socket.send_string(Message)
        except zmq.ZMQError as e:
            print(f"Error while sending message: {e}")

if __name__ == "__main__":
    """
    Example usage of NetworkServer. This will print values from received control messages.
    """
    Server = NetworkServer()

    ControlMessage = ControlMessage_pb2.ControlMessage()

    while True:
        Message = Server.Receive()

        if Message is not None:
            ControlMessage.ParseFromString(Message)
            print(f"Received control message: Forward={ControlMessage.Forward}, Pitch={ControlMessage.Pitch}, Roll={ControlMessage.Roll}, Thrust={ControlMessage.Thrust}")
