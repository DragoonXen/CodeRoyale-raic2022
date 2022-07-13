#include "DebugInterface.hpp"
#include "MyStrategy.hpp"
#include "TcpStream.hpp"
#include "codegame/ServerMessage.hpp"
#include "codegame/ClientMessage.hpp"
#include "utils/TimeMeasure.hpp"
#include <memory>
#include <string>

class Runner {
public:
    Runner(const std::string& host, int port, const std::string& token): tcpStream(host, port)
    {
        tcpStream.write(token);
        tcpStream.write(int(1));
        tcpStream.write(int(0));
        tcpStream.write(int(1));
        tcpStream.flush();
    }
    void run()
    {
        DebugInterface debugInterface(&tcpStream);
        std::shared_ptr<MyStrategy> myStrategy = std::shared_ptr<MyStrategy>();
#ifdef TICK_DEBUG_ENABLED
        std::vector<codegame::ServerMessage::GetOrder> messages;
        std::vector<MyStrategy> memory;
        messages.reserve(1000);
        memory.reserve(1000);
        try {
#endif
        while (true) {
            auto message = codegame::ServerMessage::readFrom(tcpStream);
            if (auto updateConstantsMessage = std::dynamic_pointer_cast<codegame::ServerMessage::UpdateConstants>(message)) {
                myStrategy.reset(new MyStrategy(updateConstantsMessage->constants));
            } else if (auto getOrderMessage = std::dynamic_pointer_cast<codegame::ServerMessage::GetOrder>(message)) {
                codegame::ClientMessage::OrderMessage(myStrategy->getOrder(getOrderMessage->playerView, getOrderMessage->debugAvailable ? &debugInterface : nullptr)).writeTo(tcpStream);
                tcpStream.flush();
#ifdef TICK_DEBUG_ENABLED
                messages.push_back(*getOrderMessage);
                memory.emplace_back(*myStrategy);
#endif

            } else if (auto finishMessage = std::dynamic_pointer_cast<codegame::ServerMessage::Finish>(message)) {
                TimeMeasure::printTimings();
                myStrategy->finish();
                break;
            } else if (auto debugUpdateMessage = std::dynamic_pointer_cast<codegame::ServerMessage::DebugUpdate>(message)) {
                myStrategy->debugUpdate(debugInterface);
                codegame::ClientMessage::DebugUpdateDone().writeTo(tcpStream);
                tcpStream.flush();
            } else {
                TimeMeasure::printTimings();
                throw std::runtime_error("Unexpected server message");
            }
        }
#ifdef TICK_DEBUG_ENABLED
        } catch (...) {
        }
        int tickNo;
        while (true) {
            std::cout << "Waiting for tick no to debug" << std::endl;
            std::cin >> tickNo;
            if (tickNo == -1) {
                break;
            }
            if (tickNo < 0 || tickNo >= (int) messages.size()) {
                std::cout << "out of bound, need up to " << std::to_string(messages.size()) << " exclusive" << std::endl;
                continue;
            }
            auto message = messages[tickNo];
            auto tickStrategy = memory[tickNo];
            try {
                tickStrategy.getOrder(message.playerView, message.debugAvailable ? &debugInterface : nullptr);
            } catch (...) {
            }
        }
#endif
    }

private:
    TcpStream tcpStream;
};

int main(int argc, char* argv[])
{
    std::string host = argc < 2 ? "127.0.0.1" : argv[1];
    int port = argc < 3 ? 31001 : atoi(argv[2]);
    std::string token = argc < 4 ? "0000000000000000" : argv[3];
    Runner(host, port, token).run();
    return 0;
}