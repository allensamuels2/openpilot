// got this from https://github.com/computersarecool/cpp_sockets/tree/master/src


#include <string>
#include <iostream>
#include <exception>
#include <thread>
#include <string.h>

// Windows
#if defined(_WIN32)
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <winsock2.h>
#include <Ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
typedef SSIZE_T ssize_t;

// Linux
#else
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/tcp.h>
#include <arpa/inet.h> // This contains inet_addr
#include <unistd.h> // This contains close
#define INVALID_SOCKET (SOCKET)(~0)
#define SOCKET_ERROR (-1)
typedef int SOCKET;
#endif

// These could also be enums
struct bind_err : public std::exception { };
struct socket_err : public std::exception {};
struct accept_err : public std::exception {};
struct connection_err : public std::exception{};
struct send_err : public std::exception{};
struct recv_err : public std::exception{};

class Socket {
public:
    std::string recv();
    void send(const std::string&);
    ~Socket() {
#ifdef WIN32
        closesocket(m_socket);
#else        
        close(m_socket);
#endif
    }
    Socket& operator=(const Socket& s) {
        m_socket = s.m_socket;
        m_addr   = s.m_addr;
        const_cast<Socket&>(s).m_socket = -1;
        return *this;
    }
    Socket(const Socket& s) {
        m_socket = s.m_socket;
        m_addr   = s.m_addr;
        const_cast<Socket&>(s).m_socket = -1;
    }
    void bind();
    enum class SocketType {
        TYPE_STREAM = SOCK_STREAM,
        TYPE_DGRAM = SOCK_DGRAM
    };
    void set_port(u_short port) { m_addr.sin_port = htons(port); }
    int set_address(const std::string& ip_address) { return inet_pton(AF_INET, ip_address.c_str(), &m_addr.sin_addr); }
    explicit Socket(SocketType socket_type = SocketType::TYPE_DGRAM);
    Socket accept();
    std::string format() const {
        std::ostringstream os;
        os << ipname(m_addr) << ":" << ntohs(m_addr.sin_port) << "(" << m_socket << ")";
        return os.str();
    }
    std::string getpeername() const {
        sockaddr_in addr;
        socklen_t len = sizeof(addr);
        ::getpeername(m_socket, reinterpret_cast<struct sockaddr *>(&addr), &len);
        return ipname(addr);
    }
    u_short getpeerport() const {
        sockaddr_in addr;
        socklen_t len = sizeof(addr);
        ::getpeername(m_socket, reinterpret_cast<struct sockaddr *>(&addr), &len);
        return ntohs(addr.sin_port);
    }
    static void initSockets() {
#ifdef WIN32
    // Initialize the WSDATA if no socket instance exists
    if (!s_count) {
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            throw std::runtime_error("Error initializing Winsock " + WSAGetLastError());
        }
    }
    #endif        
}
protected:
    std::string ipname(const sockaddr_in &addr) const {
        char tmp[100];
        return std::string(inet_ntop(AF_INET, &addr.sin_addr, tmp, sizeof(tmp)));
    }
    explicit Socket(SOCKET socket, sockaddr_in addr) : m_socket(socket), m_addr(addr) { }
    SOCKET m_socket;
    sockaddr_in m_addr;
#ifdef WIN32
    // Number of sockets is tracked to call WSACleanup on Windows
    static int s_count;
#endif
};

#ifdef WIN32
// Initialize s_count on windows
int Socket::s_count{ 0 };
#endif

class TCPClient : public Socket
{
public:
    TCPClient(u_short port, const std::string& ip_address = "127.0.0.1");
    void make_connection();
};

class TCPServer
{
public:
    TCPServer(u_short port, const std::string& ip_address = "0.0.0.0");
    void bind() { m_listen.bind(); }
    Socket accept() { return m_listen.accept(); }
private:
    Socket m_listen;
    TCPServer(const TCPServer&) = delete;
};

Socket::Socket(const SocketType socket_type) : m_socket(), m_addr() { 
    // Create the socket handle
    m_socket = socket(AF_INET, static_cast<int>(socket_type), 0);
    if (m_socket == INVALID_SOCKET) {
        throw socket_err();
    }
    m_addr.sin_family = AF_INET;
}

TCPClient::TCPClient(u_short port, const std::string& ip_address) : Socket(SocketType::TYPE_STREAM) {
    set_address(ip_address);
    set_port(port);
}

void TCPClient::make_connection() {
    if (connect(m_socket, reinterpret_cast<sockaddr*>(&m_addr), sizeof(m_addr)) < 0) {
        throw connection_err();
    }
}

void Socket::send(const std::string& message) {
    if (::send(m_socket, message.c_str(), message.length(), 0) < 0) {
        throw send_err();
    }
}

std::string Socket::recv() {
    char server_reply[20000];
    int result = ::recv(m_socket, server_reply, sizeof(server_reply), 0);
    if (result == SOCKET_ERROR || result == 0) {
        std::cerr << "RECV ERROR result " << result << " or " << errno << " or " << strerror(errno) << "\n";
        throw recv_err();
    }
    return std::string(server_reply, result);
}

TCPServer::TCPServer(u_short port, const std::string& ip_address) : m_listen(Socket::SocketType::TYPE_STREAM) {
    m_listen.set_port(port);
    m_listen.set_address(ip_address);
};

void Socket::bind() {
    std::cerr << "Binding to " << format() << "\n";
    int flag = 1;
    if (::setsockopt(m_socket, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag))) {
        std::cerr << "setSockOpt failure, err=" << errno << "\n";
        throw bind_err();
    }
    if (::bind(m_socket, reinterpret_cast<sockaddr*>(&m_addr), sizeof(m_addr)) == SOCKET_ERROR) {
        throw bind_err();
    }
    if (::listen(m_socket, 1)) {
        std::cerr << "listen failure, err=" << errno << "\n";
    }
}

Socket Socket::accept() {
    socklen_t client_size = sizeof(sockaddr_in);
    sockaddr_in client;
    SOCKET new_socket;
    new_socket = ::accept(m_socket, reinterpret_cast<sockaddr*>(&client), &client_size);
    if (new_socket == INVALID_SOCKET) {
        //std::cout << "Accept error " << errno << ":" << strerror(errno) << " Socket:" << m_socket << "\n";
        throw accept_err();
    }
    int i = 1;
    if (::setsockopt( new_socket, IPPROTO_TCP, TCP_NODELAY, (void *)&i, sizeof(i))) {
        throw std::runtime_error("Can't set no-delay");
    }
    Socket s(new_socket, client);
    std::cout << "Accepted connection on " << new_socket << " " << s.format() << "\n";
    return s;
}
