#pragma once
// Minimal TraCI (SUMO's remote-control protocol) client over a raw TCP
// socket. No libsumo, no Python — just the documented wire protocol,
// implemented against what SUMO 1.27.1 actually sends on the wire
// (verified byte-for-byte against the official Python client; see
// docs/TRACI_PROTOCOL_NOTES.md for the worked-out decode).
//
// This only talks to an *already-running* `sumo` process that was
// started with `--remote-port <port>`. Launching that process is
// deliberately left to the experiment runner script (run_experiment.py)
// rather than done here, so this file stays free of platform-specific
// process-spawning code — see simulation/sumo_integration/README.md.
//
// Supported subset (everything SumoBridge needs):
//   - CMD_SIMSTEP           (0x02) advance the simulation by one step
//   - CMD_GET_VEHICLE_VARIABLE (0xa4):
//       ID_LIST       (0x00) -> list of vehicle ids currently in the sim
//       VAR_POSITION  (0x42) -> (x, y) in network coordinates
//       VAR_SPEED     (0x40) -> speed in m/s
//   - CMD_CLOSE             (0x7f) clean shutdown
//
// Message framing:
//   [4-byte big-endian length, INCLUDING these 4 bytes][body]
// Command framing (inside a message body), one or more back-to-back:
//   [1-byte length, INCLUDING this byte][cmdId][payload]
//   -- if the sub-command needs to be >= 255 bytes, the length byte is
//      0x00 and is followed by a 4-byte big-endian extended length
//      (this matters mainly on the *receive* side, e.g. a vehicle id
//      list response once there are enough vehicles).
// Strings: [4-byte big-endian length][UTF-8 bytes], no terminator.
// Every command SUMO responds to gets a "status" entry back:
//   [len][cmdId][resultCode][errorString]
// GET_* commands additionally get an "answer" entry right after:
//   [len][cmdId+0x10][variableId][objectIdString][valueType][value...]

#include <cstdint>
#include <cstring>
#include <string>
#include <stdexcept>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>
#include <time.h>
#endif

namespace iov::simulation {

// --- TraCI wire constants (from SUMO's traci/constants.py) ---
namespace traci_const {
constexpr uint8_t CMD_GETVERSION = 0x00;
constexpr uint8_t CMD_SIMSTEP = 0x02;
constexpr uint8_t CMD_CLOSE = 0x7f;
constexpr uint8_t CMD_GET_VEHICLE_VARIABLE = 0xa4;

constexpr uint8_t TRACI_ID_LIST = 0x00;
constexpr uint8_t VAR_SPEED = 0x40;
constexpr uint8_t VAR_POSITION = 0x42;

constexpr uint8_t POSITION_2D = 0x01;
constexpr uint8_t TYPE_DOUBLE = 0x0B;
constexpr uint8_t TYPE_STRING = 0x0C;
constexpr uint8_t TYPE_STRINGLIST = 0x0E;
}  // namespace traci_const

class TraciClientError : public std::runtime_error {
public:
    explicit TraciClientError(const std::string& msg) : std::runtime_error(msg) {}
};

// Raw byte-buffer writer/reader matching TraCI's big-endian, length-
// prefixed encoding. Kept separate from socket I/O so the framing logic
// is unit-testable without a live SUMO process.
class TraciBuffer {
public:
    // --- writing ---
    void putU8(uint8_t v) { bytes_.push_back(static_cast<char>(v)); }

    void putU32(uint32_t v) {
        for (int i = 3; i >= 0; --i) bytes_.push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
    }

    void putF64(double v) {
        uint64_t bits;
        static_assert(sizeof(bits) == sizeof(v), "double must be 8 bytes");
        std::memcpy(&bits, &v, sizeof(bits));
        for (int i = 7; i >= 0; --i) bytes_.push_back(static_cast<char>((bits >> (8 * i)) & 0xFF));
    }

    void putStr(const std::string& s) {
        putU32(static_cast<uint32_t>(s.size()));
        bytes_.append(s);
    }

    // Appends one length-prefixed sub-command: [len][cmdId][payload].
    // Uses the extended (0x00 + 4-byte length) form automatically if the
    // sub-command would be >= 255 bytes.
    void putCommand(uint8_t cmd_id, const std::string& payload) {
        size_t total = 1 /*len byte*/ + 1 /*cmd*/ + payload.size();
        if (total < 255) {
            putU8(static_cast<uint8_t>(total));
            putU8(cmd_id);
            bytes_.append(payload);
        } else {
            putU8(0);
            putU32(static_cast<uint32_t>(5 /*0x00 + 4-byte len + cmd*/ + payload.size()));
            putU8(cmd_id);
            bytes_.append(payload);
        }
    }

    const std::string& data() const { return bytes_; }
    void clear() { bytes_.clear(); }

private:
    std::string bytes_;
};

class TraciReader {
public:
    explicit TraciReader(const std::string& body) : buf_(body), pos_(0) {}

    uint8_t readU8() {
        require(1);
        return static_cast<uint8_t>(buf_[pos_++]);
    }

    uint32_t readU32() {
        require(4);
        uint32_t v = 0;
        for (int i = 0; i < 4; ++i)
            v = (v << 8) | static_cast<uint8_t>(buf_[pos_++]);
        return v;
    }

    double readF64() {
        require(8);
        uint64_t bits = 0;
        for (int i = 0; i < 8; ++i)
            bits = (bits << 8) | static_cast<uint8_t>(buf_[pos_++]);
        double v;
        std::memcpy(&v, &bits, sizeof(v));
        return v;
    }

    std::string readStr() {
        uint32_t len = readU32();
        require(len);
        std::string s = buf_.substr(pos_, len);
        pos_ += len;
        return s;
    }

    // Reads a sub-command/answer length, transparently handling the
    // extended (0x00 + 4-byte) form.
    uint32_t readLength() {
        uint8_t b = readU8();
        if (b == 0) return readU32();
        return b;
    }

    bool atEnd() const { return pos_ >= buf_.size(); }
    size_t remaining() const { return buf_.size() - pos_; }

private:
    void require(size_t n) const {
        if (pos_ + n > buf_.size())
            throw TraciClientError("TraCI response truncated while parsing");
    }

    std::string buf_;
    size_t pos_;
};

// Parsed result of one GET_* command's answer.
struct TraciAnswer {
    uint8_t response_cmd;
    uint8_t variable_id;
    std::string object_id;
    uint8_t value_type;
    std::vector<std::string> string_list;  // valid if value_type == TYPE_STRINGLIST
    double d1 = 0.0, d2 = 0.0;             // valid if POSITION_2D (d1=x,d2=y) or TYPE_DOUBLE (d1)
};

class TraciClient {
public:
    TraciClient() = default;
    ~TraciClient() { close(); }

    TraciClient(const TraciClient&) = delete;
    TraciClient& operator=(const TraciClient&) = delete;

    // Connects to an already-running `sumo --remote-port <port>` process.
    // SUMO's TraCI server may not be listening yet the instant the
    // process starts, so this retries with a short backoff.
    void connect(const std::string& host, int port, int max_retries = 50,
                 int retry_delay_ms = 200) {
#ifdef _WIN32
        WSADATA wsa;
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
            throw TraciClientError("WSAStartup failed");
        wsa_started_ = true;
#endif
        struct addrinfo hints;
        std::memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;

        struct addrinfo* res = nullptr;
        std::string port_str = std::to_string(port);
        int rc = getaddrinfo(host.c_str(), port_str.c_str(), &hints, &res);
        if (rc != 0) throw TraciClientError("getaddrinfo failed for " + host);

        int last_errno = 0;
        for (int attempt = 0; attempt < max_retries; ++attempt) {
            sock_ = static_cast<int>(socket(res->ai_family, res->ai_socktype, res->ai_protocol));
            if (sock_ < 0) { last_errno = 1; sleepMs(retry_delay_ms); continue; }

            if (::connect(sock_, res->ai_addr, static_cast<int>(res->ai_addrlen)) == 0) {
                freeaddrinfo(res);
                connected_ = true;
                return;
            }
            closeSocket(sock_);
            sock_ = -1;
            sleepMs(retry_delay_ms);
        }
        freeaddrinfo(res);
        throw TraciClientError("Could not connect to SUMO TraCI server at " + host + ":" +
                                port_str + " after retrying (last_errno=" +
                                std::to_string(last_errno) + ")");
    }

    void close() {
        if (connected_) {
            try {
                TraciBuffer msg;
                msg.putCommand(traci_const::CMD_CLOSE, "");
                sendMessage(msg.data());
                recvMessage();  // drain the status reply, ignore errors
            } catch (...) {
                // best-effort close
            }
        }
        if (sock_ >= 0) {
            closeSocket(sock_);
            sock_ = -1;
        }
        connected_ = false;
#ifdef _WIN32
        if (wsa_started_) { WSACleanup(); wsa_started_ = false; }
#endif
    }

    // Advances the simulation by exactly one time step (SUMO's
    // --step-length, e.g. 0.1s). Ignores subscription results (we don't
    // use subscriptions — every GET is a fresh polled command).
    void simulationStep() {
        TraciBuffer msg;
        std::string payload;
        TraciBuffer time_payload;
        time_payload.putF64(0.0);  // 0.0 == "advance exactly one step"
        msg.putCommand(traci_const::CMD_SIMSTEP, time_payload.data());
        std::string resp = sendAndRecv(msg.data());
        TraciReader r(resp);
        readStatus(r, traci_const::CMD_SIMSTEP);
        // Remaining bytes (if any) are subscription results we don't use.
    }

    std::vector<std::string> getVehicleIDList() {
        TraciAnswer ans = getVehicleVariable("", traci_const::TRACI_ID_LIST);
        return ans.string_list;
    }

    bool getVehiclePosition(const std::string& veh_id, double& x, double& y) {
        try {
            TraciAnswer ans = getVehicleVariable(veh_id, traci_const::VAR_POSITION);
            x = ans.d1;
            y = ans.d2;
            return true;
        } catch (const TraciClientError&) {
            return false;  // vehicle likely left the simulation this tick
        }
    }

    bool getVehicleSpeed(const std::string& veh_id, double& speed) {
        try {
            TraciAnswer ans = getVehicleVariable(veh_id, traci_const::VAR_SPEED);
            speed = ans.d1;
            return true;
        } catch (const TraciClientError&) {
            return false;
        }
    }

    // Batched fetch: one round trip fetches position+speed for every
    // vehicle currently in the simulation, instead of 2*N round trips.
    // TraCI allows multiple commands per message; responses come back
    // as (status, answer) pairs in the same order they were sent.
    struct VehicleSample {
        std::string id;
        double x = 0.0, y = 0.0, speed = 0.0;
    };

    std::vector<VehicleSample> getAllVehicleData() {
        std::vector<std::string> ids = getVehicleIDList();
        if (ids.empty()) return {};

        TraciBuffer msg;
        for (const auto& id : ids) {
            TraciBuffer p1;
            p1.putU8(traci_const::VAR_POSITION);
            p1.putStr(id);
            msg.putCommand(traci_const::CMD_GET_VEHICLE_VARIABLE, p1.data());

            TraciBuffer p2;
            p2.putU8(traci_const::VAR_SPEED);
            p2.putStr(id);
            msg.putCommand(traci_const::CMD_GET_VEHICLE_VARIABLE, p2.data());
        }

        std::string resp = sendAndRecv(msg.data());
        TraciReader r(resp);

        std::vector<VehicleSample> out;
        out.reserve(ids.size());
        for (size_t i = 0; i < ids.size(); ++i) {
            readStatus(r, traci_const::CMD_GET_VEHICLE_VARIABLE);
            TraciAnswer pos_ans = readAnswer(r);
            readStatus(r, traci_const::CMD_GET_VEHICLE_VARIABLE);
            TraciAnswer spd_ans = readAnswer(r);

            VehicleSample vs;
            vs.id = ids[i];
            vs.x = pos_ans.d1;
            vs.y = pos_ans.d2;
            vs.speed = spd_ans.d1;
            out.push_back(vs);
        }
        return out;
    }

private:
    TraciAnswer getVehicleVariable(const std::string& object_id, uint8_t variable) {
        TraciBuffer payload;
        payload.putU8(variable);
        payload.putStr(object_id);

        TraciBuffer msg;
        msg.putCommand(traci_const::CMD_GET_VEHICLE_VARIABLE, payload.data());

        std::string resp = sendAndRecv(msg.data());
        TraciReader r(resp);
        readStatus(r, traci_const::CMD_GET_VEHICLE_VARIABLE);
        return readAnswer(r);
    }

    // Reads one [len][cmdId][resultCode][errString] status entry and
    // throws if SUMO reported an error.
    void readStatus(TraciReader& r, uint8_t expected_cmd) {
        r.readLength();
        uint8_t cmd = r.readU8();
        uint8_t result = r.readU8();
        std::string err = r.readStr();
        if (result != 0 || cmd != expected_cmd) {
            throw TraciClientError("TraCI error (cmd=" + std::to_string(cmd) +
                                    ", result=" + std::to_string(result) + "): " + err);
        }
    }

    // Reads one [len][respCmd][varId][objId][valueType][value...] answer.
    TraciAnswer readAnswer(TraciReader& r) {
        TraciAnswer ans;
        r.readLength();
        ans.response_cmd = r.readU8();
        ans.variable_id = r.readU8();
        ans.object_id = r.readStr();
        ans.value_type = r.readU8();

        switch (ans.value_type) {
            case traci_const::TYPE_STRINGLIST: {
                uint32_t n = r.readU32();
                ans.string_list.reserve(n);
                for (uint32_t i = 0; i < n; ++i) ans.string_list.push_back(r.readStr());
                break;
            }
            case traci_const::POSITION_2D:
                ans.d1 = r.readF64();
                ans.d2 = r.readF64();
                break;
            case traci_const::TYPE_DOUBLE:
                ans.d1 = r.readF64();
                break;
            case traci_const::TYPE_STRING:
                ans.object_id = r.readStr();  // rarely used here; kept for completeness
                break;
            default:
                throw TraciClientError("Unhandled TraCI value type: " +
                                        std::to_string(ans.value_type));
        }
        return ans;
    }

    std::string sendAndRecv(const std::string& body) {
        sendMessage(body);
        return recvMessage();
    }

    void sendMessage(const std::string& body) {
        if (sock_ < 0) throw TraciClientError("sendMessage on closed socket");
        TraciBuffer framed;
        framed.putU32(static_cast<uint32_t>(body.size() + 4));
        std::string full = framed.data() + body;
        sendAll(full);
    }

    std::string recvMessage() {
        std::string len_bytes = recvExact(4);
        uint32_t total_len = (static_cast<uint8_t>(len_bytes[0]) << 24) |
                              (static_cast<uint8_t>(len_bytes[1]) << 16) |
                              (static_cast<uint8_t>(len_bytes[2]) << 8) |
                              static_cast<uint8_t>(len_bytes[3]);
        if (total_len < 4) throw TraciClientError("Malformed TraCI message length");
        return recvExact(total_len - 4);
    }

    void sendAll(const std::string& data) {
        size_t sent = 0;
        while (sent < data.size()) {
#ifdef _WIN32
            int n = ::send(sock_, data.data() + sent, static_cast<int>(data.size() - sent), 0);
#else
            ssize_t n = ::send(sock_, data.data() + sent, data.size() - sent, 0);
#endif
            if (n <= 0) throw TraciClientError("Socket send failed / SUMO closed the connection");
            sent += static_cast<size_t>(n);
        }
    }

    std::string recvExact(size_t n) {
        std::string out;
        out.resize(n);
        size_t got = 0;
        while (got < n) {
#ifdef _WIN32
            int r = ::recv(sock_, &out[got], static_cast<int>(n - got), 0);
#else
            ssize_t r = ::recv(sock_, &out[got], n - got, 0);
#endif
            if (r <= 0) throw TraciClientError("Socket recv failed / SUMO closed the connection");
            got += static_cast<size_t>(r);
        }
        return out;
    }

    static void closeSocket(int s) {
#ifdef _WIN32
        closesocket(s);
#else
        ::close(s);
#endif
    }

    static void sleepMs(int ms) {
#ifdef _WIN32
        Sleep(ms);
#else
        struct timespec ts;
        ts.tv_sec = ms / 1000;
        ts.tv_nsec = (ms % 1000) * 1000000L;
        nanosleep(&ts, nullptr);
#endif
    }

    int sock_ = -1;
    bool connected_ = false;
#ifdef _WIN32
    bool wsa_started_ = false;
#endif
};

}  // namespace iov::simulation
