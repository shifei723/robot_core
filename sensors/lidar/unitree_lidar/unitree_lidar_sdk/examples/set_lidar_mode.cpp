/**
 * set_lidar_mode.cpp — Unitree L2 工作模式切换/探测工具（参数可配置版）
 *
 * 官方 example 把 IP、串口硬编码，且探测用死循环无超时，不便脚本调用。
 * 本工具将参数全部开放，并加入超时与语义化退出码。
 *
 * 关于模式切换的正确认知（对照 example_lidar_serial.cpp 得出）：
 *   串口链路本身就能下发工作模式指令，不必先走 UDP。
 *   完整序列为: startLidarRotation -> setLidarWorkMode(8) -> resetLidar
 *   缺少 setLidarWorkMode/resetLidar 时，雷达不会通过串口吐数据，
 *   表现为一直 "Serial port timeout"。
 *   UDP 途径(--via udp)仅用于只接了网线、没接 USB 的场景。
 *
 * 用法:
 *   # 确保处于串口模式并验证（幂等，自动化首选）
 *   ./set_lidar_mode --to-serial --port /dev/ttyACM0
 *
 *   # 停止/启动转子（平时停转可消除旋转噪声，需要点云时再启动）
 *   ./set_lidar_mode --stop-rotation  --port /dev/unilidar
 *   ./set_lidar_mode --start-rotation --port /dev/unilidar
 *
 *   # 只探测当前是否已在串口模式吐数据（不改变雷达状态）
 *   ./set_lidar_mode --detect-serial --port /dev/ttyACM0
 *
 *   # 只接网线时，通过 UDP 切串口模式
 *   ./set_lidar_mode --to-serial --via udp --lidar-ip 192.168.1.62 --local-ip 192.168.1.50
 *
 *   # 切回 UDP 模式
 *   ./set_lidar_mode --to-udp --port /dev/ttyACM0
 *
 * 退出码: 0=成功, 1=参数错误, 2=连接失败, 3=雷达无响应
 */

#include "unitree_lidar_sdk.h"

#include <chrono>
#include <iostream>
#include <string>
#include <unistd.h>

using namespace unilidar_sdk2;

namespace {

constexpr uint32_t kWorkModeUdp = 0;
constexpr uint32_t kWorkModeSerial = 8;

struct Options {
    std::string action;                    // detect-serial/detect-udp/to-serial/to-udp
    std::string via = "serial";            // 下发指令走哪条链路: serial / udp
    std::string port = "/dev/ttyACM0";
    uint32_t baudrate = 4000000;
    std::string lidar_ip = "192.168.1.62";
    std::string local_ip = "192.168.1.50";
    unsigned short lidar_port = 6101;
    unsigned short local_port = 6201;
    int timeout_s = 10;                    // 官方 example 常需数秒才吐首包，给足余量
};

void usage(const char* prog) {
    std::cout
        << "Unitree L2 工作模式切换/探测工具\n\n"
        << "用法: " << prog << " <动作> [选项]\n\n"
        << "动作:\n"
        << "  --to-serial       确保雷达处于串口模式(幂等), 默认经串口链路下发\n"
        << "  --to-udp          切换到 UDP 模式 (需经串口链路)\n"
        << "  --stop-rotation   停止转子 (消除旋转噪声, 不再输出点云)\n"
        << "  --start-rotation  启动转子 (恢复点云输出)\n"
        << "  --detect-serial   仅探测串口上是否有数据, 不改变雷达状态\n"
        << "  --detect-udp      仅探测 UDP 上是否有数据\n\n"
        << "选项:\n"
        << "  --via serial|udp  下发指令的链路 (默认 serial)\n"
        << "  --port PATH       串口设备 (默认 /dev/ttyACM0)\n"
        << "  --baudrate N      波特率 (默认 4000000)\n"
        << "  --lidar-ip IP     雷达 IP (默认 192.168.1.62)\n"
        << "  --local-ip IP     本机 IP (默认 192.168.1.50)\n"
        << "  --lidar-port N    雷达端口 (默认 6101)\n"
        << "  --local-port N    本机端口 (默认 6201)\n"
        << "  --timeout N       等待雷达响应的秒数 (默认 10)\n";
}

bool parseArgs(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::cerr << "缺少 " << a << " 的取值\n";
                exit(1);
            }
            return argv[++i];
        };
        if (a == "--detect-serial" || a == "--detect-udp" ||
            a == "--to-serial" || a == "--to-udp" ||
            a == "--stop-rotation" || a == "--start-rotation") {
            opt.action = a.substr(2);
        } else if (a == "--via")       opt.via = next();
        else if (a == "--port")        opt.port = next();
        else if (a == "--baudrate")    opt.baudrate = std::stoul(next());
        else if (a == "--lidar-ip")    opt.lidar_ip = next();
        else if (a == "--local-ip")    opt.local_ip = next();
        else if (a == "--lidar-port")  opt.lidar_port = std::stoi(next());
        else if (a == "--local-port")  opt.local_port = std::stoi(next());
        else if (a == "--timeout")     opt.timeout_s = std::stoi(next());
        else if (a == "-h" || a == "--help") { usage(argv[0]); exit(0); }
        else {
            std::cerr << "未知参数: " << a << "\n";
            return false;
        }
    }
    if (opt.via != "serial" && opt.via != "udp") {
        std::cerr << "--via 只能是 serial 或 udp\n";
        return false;
    }
    // 转子控制与 to-udp 均必须经串口下发
    if (opt.action == "to-udp" || opt.action == "stop-rotation" ||
        opt.action == "start-rotation") {
        opt.via = "serial";
    }
    return !opt.action.empty();
}

// 在超时内持续 runParse，直到读到固件版本（证明雷达确实在该链路上吐数据）
// 注意: 必须是紧循环，不能在两次 runParse 之间 sleep。
// 4Mbaud 下数据率约 500KB/s，插入毫秒级睡眠会让解析跟不上串口缓冲，
// 表现为持续 "Serial port timeout" 而永远读不到版本包。
bool waitForLidar(UnitreeLidarReader* reader, int timeout_s, std::string& firmware) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);
    int check_counter = 0;
    while (true) {
        reader->runParse();
        if (reader->getVersionOfLidarFirmware(firmware) && !firmware.empty()) {
            return true;
        }
        // 每若干次迭代才查一次时钟，避免频繁取时间拖慢解析
        if (++check_counter >= 200) {
            check_counter = 0;
            if (std::chrono::steady_clock::now() >= deadline) return false;
        }
    }
}

void printVersions(UnitreeLidarReader* reader, const std::string& firmware) {
    std::string hardware, sdk;
    reader->getVersionOfLidarHardware(hardware);
    reader->getVersionOfSDK(sdk);
    std::cout << "[在线] 硬件=" << hardware << " 固件=" << firmware
              << " SDK=" << sdk << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) {
        usage(argv[0]);
        return 1;
    }

    UnitreeLidarReader* reader = createUnitreeLidarReader();

    // 探测动作固定使用其名字对应的链路；切换动作由 --via 决定
    const bool use_serial =
        (opt.action == "detect-serial") ? true :
        (opt.action == "detect-udp")    ? false : (opt.via == "serial");
    if (use_serial) {
        std::cout << "[连接] 串口 " << opt.port << " @ " << opt.baudrate << std::endl;
        if (reader->initializeSerial(opt.port, opt.baudrate)) {
            std::cerr << "[失败] 无法打开串口 " << opt.port << std::endl;
            return 2;
        }
    } else {
        std::cout << "[连接] UDP 雷达 " << opt.lidar_ip << ":" << opt.lidar_port
                  << " <- 本机 " << opt.local_ip << ":" << opt.local_port << std::endl;
        if (reader->initializeUDP(opt.lidar_port, opt.lidar_ip,
                                  opt.local_port, opt.local_ip)) {
            std::cerr << "[失败] UDP 初始化失败（检查本机是否已配置 "
                      << opt.local_ip << "）" << std::endl;
            return 2;
        }
    }

    // ── 纯探测: 不改变雷达状态 ──
    if (opt.action == "detect-serial" || opt.action == "detect-udp") {
        std::string firmware;
        if (waitForLidar(reader, opt.timeout_s, firmware)) {
            printVersions(reader, firmware);
            return 0;
        }
        std::cerr << "[无响应] " << opt.timeout_s
                  << " 秒内未收到数据（若雷达已接好，说明它当前不在该模式）" << std::endl;
        return 3;
    }

    // ── 转子控制: 平时停转降噪，需要点云时再启动 ──
    if (opt.action == "stop-rotation") {
        std::cout << "[停转] stopLidarRotation" << std::endl;
        reader->stopLidarRotation();
        sleep(2);   // 给转子减速停下的时间
        std::cout << "[完成] 雷达已停转，不再输出点云" << std::endl;
        return 0;
    }

    if (opt.action == "start-rotation") {
        std::cout << "[启转] startLidarRotation" << std::endl;
        reader->startLidarRotation();
        sleep(2);   // 等转速稳定
        std::cout << "[验证] 等待点云数据 (最多 " << opt.timeout_s << " 秒)..."
                  << std::endl;
        std::string fw;
        if (waitForLidar(reader, opt.timeout_s, fw)) {
            printVersions(reader, fw);
            std::cout << "[成功] 雷达已旋转并输出数据" << std::endl;
            return 0;
        }
        std::cerr << "[失败] 启转后未收到数据，雷达可能不在串口模式" << std::endl;
        return 3;
    }

    // ── 模式切换 ──
    const uint32_t mode = (opt.action == "to-serial") ? kWorkModeSerial : kWorkModeUdp;

    // 幂等保证: 已在目标模式正常吐数据时直接返回。
    // resetLidar 会重启雷达中断数据流，对本已就绪的雷达是破坏性操作。
    if (mode == kWorkModeSerial && use_serial) {
        std::cout << "[检查] 先看是否已在串口模式..." << std::endl;
        std::string existing;
        if (waitForLidar(reader, 5, existing)) {
            printVersions(reader, existing);
            std::cout << "[就绪] 雷达已处于串口模式，无需切换" << std::endl;
            return 0;
        }
        std::cout << "      无数据，执行模式切换" << std::endl;
    }

    if (use_serial) {
        // 先让雷达转起来，否则不会响应后续指令
        reader->startLidarRotation();
        sleep(1);
    }

    std::cout << "[切换] work_mode = " << mode
              << (mode == kWorkModeSerial ? " (串口)" : " (UDP)") << std::endl;
    reader->setLidarWorkMode(mode);
    sleep(1);

    if (use_serial) {
        // resetLidar 让新模式生效，缺此步会持续 Serial port timeout
        std::cout << "[重置] resetLidar 使模式生效" << std::endl;
        reader->resetLidar();
        sleep(3);   // 给雷达重启留足时间
    }

    // 切到 UDP 后串口自然不再吐数据，无法也无需在此链路验证
    if (mode == kWorkModeUdp) {
        std::cout << "[完成] 已切到 UDP 模式，请将 ROS2 launch 的 "
                     "initialize_type 改为 2" << std::endl;
        return 0;
    }

    // reset 后原连接已失效，重开串口再验证
    if (use_serial) {
        reader->closeSerial();
        sleep(1);
        if (reader->initializeSerial(opt.port, opt.baudrate)) {
            std::cerr << "[失败] reset 后无法重新打开 " << opt.port
                      << "（设备可能正在重新枚举，稍后用 --detect-serial 验证）"
                      << std::endl;
            return 3;
        }
        reader->startLidarRotation();
        sleep(1);
    }

    std::cout << "[验证] 等待串口数据 (最多 " << opt.timeout_s << " 秒)..." << std::endl;
    std::string firmware;
    if (waitForLidar(reader, opt.timeout_s, firmware)) {
        printVersions(reader, firmware);
        std::cout << "[成功] 雷达已在串口模式正常工作" << std::endl;
        return 0;
    }
    std::cerr << "[失败] 指令已下发但未收到数据，可稍等后重试 --detect-serial" << std::endl;
    return 3;
}
