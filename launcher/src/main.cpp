// =============================================================
// cameratracking  (launcher)
//
// Spawns backend.exe + frontend.exe + logger_node.py together, and
// tears all three down when frontend.exe exits (closing the window is
// the user-facing signal that the session is over).
//
// Each child gets its own console process group (CREATE_NEW_PROCESS_GROUP)
// so it can be signalled individually via CTRL_BREAK_EVENT without also
// hitting the launcher itself or the other children -- Windows only
// supports targeting one specific process group with CTRL_BREAK; CTRL_C
// can only be broadcast to every process sharing the console. backend.exe
// relies on eCAL's own console-ctrl handling to turn CTRL_BREAK into
// eCAL::Ok() == false; logger_node.py has an explicit SIGBREAK handler
// for the same reason (see logger_node.py). If a child doesn't exit
// within the grace period, it's force-terminated as a fallback.
// =============================================================

#include <windows.h>

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

#ifndef CAMERA_TRACKING_SOURCE_DIR
#define CAMERA_TRACKING_SOURCE_DIR "."
#endif

struct Child {
    const char* name;
    PROCESS_INFORMATION pi{};
};

static bool spawn(std::string cmdLine, const std::string& workDir, PROCESS_INFORMATION& pi)
{
    STARTUPINFOA si{};
    si.cb = sizeof(si);
    return CreateProcessA(nullptr, cmdLine.data(), nullptr, nullptr, FALSE,
                           CREATE_NEW_PROCESS_GROUP, nullptr, workDir.c_str(), &si, &pi) != 0;
}

// Ask nicely first (CTRL_BREAK, lets backend/logger flush + shut down
// cleanly), then force-kill if it doesn't exit within the grace period.
static void stopChild(const Child& c, DWORD graceMs = 3000)
{
    if (c.pi.hProcess == nullptr) return;
    GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT, c.pi.dwProcessId);
    if (WaitForSingleObject(c.pi.hProcess, graceMs) != WAIT_OBJECT_0)
    {
        std::fprintf(stderr, "[cameratracking] %s didn't exit gracefully, force-terminating\n", c.name);
        TerminateProcess(c.pi.hProcess, 1);
        WaitForSingleObject(c.pi.hProcess, 2000);
    }
    CloseHandle(c.pi.hProcess);
    CloseHandle(c.pi.hThread);
}

int main()
{
    char selfPathBuf[MAX_PATH];
    GetModuleFileNameA(nullptr, selfPathBuf, MAX_PATH);
    const fs::path exeDir    = fs::path(selfPathBuf).parent_path();
    const fs::path sourceDir = CAMERA_TRACKING_SOURCE_DIR;

    std::vector<Child> children;  // stopped in reverse order on shutdown

    PROCESS_INFORMATION backendPi{};
    if (!spawn((exeDir / "backend.exe").string(), sourceDir.string(), backendPi))
    {
        std::fprintf(stderr, "[cameratracking] failed to start backend.exe\n");
        return 1;
    }
    children.push_back({"backend", backendPi});
    std::printf("[cameratracking] backend.exe started (pid %lu)\n", backendPi.dwProcessId);

    PROCESS_INFORMATION loggerPi{};
    const std::string loggerScript = (sourceDir / "nodes" / "logger_node" / "logger_node.py").string();
    if (spawn("python \"" + loggerScript + "\"", sourceDir.string(), loggerPi))
    {
        children.push_back({"logger_node", loggerPi});
        std::printf("[cameratracking] logger_node.py started (pid %lu)\n", loggerPi.dwProcessId);
    }
    else
    {
        std::fprintf(stderr, "[cameratracking] WARNING: failed to start logger_node.py "
                              "(is 'python' on PATH?) -- continuing without CSV logging\n");
    }

    PROCESS_INFORMATION frontendPi{};
    if (!spawn((exeDir / "frontend.exe").string(), exeDir.string(), frontendPi))
    {
        std::fprintf(stderr, "[cameratracking] failed to start frontend.exe\n");
        for (auto it = children.rbegin(); it != children.rend(); ++it) stopChild(*it);
        return 1;
    }
    std::printf("[cameratracking] frontend.exe started (pid %lu) -- close its window to end the session\n",
                frontendPi.dwProcessId);

    // frontend is the user-facing process; its exit is the signal the
    // session is over.
    WaitForSingleObject(frontendPi.hProcess, INFINITE);
    CloseHandle(frontendPi.hProcess);
    CloseHandle(frontendPi.hThread);

    std::printf("[cameratracking] frontend exited, shutting down backend + logger\n");
    for (auto it = children.rbegin(); it != children.rend(); ++it) stopChild(*it);

    return 0;
}
