// cameratracking: starts the session and guarantees it ends

#include <windows.h>

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

#ifndef CAMERA_TRACKING_SOURCE_DIR
#define CAMERA_TRACKING_SOURCE_DIR "."
#endif

namespace {

struct Child {
    const char*         name;
    PROCESS_INFORMATION pi{};
};

HANDLE g_job = nullptr;

std::vector<Child>* g_children = nullptr;
volatile LONG       g_shuttingDown = 0;

HANDLE createKillOnCloseJob()
{
    HANDLE job = CreateJobObjectA(nullptr, nullptr);
    if (job == nullptr) return nullptr;

    JOBOBJECT_EXTENDED_LIMIT_INFORMATION limits{};
    limits.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
    if (!SetInformationJobObject(job, JobObjectExtendedLimitInformation, &limits, sizeof(limits)))
    {
        CloseHandle(job);
        return nullptr;
    }
    return job;
}

bool spawn(std::string cmdLine, const std::string& workDir, PROCESS_INFORMATION& pi)
{
    STARTUPINFOA si{};
    si.cb = sizeof(si);

    if (CreateProcessA(nullptr, cmdLine.data(), nullptr, nullptr, FALSE,
                       CREATE_NEW_PROCESS_GROUP | CREATE_SUSPENDED, nullptr, workDir.c_str(),
                       &si, &pi) == 0)
        return false;

    if (g_job != nullptr && !AssignProcessToJobObject(g_job, pi.hProcess))
    {
        std::fprintf(stderr, "[cameratracking] could not put pid %lu under the job object "
                              "(err %lu) -- refusing to start it, since it could outlive this "
                              "launcher and publish alongside the next one\n",
                     pi.dwProcessId, GetLastError());
        TerminateProcess(pi.hProcess, 1);
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);
        pi = PROCESS_INFORMATION{};
        return false;
    }

    ResumeThread(pi.hThread);
    return true;
}

void stopChild(const Child& c, DWORD graceMs = 3000)
{
    if (c.pi.hProcess == nullptr) return;

    GenerateConsoleCtrlEvent(CTRL_BREAK_EVENT, c.pi.dwProcessId);
    if (WaitForSingleObject(c.pi.hProcess, graceMs) != WAIT_OBJECT_0)
    {
        std::fprintf(stderr, "[cameratracking] %s didn't exit gracefully, force-terminating\n",
                     c.name);
        TerminateProcess(c.pi.hProcess, 1);
        WaitForSingleObject(c.pi.hProcess, 2000);
    }
    CloseHandle(c.pi.hProcess);
    CloseHandle(c.pi.hThread);
}

void stopAll()
{
    if (InterlockedExchange(&g_shuttingDown, 1) != 0) return;   // handler may re-enter
    if (g_children == nullptr) return;
    for (auto it = g_children->rbegin(); it != g_children->rend(); ++it) stopChild(*it);
}

BOOL WINAPI consoleHandler(DWORD signal)
{
    switch (signal)
    {
        case CTRL_C_EVENT:
        case CTRL_BREAK_EVENT:
        case CTRL_CLOSE_EVENT:
        case CTRL_LOGOFF_EVENT:
        case CTRL_SHUTDOWN_EVENT:
            std::printf("[cameratracking] shutdown requested -- stopping children\n");
            std::fflush(stdout);
            stopAll();
            return TRUE;
        default:
            return FALSE;
    }
}

}  // namespace

int main()
{
    char selfPathBuf[MAX_PATH];
    GetModuleFileNameA(nullptr, selfPathBuf, MAX_PATH);
    const fs::path exeDir    = fs::path(selfPathBuf).parent_path();
    const fs::path sourceDir = CAMERA_TRACKING_SOURCE_DIR;

    g_job = createKillOnCloseJob();
    if (g_job == nullptr)
        std::fprintf(stderr, "[cameratracking] WARNING: no job object (err %lu). Children will "
                              "still be stopped on a clean exit, but killing this launcher "
                              "outright would orphan them -- and an orphaned backend publishes "
                              "alongside the next run rather than failing visibly.\n",
                     GetLastError());

    std::vector<Child> children;   // stopped in reverse order on shutdown
    g_children = &children;
    SetConsoleCtrlHandler(consoleHandler, TRUE);

    PROCESS_INFORMATION backendPi{};
    if (!spawn((exeDir / "backend.exe").string(), sourceDir.string(), backendPi))
    {
        std::fprintf(stderr, "[cameratracking] failed to start backend.exe\n");
        return 1;
    }
    children.push_back({"backend", backendPi});
    std::printf("[cameratracking] backend.exe started (pid %lu)\n", backendPi.dwProcessId);

    PROCESS_INFORMATION loggerPi{};
    if (spawn((exeDir / "logger_node.exe").string(), sourceDir.string(), loggerPi))
    {
        children.push_back({"logger_node", loggerPi});
        std::printf("[cameratracking] logger_node.exe started (pid %lu)\n", loggerPi.dwProcessId);
    }
    else
    {
        std::fprintf(stderr, "[cameratracking] WARNING: failed to start logger_node.exe "
                              "(target not built?) -- continuing without CSV logging\n");
    }

    PROCESS_INFORMATION frontendPi{};
    if (!spawn((exeDir / "frontend.exe").string(), exeDir.string(), frontendPi))
    {
        std::fprintf(stderr, "[cameratracking] failed to start frontend.exe\n");
        stopAll();
        return 1;
    }
    children.push_back({"frontend", frontendPi});
    std::printf("[cameratracking] frontend.exe started (pid %lu) -- close its window, or Ctrl+C "
                "here, to end the session\n",
                frontendPi.dwProcessId);

    std::vector<HANDLE> handles;
    handles.reserve(children.size());
    for (const Child& c : children) handles.push_back(c.pi.hProcess);

    const DWORD waited = WaitForMultipleObjects(static_cast<DWORD>(handles.size()), handles.data(),
                                                FALSE /* any */, INFINITE);
    const std::size_t firstOut =
        (waited >= WAIT_OBJECT_0 && waited < WAIT_OBJECT_0 + handles.size())
            ? static_cast<std::size_t>(waited - WAIT_OBJECT_0)
            : 0;

    DWORD exitCode = 0;
    GetExitCodeProcess(children[firstOut].pi.hProcess, &exitCode);

    constexpr DWORD kRecalibrateExitCode = 2;
    const bool byFrontend = std::string(children[firstOut].name) == "frontend";

    if (byFrontend && exitCode == kRecalibrateExitCode)
        std::printf("[cameratracking] SESSION ABANDONED at the placement review gate -- "
                     "the operator rejected the latched placements. Recalibrate the robot "
                     "before rerunning; this session produced no validation data worth "
                     "keeping.\n");
    else if (byFrontend)
        std::printf("[cameratracking] frontend exited (code %lu), shutting down the rest\n",
                    exitCode);
    else
        std::fprintf(stderr, "[cameratracking] %s exited FIRST (code %lu) -- ending the session. "
                              "It was not supposed to stop before the frontend, so treat the last "
                              "data as suspect rather than as a completed run.\n",
                     children[firstOut].name, exitCode);

    stopAll();
    return static_cast<int>(exitCode);
}
