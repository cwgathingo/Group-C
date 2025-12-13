from maze_shared.maze_config import LOG_LEVEL, LogLevel


def log(level: LogLevel, message: str) -> None:
    if level.value < LOG_LEVEL.value:
        return
    print(f"[{level.name}] {message}")


def logDebug(message: str) -> None:
    log(LogLevel.DEBUG, message)


def logInfo(message: str) -> None:
    log(LogLevel.INFO, message)


def logWarn(message: str) -> None:
    log(LogLevel.WARN, message)


def logError(message: str) -> None:
    log(LogLevel.ERROR, message)
