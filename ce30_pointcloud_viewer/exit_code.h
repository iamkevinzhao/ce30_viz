#ifndef EXIT_CODE_H
#define EXIT_CODE_H

enum class ExitCode {
  no_exit = -1,
  normal_exit = 0,
  device_connection_failure = 1,
  start_ce30_failure = 2,
  retrieve_ce30_version_failure = 3,
};

#endif // EXIT_CODE_H
