/*
 * pr2-rt-exec: run a program with the capabilities the PR2 realtime loop needs.
 *
 *   pr2-rt-exec /absolute/path/to/program [args...]
 *
 * Installed setuid root, mode 4750, group-restricted (see install_pr2_rt.sh). It grants exactly
 *   CAP_NET_RAW   raw EtherCAT socket
 *   CAP_IPC_LOCK  mlockall
 *   CAP_SYS_NICE  SCHED_FIFO
 * as *ambient* capabilities, drops root completely (real, effective and saved uid/gid become the
 * caller's) and execs the program. Unlike file capabilities (setcap), nothing is attached to the
 * program file, so rebuilding it changes nothing, and the program does not run in the dynamic
 * loader's secure mode, so LD_LIBRARY_PATH works.
 *
 * The capability bounding set is reduced to the same three capabilities first, so the program
 * and its children can never gain any other capability (e.g. through setuid binaries).
 *
 * glibc removes LD_LIBRARY_PATH from the environment of setuid programs. The companion script
 * pr2-rt stores it in PR2_RT_EXEC_LD_LIBRARY_PATH; it is restored here after root is dropped,
 * when it can only affect a process running as the caller.
 *
 * This replaces, for ROS 2, what ROS 1's pr2-grant did (which also granted CAP_NET_ADMIN and set
 * file capabilities on a copy of the program).
 */
#define _GNU_SOURCE
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/capability.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <unistd.h>

static const cap_value_t kCaps[] = {CAP_NET_RAW, CAP_IPC_LOCK, CAP_SYS_NICE};
#define NCAPS ((int)(sizeof(kCaps) / sizeof(kCaps[0])))
static const char kLdEnv[] = "PR2_RT_EXEC_LD_LIBRARY_PATH";

static int wanted(cap_value_t c)
{
  for (int i = 0; i < NCAPS; ++i)
    if (kCaps[i] == c) return 1;
  return 0;
}

static void die(const char *what)
{
  fprintf(stderr, "pr2-rt-exec: %s: %s\n", what, strerror(errno));
  exit(126);
}

int main(int argc, char **argv)
{
  if (argc < 2 || argv[1][0] != '/')
  {
    fprintf(stderr, "usage: pr2-rt-exec /absolute/path/to/program [args...]\n"
                    "       (or use pr2-rt, which resolves the path and keeps LD_LIBRARY_PATH)\n");
    return 2;
  }
  struct stat st;
  if (stat(argv[1], &st) != 0) die(argv[1]);
  if (!S_ISREG(st.st_mode))
  {
    fprintf(stderr, "pr2-rt-exec: %s is not a regular file\n", argv[1]);
    return 2;
  }
  if (geteuid() != 0)
  {
    fprintf(stderr, "pr2-rt-exec: not installed setuid root (run install_pr2_rt.sh)\n");
    return 126;
  }
  const uid_t uid = getuid();
  const gid_t gid = getgid();

  /* 1. Bounding set: only our three capabilities (root still has CAP_SETPCAP here). */
  const int max_cap = cap_max_bits();
  for (cap_value_t c = 0; c < max_cap; ++c)
    if (!wanted(c) && prctl(PR_CAPBSET_DROP, c, 0, 0, 0) != 0 && errno != EINVAL) die("PR_CAPBSET_DROP");

  /* 2. Drop root for good, keeping the permitted set across the uid change. */
  if (prctl(PR_SET_KEEPCAPS, 1, 0, 0, 0) != 0) die("PR_SET_KEEPCAPS");
  if (setresgid(gid, gid, gid) != 0) die("setresgid");
  if (setresuid(uid, uid, uid) != 0) die("setresuid");

  /* 3. Exactly the three capabilities: permitted, effective and inheritable. */
  cap_t caps = cap_init();
  if (!caps) die("cap_init");
  if (cap_set_flag(caps, CAP_PERMITTED, NCAPS, kCaps, CAP_SET) != 0 ||
      cap_set_flag(caps, CAP_EFFECTIVE, NCAPS, kCaps, CAP_SET) != 0 ||
      cap_set_flag(caps, CAP_INHERITABLE, NCAPS, kCaps, CAP_SET) != 0)
    die("cap_set_flag");
  if (cap_set_proc(caps) != 0) die("cap_set_proc");
  cap_free(caps);

  /* 4. Ambient: kept across execve of an ordinary program. */
  for (int i = 0; i < NCAPS; ++i)
    if (prctl(PR_CAP_AMBIENT, PR_CAP_AMBIENT_RAISE, kCaps[i], 0, 0) != 0) die("PR_CAP_AMBIENT_RAISE");
  prctl(PR_SET_KEEPCAPS, 0, 0, 0, 0);

  /* 5. Restore LD_LIBRARY_PATH (stripped by glibc because this program is setuid). */
  const char *ld = getenv(kLdEnv);
  if (ld && *ld && setenv("LD_LIBRARY_PATH", ld, 1) != 0) die("setenv");
  unsetenv(kLdEnv);

  execv(argv[1], argv + 1);
  die(argv[1]);
  return 126;
}
