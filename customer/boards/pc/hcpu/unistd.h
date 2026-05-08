/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * PC simulator stub of <unistd.h>. MSVC has no native unistd; map the POSIX
 * names project code uses to MSVC's _-prefixed equivalents in <io.h>.
 *
 * IMPORTANT: function-like macros only — `#define read _read` would also
 * substitute the literal `read` token in `__pragma(section(x, read))` from
 * RT-Thread's INIT_*_EXPORT, breaking section placement under MSVC.
 */
#ifndef __PC_UNISTD_H__
#define __PC_UNISTD_H__

#ifdef _MSC_VER
#include <io.h>
#include <process.h>

#ifndef F_OK
#define F_OK 0
#endif
#ifndef R_OK
#define R_OK 4
#endif
#ifndef W_OK
#define W_OK 2
#endif
#ifndef X_OK
#define X_OK 1
#endif

#define access(p, m)            _access((p), (m))
#define close(fd)               _close(fd)
#define read(fd, buf, n)        _read((fd), (buf), (n))
#define write(fd, buf, n)       _write((fd), (buf), (n))
#define lseek(fd, off, whence)  _lseek((fd), (off), (whence))
#define unlink(p)               _unlink(p)
#define ftruncate(fd, sz)       _chsize((fd), (sz))

typedef int ssize_t;

#endif /* _MSC_VER */

#endif /* __PC_UNISTD_H__ */
