#ifndef __QDL_H__
#define __QDL_H__

#include <stdbool.h>

#include "patch.h"
#include "program.h"

int firehose_init(int fd);
int firehose_reset(int fd);
int firehose_get_storage_info(int fd, int part);
int firehose_provision(int fd);
int sahara_run(int fd, char *prog_mbn);
void print_hex_dump(const char *prefix, const void *buf, size_t len);
int open_in_search_path(const char *filename, int flags);

#define dbg(fmt, ...) do {			\
	if (qdl_debug)				\
		printf(fmt, ##__VA_ARGS__);	\
} while (0)

extern bool qdl_debug;

#endif
