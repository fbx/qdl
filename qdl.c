/*
 * Copyright (c) 2016-2017, Linaro Ltd.
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <sys/types.h>
#include <assert.h>
#include <ctype.h>
#include <dirent.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <libxml/parser.h>
#include <libxml/tree.h>

#include "qdl.h"
#include "patch.h"
#include "ufs.h"

enum {
	QDL_FILE_UNKNOWN,
	QDL_FILE_PATCH,
	QDL_FILE_PROGRAM,
	QDL_FILE_UFS,
	QDL_FILE_CONTENTS,
};

bool qdl_debug;
bool qdl_get_info;
const char **qdl_search_path;

static int detect_type(const char *xml_file)
{
	xmlNode *root;
	xmlDoc *doc;
	xmlNode *node;
	int type = QDL_FILE_UNKNOWN;

	doc = xmlReadFile(xml_file, NULL, 0);
	if (!doc) {
		warnx("[PATCH] failed to parse %s\n", xml_file);
		return -EINVAL;
	}

	root = xmlDocGetRootElement(doc);
	if (!xmlStrcmp(root->name, (xmlChar*)"patches")) {
		type = QDL_FILE_PATCH;
	} else if (!xmlStrcmp(root->name, (xmlChar*)"data")) {
		for (node = root->children; node ; node = node->next) {
			if (node->type != XML_ELEMENT_NODE)
				continue;
			if (!xmlStrcmp(node->name, (xmlChar*)"program")) {
				type = QDL_FILE_PROGRAM;
				break;
			}
			if (!xmlStrcmp(node->name, (xmlChar*)"ufs")) {
				type = QDL_FILE_UFS;
				break;
			}
		}
	} else if (!xmlStrcmp(root->name, (xmlChar*)"contents")) {
		type = QDL_FILE_CONTENTS;
	}

	xmlFreeDoc(doc);

	return type;
}

static int read_file(const char *name, char *buf, size_t len)
{
	ssize_t n;
	int fd;
	int ret = 0;

	fd = open(name, O_RDONLY);
	if (fd < 0)
		return fd;

	n = read(fd, buf, len - 1);
	if (n < 0) {
		warn("failed to read %s", name);
		ret = -EINVAL;
		goto close_fd;
	}
	buf[n] = '\0';

	buf[strcspn(buf, "\n")] = '\0';

close_fd:
	close(fd);
	return ret;
}

static int find_qdl_tty(char *dev_name, size_t dev_name_len)
{
	const char tty_path[] = "/sys/class/tty";
	struct dirent *de;
	int found = -ENOENT;
	char path[PATH_MAX];
	char vid[5];
	char pid[5];
	DIR *dir;
	int ret;

	dir = opendir(tty_path);
	if (!dir)
		err(1, "failed to opendir %s", tty_path);

	while ((de = readdir(dir)) != NULL) {
		if (strncmp(de->d_name, "ttyUSB", 6) != 0)
			continue;

		snprintf(path, sizeof (path), "%s/%s/../../../../idVendor",
			 tty_path, de->d_name);

		ret = read_file(path, vid, sizeof(vid));
		if (ret < 0)
			continue;

		snprintf(path, sizeof (path), "%s/%s/../../../../idProduct",
			 tty_path, de->d_name);

		ret = read_file(path, pid, sizeof(pid));
		if (ret < 0)
			continue;

		if (strcmp(vid, "05c6") || strcmp(pid, "9008"))
			continue;

		snprintf(dev_name, dev_name_len, "/dev/%s", de->d_name);
		found = 0;
	}

	closedir(dir);

	return found;

}

static int tty_open(struct termios *old)
{
	struct termios tios;
	char path[PATH_MAX];
	int ret;
	int fd;

retry:
	ret = find_qdl_tty(path, sizeof(path));
	if (ret < 0) {
		printf("Waiting for QDL tty...\r");
		fflush(stdout);
		sleep(1);
		goto retry;
	}

	fd = open(path, O_RDWR | O_NOCTTY | O_EXCL);
	if (fd < 0) {
		if (errno == EACCES) {
			sleep(1);
			goto retry;
		}
		err(1, "unable to open \"%s\"", path);
	}

	ret = tcgetattr(fd, old);
	if (ret < 0)
		err(1, "unable to retrieve \"%s\" tios", path);

	memset(&tios, 0, sizeof(tios));
	tios.c_cflag = B115200 | CRTSCTS | CS8 | CLOCAL | CREAD;
	tios.c_iflag = IGNPAR;
	tios.c_oflag = 0;

	tcflush(fd, TCIFLUSH);

	ret = tcsetattr(fd, TCSANOW, &tios);
	if (ret < 0)
		err(1, "unable to update \"%s\" tios", path);

	return fd;
}

int open_in_search_path(const char *filename, int flags)
{
	const char **d;
	char path[PATH_MAX];
	int fd;
	int ret;

	for (d = qdl_search_path; *d; d++) {
		ret = snprintf(path, sizeof (path), "%s/%s", *d, filename);
		if (ret < 0 || ret >= sizeof (path))
			continue;

		fd = open(path, flags);
		if (fd != -1)
			return fd;
	}

	return -1;
}

static void add_search_path(const char *path)
{
	int n = 0;

	while (qdl_search_path && qdl_search_path[n])
		n++;

	qdl_search_path = realloc(qdl_search_path, (n + 2) * sizeof (char *));
	qdl_search_path[n] = path;
	qdl_search_path[n + 1] = NULL;
}

static void print_usage(void)
{
	extern const char *__progname;
	fprintf(stderr, "usage: %s OPTS [PROGRAM|PATCH...]\n"
		"Send commands to a Qualcomm device using the firehose protocol.\n"
		"  OPTS can be:\n"
		"  -d, --debug             verbose output\n"
		"  -l, --load ELF          load programmer file using SAHARA protocol\n"
		"  -r, --reset             reset target before exiting\n"
		"  -f, --finalize          finalize provisioning of UFS (WARNING: irreversible)\n"
		"  -s, --search-path DIR   add directory to image search path\n",
		__progname);
}

int main(int argc, char **argv)
{
	struct termios tios;
	char *prog_mbn = NULL;
	int type;
	int ret = 0;
	int fd;
	int opt;
	int n_progs = 0;
	bool qdl_finalize_provisioning = false;
	bool qdl_get_info = false;
	bool qdl_reset = false;

	static struct option options[] = {
		{"debug", no_argument, 0, 'd'},
		{"help", no_argument, 0, 'h'},
		{"load", required_argument, 0, 'l'},
		{"info", no_argument, 0, 'i'},
		{"reset", no_argument, 0, 'r'},
		{"finalize", no_argument, 0, 'f'},
		{"search-path", required_argument, 0, 's'},
		{0, 0, 0, 0}
	};

	if (argc < 2) {
		print_usage();
		return 1;
	}

	while ((opt = getopt_long(argc, argv, "dhl:ifs:", options, NULL )) != -1) {
		switch (opt) {
		case 'd':
			qdl_debug = true;
			break;
		case 'i':
			qdl_get_info = true;
			break;
		case 'f':
			qdl_finalize_provisioning = true;
			break;
		case 'l':
			prog_mbn = optarg;
			break;
		case 'r':
			qdl_reset = true;
			break;
		case 's':
			add_search_path(optarg);
			break;
		case 'h':
			print_usage();
			return 0;
		default:
			print_usage();
			return 1;
		}
	}

	/* default to cwd as search path */
	if (!qdl_search_path)
		add_search_path(".");

	while (optind < argc) {
		char *filename = argv[optind++];

		type = detect_type(filename);
		if (type < 0 || type == QDL_FILE_UNKNOWN)
			errx(1, "failed to detect file type of %s\n", filename);

		switch (type) {
		case QDL_FILE_PATCH:
			ret = patch_load(filename);
			if (ret < 0)
				errx(1, "patch_load %s failed", filename);
			break;
		case QDL_FILE_PROGRAM:
			ret = program_load(filename);
			if (ret < 0)
				errx(1, "program_load %s failed", filename);
			break;
		case QDL_FILE_UFS:
			ret = ufs_load(filename, qdl_finalize_provisioning);
			if (ret < 0)
				errx(1, "ufs_load %s failed", filename);
			break;
		default:
			errx(1, "%s type not yet supported", filename);
			break;
		}
		n_progs++;
	}

	fd = tty_open(&tios);
	if (fd < 0)
		err(1, "failed to open QDL tty");

	if (prog_mbn) {
		ret = sahara_run(fd, prog_mbn);
		if (ret < 0)
			goto out;

		ret = firehose_init(fd);
		if (ret < 0)
			goto out;
	}

	if (qdl_get_info) {
		ret = firehose_get_storage_info(fd, 65210);
		if (ret < 0)
			goto out;
	}

	if (n_progs > 0) {
		ret = firehose_provision(fd);
		if (ret < 0)
			goto out;
	}

out:
	if (qdl_reset)
		firehose_reset(fd);

	if (tcsetattr(fd, TCSANOW, &tios) < 0)
		warn("unable to restore tios of ttyUSB1");
	close(fd);

	return ret ? 1 : 0;
}
