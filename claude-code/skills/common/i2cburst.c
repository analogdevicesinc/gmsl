/*
 * i2cburst: back-to-back I2C register accesses for the gmsl-camera-pipeline skill (I2C check).
 * Usage: i2cburst <bus> <addr> <reg16> <count> [gap_us]
 * Reads the register once, then alternates writes of that same value and reads, back to
 * back (or with gap_us between transfers), and prints the number of successes and errors.
 * Use registers without side effects.
 */
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int xfer(int fd, struct i2c_msg *m, int n)
{
	struct i2c_rdwr_ioctl_data d = { .msgs = m, .nmsgs = n };

	return ioctl(fd, I2C_RDWR, &d) < 0 ? -errno : 0;
}

int main(int argc, char **argv)
{
	if (argc < 5)
		return fprintf(stderr, "usage: %s <bus> <addr> <reg16> <count> [gap_us]\n", argv[0]), 2;

	int bus = atoi(argv[1]), addr = strtol(argv[2], NULL, 0);
	int reg = strtol(argv[3], NULL, 0), count = atoi(argv[4]);
	int gap_us = argc > 5 ? atoi(argv[5]) : 0;
	unsigned char rb[2] = { reg >> 8, reg & 0xff }, val, wb[3];
	int nerr[256] = { 0 }, ok = 0, i, r;
	char path[32];
	int fd;

	snprintf(path, sizeof(path), "/dev/i2c-%d", bus);
	fd = open(path, O_RDWR);
	if (fd < 0)
		return perror(path), 1;

	struct i2c_msg rd[2] = { { addr, 0, 2, rb }, { addr, I2C_M_RD, 1, &val } };
	if (xfer(fd, rd, 2))
		return fprintf(stderr, "initial read failed\n"), 1;
	wb[0] = rb[0]; wb[1] = rb[1]; wb[2] = val;
	struct i2c_msg wr = { addr, 0, 3, wb };

	for (i = 0; i < count; i++) {
		r = (i & 1) ? xfer(fd, rd, 2) : xfer(fd, &wr, 1);
		if (gap_us)
			usleep(gap_us);
		if (r)
			nerr[-r & 0xff]++;
		else
			ok++;
	}
	printf("bus %d addr 0x%02x reg 0x%04x val 0x%02x gap %d us: %d ok", bus, addr, reg, val, gap_us, ok);
	for (i = 0; i < 256; i++)
		if (nerr[i])
			printf(", %s x%d", strerror(i), nerr[i]);
	printf("\n");
	return 0;
}
