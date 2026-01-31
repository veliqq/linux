// SPDX-License-Identifier: GPL-2.0-or-later
/* GD ROM driver for the SEGA Dreamcast
 * copyright Adrian McMenamin, 2007
 * Modernized and improved by veliqq, 2026
 * With thanks to Marcus Comstedt and Nathan Keynes
 * for work in reversing PIO and DMA
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/cdrom.h>
#include <linux/bio.h>
#include <linux/blk-mq.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <scsi/scsi.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <asm/delay.h>
#include <mach/dma.h>
#include <mach/sysasic.h>

#define GDROM_DEV_NAME "gdrom"
#define GD_SESSION_OFFSET 150

/* GD Rom commands */
#define GDROM_COM_SOFTRESET 0x08
#define GDROM_COM_EXECDIAG 0x90
#define GDROM_COM_PACKET 0xA0
#define GDROM_COM_IDDEV 0xA1

/* GD Rom registers */
#define GDROM_BASE_REG			0xA05F7000
#define GDROM_ALTSTATUS_REG		(GDROM_BASE_REG + 0x18)
#define GDROM_DATA_REG			(GDROM_BASE_REG + 0x80)
#define GDROM_ERROR_REG			(GDROM_BASE_REG + 0x84)
#define GDROM_INTSEC_REG		(GDROM_BASE_REG + 0x88)
#define GDROM_SECNUM_REG		(GDROM_BASE_REG + 0x8C)
#define GDROM_BCL_REG			(GDROM_BASE_REG + 0x90)
#define GDROM_BCH_REG			(GDROM_BASE_REG + 0x94)
#define GDROM_DSEL_REG			(GDROM_BASE_REG + 0x98)
#define GDROM_STATUSCOMMAND_REG	(GDROM_BASE_REG + 0x9C)
#define GDROM_RESET_REG			(GDROM_BASE_REG + 0x4E4)

#define GDROM_DMA_STARTADDR_REG	(GDROM_BASE_REG + 0x404)
#define GDROM_DMA_LENGTH_REG	(GDROM_BASE_REG + 0x408)
#define GDROM_DMA_DIRECTION_REG	(GDROM_BASE_REG + 0x40C)
#define GDROM_DMA_ENABLE_REG	(GDROM_BASE_REG + 0x414)
#define GDROM_DMA_STATUS_REG	(GDROM_BASE_REG + 0x418)
#define GDROM_DMA_WAIT_REG		(GDROM_BASE_REG + 0x4A0)
#define GDROM_DMA_ACCESS_CTRL_REG	(GDROM_BASE_REG + 0x4B8)

#define GDROM_HARD_SECTOR	2048
#define BLOCK_LAYER_SECTOR	512
#define GD_TO_BLK		4

#define GDROM_DEFAULT_TIMEOUT	(HZ * 7)

static DEFINE_MUTEX(gdrom_mutex);
static const struct {
	int sense_key;
	const char * const text;
} sense_texts[] = {
	{NO_SENSE, "OK"},
	{RECOVERED_ERROR, "Recovered from error"},
	{NOT_READY, "Device not ready"},
	{MEDIUM_ERROR, "Disk not ready"},
	{HARDWARE_ERROR, "Hardware error"},
	{ILLEGAL_REQUEST, "Command has failed"},
	{UNIT_ATTENTION, "Device needs attention - disk may have been changed"},
	{DATA_PROTECT, "Data protection error"},
	{ABORTED_COMMAND, "Command aborted"},
};

static struct platform_device *pd;
static int gdrom_major;
static DECLARE_WAIT_QUEUE_HEAD(command_queue);
static DECLARE_WAIT_QUEUE_HEAD(request_queue);

struct gdromtoc {
	unsigned int entry[99];
	unsigned int first, last;
	unsigned int leadout;
};

struct gdrom_unit {
	struct gendisk *disk;
	struct cdrom_device_info *cd_info;
	int status;
	atomic_t pending;
	atomic_t transfer;
	char disk_type;
	struct gdromtoc *toc;
	struct request_queue *gdrom_rq;
	struct blk_mq_tag_set tag_set;
} gd;

struct gdrom_id {
	char mid;
	char modid;
	char verid;
	char padA[13];
	char mname[16];
	char modname[16];
	char firmver[16];
	char padB[16];
};

/* Prototypes */
static int gdrom_getsense(short *bufstring);
static int gdrom_packetcommand(struct cdrom_device_info *cd_info,
	struct packet_command *command);
static int gdrom_hardreset(struct cdrom_device_info *cd_info);

/* Helpers for busy/wait handling */
static bool gdrom_is_busy(void)
{
	return (__raw_readb(GDROM_ALTSTATUS_REG) & 0x80) != 0;
}

static bool gdrom_data_request(void)
{
	return (__raw_readb(GDROM_ALTSTATUS_REG) & 0x88) == 8;
}

static bool gdrom_wait_clrbusy(void)
{
	unsigned long timeout = jiffies + GDROM_DEFAULT_TIMEOUT;
	while ((__raw_readb(GDROM_ALTSTATUS_REG) & 0x80) &&
		time_before(jiffies, timeout))
		cpu_relax();
	return !gdrom_is_busy();
}

static bool gdrom_wait_busy_sleeps(void)
{
	unsigned long timeout;
	timeout = jiffies + GDROM_DEFAULT_TIMEOUT;
	while (!gdrom_is_busy() && time_before(jiffies, timeout))
		cpu_relax();
	return gdrom_wait_clrbusy();
}

/* Identify device and read ID data */
static void gdrom_identifydevice(void *buf)
{
	int c;
	short *data = buf;

	if (!gdrom_wait_clrbusy()) {
		gdrom_getsense(NULL);
		return;
	}

	__raw_writeb(GDROM_COM_IDDEV, GDROM_STATUSCOMMAND_REG);
	if (!gdrom_wait_busy_sleeps()) {
		gdrom_getsense(NULL);
		return;
	}

	for (c = 0; c < 40; c++)
		data[c] = __raw_readw(GDROM_DATA_REG);
}

/* SPI command helper */
static void gdrom_spicommand(void *spi_string, int buflen)
{
	short *cmd = spi_string;
	unsigned long timeout;

	__raw_writeb(0x08, GDROM_ALTSTATUS_REG);
	__raw_writeb(buflen & 0xFF, GDROM_BCL_REG);
	__raw_writeb((buflen >> 8) & 0xFF, GDROM_BCH_REG);
	__raw_writeb(0, GDROM_INTSEC_REG);
	__raw_writeb(0, GDROM_SECNUM_REG);
	__raw_writeb(0, GDROM_ERROR_REG);

	if (!gdrom_wait_clrbusy()) {
		gdrom_getsense(NULL);
		return;
	}

	timeout = jiffies + GDROM_DEFAULT_TIMEOUT;
	__raw_writeb(GDROM_COM_PACKET, GDROM_STATUSCOMMAND_REG);
	while (!gdrom_data_request() && time_before(jiffies, timeout))
		cpu_relax();

	if (!time_before(jiffies, timeout + 1)) {
		gdrom_getsense(NULL);
		return;
	}

	outsw(GDROM_DATA_REG, cmd, 6);
}

/* Execute Diagnostic command */
static char gdrom_execute_diagnostic(void)
{
	gdrom_hardreset(gd.cd_info);
	if (!gdrom_wait_clrbusy())
		return 0;
	__raw_writeb(GDROM_COM_EXECDIAG, GDROM_STATUSCOMMAND_REG);
	if (!gdrom_wait_busy_sleeps())
		return 0;
	return __raw_readb(GDROM_ERROR_REG);
}

/* Prepare disk spin command */
static int gdrom_preparedisk_cmd(void)
{
	struct packet_command *spin_command;
	spin_command = kzalloc(sizeof(*spin_command), GFP_KERNEL);
	if (!spin_command)
		return -ENOMEM;

	spin_command->cmd[0] = 0x70;
	spin_command->cmd[2] = 0x1f;
	spin_command->buflen = 0;

	atomic_set(&gd.pending, 1);
	gdrom_packetcommand(gd.cd_info, spin_command);

	wait_event_interruptible_timeout(command_queue,
		atomic_read(&gd.pending) == 0, GDROM_DEFAULT_TIMEOUT);

	atomic_set(&gd.pending, 0);
	kfree(spin_command);

	if (gd.status & 0x01) {
		gdrom_getsense(NULL);
		return -EIO;
	}

	return 0;
}

/* Read TOC command */
static int gdrom_readtoc_cmd(struct gdromtoc *toc, int session)
{
	int tocsize = sizeof(*toc);
	struct packet_command *toc_command;
	int err = 0;

	toc_command = kzalloc(sizeof(*toc_command), GFP_KERNEL);
	if (!toc_command)
		return -ENOMEM;

	toc_command->cmd[0] = 0x14;
	toc_command->cmd[1] = session;
	toc_command->cmd[3] = tocsize >> 8;
	toc_command->cmd[4] = tocsize & 0xff;
	toc_command->buflen = tocsize;

	if (atomic_read(&gd.pending)) {
		err = -EBUSY;
		goto cleanup;
	}

	atomic_set(&gd.pending, 1);
	gdrom_packetcommand(gd.cd_info, toc_command);

	wait_event_interruptible_timeout(command_queue,
		atomic_read(&gd.pending) == 0, GDROM_DEFAULT_TIMEOUT);

	if (atomic_read(&gd.pending))
		err = -EINVAL;
	else
		insw(GDROM_DATA_REG, toc, tocsize/2);

	atomic_set(&gd.pending, 0);

cleanup:
	kfree(toc_command);
	return err;
}

/* TOC helpers */
static int get_entry_lba(int track) { return (cpu_to_be32(track & 0xffffff00) - GD_SESSION_OFFSET); }
static int get_entry_q_ctrl(int track) { return (track & 0x000000f0) >> 4; }
static int get_entry_track(int track) { return (track & 0x0000ff00) >> 8; }

/* Get last session for multisession support */
static int gdrom_get_last_session(struct cdrom_device_info *cd_info,
	struct cdrom_multisession *ms_info)
{
	int fentry, lentry, track, data, err;

	if (!gd.toc)
		return -ENOMEM;

	err = gdrom_readtoc_cmd(gd.toc, 1);
	if (err) {
		err = gdrom_readtoc_cmd(gd.toc, 0);
		if (err) {
			pr_info("Could not get CD table of contents\n");
			return -ENXIO;
		}
	}

	fentry = get_entry_track(gd.toc->first);
	lentry = get_entry_track(gd.toc->last);

	track = get_entry_track(gd.toc->last);
	do {
		data = gd.toc->entry[track - 1];
		if (get_entry_q_ctrl(data))
			break;
		track--;
	} while (track >= fentry);

	if ((track > 100) || (track < get_entry_track(gd.toc->first))) {
		pr_info("No data on the last session of the CD\n");
		gdrom_getsense(NULL);
		return -ENXIO;
	}

	ms_info->addr_format = CDROM_LBA;
	ms_info->addr.lba = get_entry_lba(data);
	ms_info->xa_flag = 1;
	return 0;
}

/* Open/Release CD-ROM device */
static int gdrom_open(struct cdrom_device_info *cd_info, int purpose) { return gdrom_preparedisk_cmd(); }
static void gdrom_release(struct cdrom_device_info *cd_info) {}

/* Drive status */
static int gdrom_drivestatus(struct cdrom_device_info *cd_info, int ignore)
{
	char sense = __raw_readb(GDROM_ERROR_REG) & 0xF0;
	if (sense == 0)
		return CDS_DISC_OK;
	if (sense == 0x20)
		return CDS_DRIVE_NOT_READY;
	return CDS_NO_INFO;
}

/* Check media events */
static unsigned int gdrom_check_events(struct cdrom_device_info *cd_info,
				       unsigned int clearing, int ignore)
{
	return (__raw_readb(GDROM_ERROR_REG) & 0xF0) == 0x60 ?
		DISK_EVENT_MEDIA_CHANGE : 0;
}

/* Hard reset */
static int gdrom_hardreset(struct cdrom_device_info *cd_info)
{
	int count;
	__raw_writel(0x1fffff, GDROM_RESET_REG);
	for (count = 0xa0000000; count < 0xa0200000; count += 4)
		__raw_readl(count);
	return 0;
}

/* Packet command interface */
static int gdrom_packetcommand(struct cdrom_device_info *cd_info,
	struct packet_command *command)
{
	gdrom_spicommand(&command->cmd, command->buflen);
	return 0;
}

/* Sense key retrieval */
static int gdrom_getsense(short *bufstring)
{
	struct packet_command *sense_command;
	short sense[5];
	int sense_key;
	int err = -EIO;

	sense_command = kzalloc(sizeof(*sense_command), GFP_KERNEL);
	if (!sense_command)
		return -ENOMEM;

	sense_command->cmd[0] = 0x13;
	sense_command->cmd[4] = 10;
	sense_command->buflen = 10;

	if (atomic_read(&gd.pending) && !gdrom_wait_clrbusy()) {
		err = -EBUSY;
		goto cleanup;
	}

	atomic_set(&gd.pending, 1);
	gdrom_packetcommand(gd.cd_info, sense_command);
	wait_event_interruptible_timeout(command_queue, atomic_read(&gd.pending) == 0,
		GDROM_DEFAULT_TIMEOUT);

	if (!atomic_read(&gd.pending))
		insw(GDROM_DATA_REG, &sense, sense_command->buflen/2);

	sense_key = sense[1] & 0x0F;
	if (sense_key < ARRAY_SIZE(sense_texts))
		pr_info("%s\n", sense_texts[sense_key].text);
	else
		pr_err("Unknown sense key: %d\n", sense_key);

	if (bufstring)
		memcpy(bufstring, &sense[4], 2);

	if (sense_key < 2)
		err = 0;

	atomic_set(&gd.pending, 0);

cleanup:
	kfree(sense_command);
	return err;
}

/* Audio ioctl stub */
static int gdrom_audio_ioctl(struct cdrom_device_info *cdi, unsigned int cmd,
			     void *arg) { return -EINVAL; }

/* CD-ROM ops */
static const struct cdrom_device_ops gdrom_ops = {
	.open			= gdrom_open,
	.release		= gdrom_release,
	.drive_status		= gdrom_drivestatus,
	.check_events		= gdrom_check_events,
	.get_last_session	= gdrom_get_last_session,
	.reset			= gdrom_hardreset,
	.audio_ioctl		= gdrom_audio_ioctl,
	.generic_packet		= cdrom_dummy_generic_packet,
	.capability		= CDC_MULTI_SESSION | CDC_MEDIA_CHANGED |
				  CDC_RESET | CDC_DRIVE_STATUS | CDC_CD_R,
};

/* Block layer DMA read */
static blk_status_t gdrom_readdisk_dma(struct request *req)
{
	int block, block_cnt;
	blk_status_t err;
	struct packet_command *read_command;
	unsigned long timeout;
	struct bio *bio = req->bio;

	read_command = kzalloc(sizeof(*read_command), GFP_KERNEL);
	if (!read_command)
		return BLK_STS_RESOURCE;

	read_command->cmd[0] = 0x30; /* Read command */
	read_command->cmd[1] = 0x20; /* Mode 0x20 = data transfer */

	block = blk_rq_pos(req)/GD_TO_BLK + GD_SESSION_OFFSET;
	block_cnt = blk_rq_sectors(req)/GD_TO_BLK;

	__raw_writel(page_to_phys(bio->bi_io_vec->bv_page) + bio->bi_io_vec->bv_offset,
			GDROM_DMA_STARTADDR_REG);
	__raw_writel(block_cnt * GDROM_HARD_SECTOR, GDROM_DMA_LENGTH_REG);
	__raw_writel(1, GDROM_DMA_DIRECTION_REG); /* read */
	__raw_writel(1, GDROM_DMA_ENABLE_REG);

	read_command->cmd[2] = (block >> 16) & 0xFF;
	read_command->cmd[3] = (block >> 8) & 0xFF;
	read_command->cmd[4] = block & 0xFF;
	read_command->cmd[8] = (block_cnt >> 16) & 0xFF;
	read_command->cmd[9] = (block_cnt >> 8) & 0xFF;
	read_command->cmd[10] = block_cnt & 0xFF;

	__raw_writeb(1, GDROM_ERROR_REG); /* mark DMA active */
	__raw_writeb(0, GDROM_SECNUM_REG);
	__raw_writeb(0, GDROM_BCL_REG);
	__raw_writeb(0, GDROM_BCH_REG);
	__raw_writeb(0, GDROM_DSEL_REG);
	__raw_writeb(0, GDROM_INTSEC_REG);

	timeout = jiffies + HZ / 2;
	while (gdrom_is_busy() && time_before(jiffies, timeout))
		cpu_relax();

	__raw_writeb(GDROM_COM_PACKET, GDROM_STATUSCOMMAND_REG);
	timeout = jiffies + HZ / 2;
	while (gdrom_is_busy() && time_before(jiffies, timeout))
		cpu_relax();

	atomic_set(&gd.pending, 1);
	atomic_set(&gd.transfer, 1);
	outsw(GDROM_DATA_REG, &read_command->cmd, 6);

	timeout = jiffies + HZ / 2;
	while (__raw_readb(GDROM_DMA_STATUS_REG) && time_before(jiffies, timeout))
		cpu_relax();

	__raw_writeb(1, GDROM_DMA_STATUS_REG); /* start transfer */

	wait_event_interruptible_timeout(request_queue,
		atomic_read(&gd.transfer) == 0, GDROM_DEFAULT_TIMEOUT);

	err = atomic_read(&gd.transfer) ? BLK_STS_IOERR : BLK_STS_OK;

	atomic_set(&gd.transfer, 0);
	atomic_set(&gd.pending, 0);
	kfree(read_command);

	blk_mq_end_request(req, err);
	return BLK_STS_OK;
}

/* Block MQ queue function */
static blk_status_t gdrom_queue_rq(struct blk_mq_hw_ctx *hctx,
				   const struct blk_mq_queue_data *bd)
{
	blk_mq_start_request(bd->rq);

	switch (req_op(bd->rq)) {
	case REQ_OP_READ:
		return gdrom_readdisk_dma(bd->rq);
	case REQ_OP_WRITE:
		pr_notice("Read only device - write request ignored\n");
		return BLK_STS_IOERR;
	default:
		pr_debug("gdrom: Non-fs request ignored\n");
		return BLK_STS_IOERR;
	}
}

/* IRQ handlers */
static irqreturn_t gdrom_command_interrupt(int irq, void *dev_id)
{
	gd.status = __raw_readb(GDROM_STATUSCOMMAND_REG);
	if (!atomic_read(&gd.pending))
		return IRQ_HANDLED;

	atomic_set(&gd.pending, 0);
	wake_up_interruptible(&command_queue);
	return IRQ_HANDLED;
}

static irqreturn_t gdrom_dma_interrupt(int irq, void *dev_id)
{
	gd.status = __raw_readb(GDROM_STATUSCOMMAND_REG);
	if (!atomic_read(&gd.transfer))
		return IRQ_HANDLED;

	atomic_set(&gd.transfer, 0);
	wake_up_interruptible(&request_queue);
	return IRQ_HANDLED;
}

/* Request IRQs */
static int gdrom_set_interrupt_handlers(void)
{
	int err;

	err = request_irq(HW_EVENT_GDROM_CMD, gdrom_command_interrupt,
		0, "gdrom_command", &gd);
	if (err)
		return err;

	err = request_irq(HW_EVENT_GDROM_DMA, gdrom_dma_interrupt,
		0, "gdrom_dma", &gd);
	if (err) {
		free_irq(HW_EVENT_GDROM_CMD, &gd);
		return err;
	}

	return 0;
}

/* Initialize DMA mode */
static int gdrom_init_dma_mode(void)
{
	__raw_writeb(0x13, GDROM_ERROR_REG);
	__raw_writeb(0x22, GDROM_INTSEC_REG);
	if (!gdrom_wait_clrbusy())
		return -EBUSY;

	__raw_writeb(0xEF, GDROM_STATUSCOMMAND_REG);
	if (!gdrom_wait_busy_sleeps())
		return -EBUSY;

	__raw_writel(0x8843407F, GDROM_DMA_ACCESS_CTRL_REG); /* access control */
	__raw_writel(9, GDROM_DMA_WAIT_REG);

	return 0;
}

/* Setup CD info */
static void probe_gdrom_setupcd(void)
{
	gd.cd_info->ops = &gdrom_ops;
	gd.cd_info->capacity = 1;
	strcpy(gd.cd_info->name, GDROM_DEV_NAME);
	gd.cd_info->mask = CDC_CLOSE_TRAY|CDC_OPEN_TRAY|CDC_LOCK|CDC_SELECT_DISC;
}

/* Setup disk info */
static void probe_gdrom_setupdisk(void)
{
	gd.disk->major = gdrom_major;
	gd.disk->first_minor = 1;
	gd.disk->minors = 1;
	gd.disk->flags |= GENHD_FL_NO_PART;
	strcpy(gd.disk->disk_name, GDROM_DEV_NAME);
}

/* Setup queue */
static int probe_gdrom_setupqueue(void)
{
	gd.disk->queue = gd.gdrom_rq;
	return gdrom_init_dma_mode();
}

/* Block MQ ops */
static const struct blk_mq_ops gdrom_mq_ops = {
	.queue_rq	= gdrom_queue_rq,
};

/* Probe GD-ROM device */
static int probe_gdrom(struct platform_device *devptr)
{
	struct queue_limits lim = {
		.logical_block_size = GDROM_HARD_SECTOR,
		.max_segments = 1,
		.max_segment_size = 0x40000,
		.features = BLK_FEAT_ROTATIONAL,
	};
	int err;

	memset(&gd, 0, sizeof(gd));

	if (gdrom_execute_diagnostic() != 1) {
		pr_warn("ATA Probe for GDROM failed\n");
		return -ENODEV;
	}

	if (gdrom_outputversion())
		return -ENOMEM;

	gdrom_major = register_blkdev(0, GDROM_DEV_NAME);
	if (gdrom_major <= 0)
		return gdrom_major;

	pr_info("Registered with major number %d\n", gdrom_major);

	gd.cd_info = kzalloc(sizeof(*gd.cd_info), GFP_KERNEL);
	if (!gd.cd_info) {
		err = -ENOMEM;
		goto probe_fail_no_mem;
	}
	probe_gdrom_setupcd();

	err = blk_mq_alloc_sq_tag_set(&gd.tag_set, &gdrom_mq_ops, 1, BLK_MQ_F_BLOCKING);
	if (err)
		goto probe_fail_free_cd_info;

	gd.disk = blk_mq_alloc_disk(&gd.tag_set, &lim, NULL);
	if (IS_ERR(gd.disk)) {
		err = PTR_ERR(gd.disk);
		goto probe_fail_free_tag_set;
	}

	gd.gdrom_rq = gd.disk->queue;
	probe_gdrom_setupdisk();

	if (register_cdrom(gd.disk, gd.cd_info)) {
		err = -ENODEV;
		goto probe_fail_cleanup_disk;
	}

	gd.disk->fops = &gdrom_bdops;
	gd.disk->events = DISK_EVENT_MEDIA_CHANGE;

	err = gdrom_set_interrupt_handlers();
	if (err)
		goto probe_fail_cleanup_disk;

	err = probe_gdrom_setupqueue();
	if (err)
		goto probe_fail_free_irqs;

	gd.toc = kzalloc(sizeof(*gd.toc), GFP_KERNEL);
	if (!gd.toc) {
		err = -ENOMEM;
		goto probe_fail_free_irqs;
	}

	err = add_disk(gd.disk);
	if (err)
		goto probe_fail_add_disk;

	return 0;

probe_fail_add_disk:
	kfree(gd.toc);
probe_fail_free_irqs:
	free_irq(HW_EVENT_GDROM_DMA, &gd);
	free_irq(HW_EVENT_GDROM_CMD, &gd);
probe_fail_cleanup_disk:
	put_disk(gd.disk);
probe_fail_free_tag_set:
	blk_mq_free_tag_set(&gd.tag_set);
probe_fail_free_cd_info:
	kfree(gd.cd_info);
probe_fail_no_mem:
	unregister_blkdev(gdrom_major, GDROM_DEV_NAME);
	gdrom_major = 0;
	pr_warn("Probe failed - error is 0x%X\n", err);
	return err;
}

/* Remove GD-ROM device */
static void remove_gdrom(struct platform_device *devptr)
{
	blk_mq_free_tag_set(&gd.tag_set);
	free_irq(HW_EVENT_GDROM_CMD, &gd);
	free_irq(HW_EVENT_GDROM_DMA, &gd);
	del_gendisk(gd.disk);
	if (gdrom_major)
		unregister_blkdev(gdrom_major, GDROM_DEV_NAME);
	unregister_cdrom(gd.cd_info);
	kfree(gd.cd_info);
	kfree(gd.toc);
}

/* Platform driver struct */
static struct platform_driver gdrom_driver = {
	.probe = probe_gdrom,
	.remove = remove_gdrom,
	.driver = {
		.name = GDROM_DEV_NAME,
	},
};

/* Module init/exit */
static int __init init_gdrom(void)
{
	int rc;

	rc = platform_driver_register(&gdrom_driver);
	if (rc)
		return rc;

	pd = platform_device_register_simple(GDROM_DEV_NAME, -1, NULL, 0);
	if (IS_ERR(pd)) {
		platform_driver_unregister(&gdrom_driver);
		return PTR_ERR(pd);
	}

	return 0;
}

static void __exit exit_gdrom(void)
{
	platform_device_unregister(pd);
	platform_driver_unregister(&gdrom_driver);
}

module_init(init_gdrom);
module_exit(exit_gdrom);

MODULE_AUTHOR("Adrian McMenamin <adrian@mcmen.demon.co.uk>");
MODULE_DESCRIPTION("SEGA Dreamcast GD-ROM Driver");
MODULE_LICENSE("GPL");
