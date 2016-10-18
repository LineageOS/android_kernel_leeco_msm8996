#ifndef __PRINTK_I_LETV_INTERNAL__
#define __PRINTK_I_LETV_INTERNAL__

/*
* The definition here is a part of printk.c
* For LETV add last_kmsg.c and the function will
* use the definition too. So move them here.
*/

enum log_flags {
	LOG_NOCONS	= 1,	/* already flushed, do not print to console */
	LOG_NEWLINE	= 2,	/* text ended with a newline */
	LOG_PREFIX	= 4,	/* text started with a prefix */
	LOG_CONT	= 8,	/* text is a fragment of a continuation line */
};

struct printk_log {
	u64 ts_nsec;		/* timestamp in nanoseconds */
	u16 len;		/* length of entire record */
	u16 text_len;		/* length of text buffer */
	u16 dict_len;		/* length of dictionary buffer */
	u8 facility;		/* syslog facility */
	u8 flags:5;		/* internal record flags */
	u8 level:3;		/* syslog level */
#if defined(CONFIG_LOG_BUF_MAGIC)
	u32 magic;		/* handle for ramdump analysis tools */
	u32 reserved;	/* to 64bit align*/
#endif
	u64 utc_sec;		/* add by letv to mark UTC */
};

#define PREFIX_MAX		64
#define LOG_LINE_MAX		(1024 - PREFIX_MAX)

/* record buffer */
#if defined(CONFIG_HAVE_EFFICIENT_UNALIGNED_ACCESS)
#define LOG_ALIGN 4
#else
#define LOG_ALIGN __alignof__(struct printk_log)
#endif
#define __LOG_BUF_LEN (1 << CONFIG_LOG_BUF_SHIFT)

#if defined(CONFIG_LOG_BUF_MAGIC)
#define LOG_MAGIC_VAL (0x5d7aefca)
static u32 __log_align __used = LOG_ALIGN;
#define LOG_MAGIC(msg) ((msg)->magic = LOG_MAGIC_VAL)
#else
#define LOG_MAGIC(msg)
#endif

extern size_t msg_print_text(const struct printk_log *msg, enum log_flags prev,
			     bool syslog, char *buf, size_t size);
#endif
