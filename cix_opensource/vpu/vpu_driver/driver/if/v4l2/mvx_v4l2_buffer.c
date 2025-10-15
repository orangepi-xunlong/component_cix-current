/*
 * The confidential and proprietary information contained in this file may
 * only be used by a person authorised under and to the extent permitted
 * by a subsisting licensing agreement from Arm Technology (China) Co., Ltd.
 *
 *            (C) COPYRIGHT 2021-2021 Arm Technology (China) Co., Ltd.
 *                ALL RIGHTS RESERVED
 *
 * This entire notice must be reproduced on all copies of this file
 * and copies of this file may only be made by a person if such person is
 * permitted to do so under the terms of a subsisting license agreement
 * from Arm Technology (China) Co., Ltd.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/time.h>
#include <linux/mvx-v4l2-controls.h>
#include "mvx_ext_if.h"
#include "mvx_log_group.h"
#include "mvx_seq.h"
#include "mvx_v4l2_buffer.h"

/****************************************************************************
 * Static functions and variables
 ****************************************************************************/

static void v4l2_buffer_show(struct mvx_v4l2_buffer *buffer,
                 struct seq_file *s)
{
    struct vb2_v4l2_buffer *v4l2 = &buffer->vb2_v4l2_buffer;
    struct vb2_buffer *vb2 = &v4l2->vb2_buf;
    int is_multi = V4L2_TYPE_IS_MULTIPLANAR(vb2->type);
    int i;
    int ind = 0;

    mvx_seq_printf(s, "mvx_v4l2_buffer", ind, "%px\n", buffer);

    ind++;
    mvx_seq_printf(s, "vb2", ind, "%px\n", vb2);

    ind++;
    mvx_seq_printf(s, "index", ind, "%u\n", vb2->index);
    mvx_seq_printf(s, "type", ind, "%u (multi: %s)\n",
               vb2->type, is_multi ? "yes" : "no");
    mvx_seq_printf(s, "flags", ind, "0x%08x\n", v4l2->flags);
    mvx_seq_printf(s, "field", ind, "%u\n", v4l2->field);

#if KERNEL_VERSION(4, 5, 0) <= LINUX_VERSION_CODE
    mvx_seq_printf(s, "timestamp", ind, "%llu\n", vb2->timestamp);
#else
    mvx_seq_printf(s, "timestamp", ind, "\n");
    ind++;
    mvx_seq_printf(s, "tv_sec", ind, "%lu\n", v4l2->timestamp.tv_sec);
    mvx_seq_printf(s, "tv_usec", ind, "%lu\n", v4l2->timestamp.tv_usec);
    ind--;
#endif
    mvx_seq_printf(s, "timecode", ind, "\n");
    ind++;
    mvx_seq_printf(s, "type", ind, "%u\n", v4l2->timecode.type);
    mvx_seq_printf(s, "flags", ind, "%u\n", v4l2->timecode.flags);
    mvx_seq_printf(s, "frames", ind, "%u\n", v4l2->timecode.frames);
    mvx_seq_printf(s, "seconds", ind, "%u\n", v4l2->timecode.seconds);
    mvx_seq_printf(s, "minutes", ind, "%u\n", v4l2->timecode.minutes);
    mvx_seq_printf(s, "hours", ind, "%u\n", v4l2->timecode.hours);
    ind--;

    mvx_seq_printf(s, "sequence", ind, "%u\n", v4l2->sequence);
    mvx_seq_printf(s, "memory", ind, "%u\n", vb2->memory);

    mvx_seq_printf(s, "num_planes", ind, "%u\n", vb2->num_planes);

    mvx_seq_printf(s, "planes", ind, "\n");
    ind++;
    for (i = 0; i < vb2->num_planes; ++i) {
        char tag[10];
        struct vb2_plane *plane = &vb2->planes[i];

        scnprintf(tag, sizeof(tag), "#%d", i);
        mvx_seq_printf(s, tag, ind,
                   "bytesused: %10u, length: %10u, m.offset: %10u, m.userptr: %10lu, m.fd: %10d, data_offset: %10u\n",
                   plane->bytesused,
                   plane->length,
                   plane->m.offset,
                   plane->m.userptr,
                   plane->m.fd,
                   plane->data_offset);
    }

    ind--;
}

static int buffer_stat_show(struct seq_file *s,
                void *v)
{
    struct mvx_v4l2_buffer *vbuf = s->private;

    v4l2_buffer_show(vbuf, s);
    seq_puts(s, "\n");
    mvx_buffer_show(&vbuf->buf, s);

    return 0;
}

static int buffer_stat_open(struct inode *inode,
                struct file *file)
{
    return single_open(file, buffer_stat_show, inode->i_private);
}

static const struct file_operations buffer_stat_fops = {
    .open    = buffer_stat_open,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = seq_release
};

static int buffer_debugfs_init(struct dentry *parent,
                   struct mvx_v4l2_buffer *vbuf)
{
    char name[20];
    struct dentry *dentry;

    scnprintf(name, sizeof(name), "buffer%u", to_vb2_buf(vbuf)->index);
    vbuf->dentry = debugfs_create_dir(name, parent);
    if (IS_ERR_OR_NULL(vbuf->dentry))
        return -ENOMEM;

    dentry = debugfs_create_file("stat", 0400, vbuf->dentry, vbuf,
                     &buffer_stat_fops);
    if (IS_ERR_OR_NULL(dentry))
        return -ENOMEM;

    return 0;
}

/**
 * get_bytesused() - Get total number of bytes used for Vb2 buffer.
 */
static size_t get_bytesused(struct vb2_buffer *b)
{
    size_t size;
    uint32_t i;

    for (i = 0, size = 0; i < b->num_planes; i++)
        size += b->planes[i].bytesused;

    return size;
}

static int clear_bytesused(struct vb2_buffer *b)
{
    uint32_t i;

    for (i = 0; i < b->num_planes; i++)
        b->planes[i].bytesused = 0;

    return 0;
}

/* Update mvx_buffer flags from vb2_buffer flags */
static int update_mvx_flags(struct mvx_buffer *buf,
                struct vb2_buffer *b)
{
    struct mvx_v4l2_port *vport = vb2_get_drv_priv(b->vb2_queue);
    struct mvx_session *session = &vport->vsession->session;
    enum mvx_nalu_format nalu_format = session->nalu_format;
    struct vb2_v4l2_buffer *vb2_v4l2 = to_vb2_v4l2_buffer(b);
    __u32 flags = vb2_v4l2->flags;
    __u32 osd_flags = (buf->flags & MVX_BUFFER_FRAME_FLAG_OSD_MASK);
    buf->flags = osd_flags;

    if (V4L2_TYPE_IS_OUTPUT(b->type) != false && get_bytesused(b) == 0)
        flags |= V4L2_BUF_FLAG_LAST;

    if (flags & V4L2_BUF_FLAG_LAST)
        buf->flags |= MVX_BUFFER_EOS;

    if (mvx_is_frame(buf->format) && buf->dir == MVX_DIR_INPUT) {
        if (flags & V4L2_BUF_FLAG_KEYFRAME) {
            //idr flag has conflict with B frames, disable idr when B frames has positive value
            if (session->b_frames == 0)
                //encode frame port for idr flag
                buf->flags |= MVX_BUFFER_FRAME_FLAG_FORCE_IDR;
        }
    }

    if (mvx_is_afbc(buf->format)) {
        if ((flags & V4L2_BUF_FLAG_MVX_AFBC_TILED_HEADERS) == V4L2_BUF_FLAG_MVX_AFBC_TILED_HEADERS)
            buf->flags |= MVX_BUFFER_AFBC_TILED_HEADERS;

        if ((flags & V4L2_BUF_FLAG_MVX_AFBC_TILED_BODY) == V4L2_BUF_FLAG_MVX_AFBC_TILED_BODY)
            buf->flags |= MVX_BUFFER_AFBC_TILED_BODY;

        if ((flags & V4L2_BUF_FLAG_MVX_AFBC_32X8_SUPERBLOCK) == V4L2_BUF_FLAG_MVX_AFBC_32X8_SUPERBLOCK)
            buf->flags |= MVX_BUFFER_AFBC_32X8_SUPERBLOCK;
        if (buf->dir == MVX_DIR_INPUT) //encode frame port
        {
            if ((flags & V4L2_BUF_FLAG_MVX_BUFFER_EPR) == V4L2_BUF_FLAG_MVX_BUFFER_EPR) {
                buf->flags |= MVX_BUFFER_FRAME_FLAG_GENERAL;
            }
            if ((flags & V4L2_BUF_FLAG_MVX_BUFFER_ROI) == V4L2_BUF_FLAG_MVX_BUFFER_ROI) {
                buf->flags |= MVX_BUFFER_FRAME_FLAG_ROI;
            }
            if ((flags & V4L2_BUF_FLAG_MVX_BUFFER_CHR) == V4L2_BUF_FLAG_MVX_BUFFER_CHR) {
                buf->flags |= MVX_BUFFER_FRAME_FLAG_CHR;
            }
            if ((flags & V4L2_BUF_FLAG_MVX_BUFFER_RESET_RC) == V4L2_BUF_FLAG_MVX_BUFFER_RESET_RC)
            {
                buf->flags |= MVX_BUFFER_FRAME_FLAG_RESET_RC;
            }
        }
    } else if (mvx_is_bitstream(buf->format)) {
        if (buf->dir == MVX_DIR_INPUT) {
            //decode bitstream port
            if ((flags & V4L2_BUF_FLAG_END_OF_SUB_FRAME) == V4L2_BUF_FLAG_END_OF_SUB_FRAME){
                buf->flags |= MVX_BUFFER_END_OF_SUB_FRAME;
            }
            if (flags & V4L2_BUF_FLAG_KEYFRAME)
                buf->flags |= MVX_BUFFER_EOF;
            if ((flags & V4L2_BUF_FLAG_MVX_CODEC_CONFIG) == V4L2_BUF_FLAG_MVX_CODEC_CONFIG)
                buf->flags |= MVX_BUFFER_CODEC_CONFIG;

            /*
             * MMF does not set nalu_format and usually one frame per buffer.
             * Hence, MVX_BUFFER_EOF flag must be set, otherwise the timestamp
             * may incorrect. If one buffer contains multiple frames, nalu_format
             * must be set.
             */
            if (nalu_format == MVX_NALU_FORMAT_UNDEFINED)
                buf->flags |= MVX_BUFFER_EOF;
        }
    } else if (mvx_is_frame(buf->format)) {
        if (buf->dir == MVX_DIR_OUTPUT) {
            //decode frame port
            if (flags & V4L2_BUF_FRAME_FLAG_ROTATION_MASK) {
                if ((flags & V4L2_BUF_FRAME_FLAG_ROTATION_MASK) == V4L2_BUF_FRAME_FLAG_ROTATION_90) {
                    buf->flags |= MVX_BUFFER_FRAME_FLAG_ROTATION_90;
                } else if ((flags & V4L2_BUF_FRAME_FLAG_ROTATION_MASK) == V4L2_BUF_FRAME_FLAG_ROTATION_180) {
                    buf->flags |= MVX_BUFFER_FRAME_FLAG_ROTATION_180;
                } else if ((flags & V4L2_BUF_FRAME_FLAG_ROTATION_MASK) == V4L2_BUF_FRAME_FLAG_ROTATION_270) {
                    buf->flags |= MVX_BUFFER_FRAME_FLAG_ROTATION_270;
                }
            }
            if (flags & V4L2_BUF_FRAME_FLAG_SCALING_MASK) {
                if ((flags & V4L2_BUF_FRAME_FLAG_SCALING_MASK) == V4L2_BUF_FRAME_FLAG_SCALING_2) {
                    buf->flags |= MVX_BUFFER_FRAME_FLAG_SCALING_2;
                } else if ((flags & V4L2_BUF_FRAME_FLAG_SCALING_MASK) == V4L2_BUF_FRAME_FLAG_SCALING_4) {
                    buf->flags |= MVX_BUFFER_FRAME_FLAG_SCALING_4;
                }
            }
        } else if (buf->dir == MVX_DIR_INPUT) {
            //encode frame port
            if (flags & V4L2_BUF_FRAME_FLAG_MIRROR_MASK) {
                if ((flags & V4L2_BUF_FRAME_FLAG_MIRROR_MASK) == V4L2_BUF_FRAME_FLAG_MIRROR_HORI) {
                    buf->flags |= MVX_BUFFER_FRAME_FLAG_MIRROR_HORI;
                } else if ((flags & V4L2_BUF_FRAME_FLAG_MIRROR_MASK) == V4L2_BUF_FRAME_FLAG_MIRROR_VERT) {
                    buf->flags |= MVX_BUFFER_FRAME_FLAG_MIRROR_VERT;
                }
            }
            if (flags & V4L2_BUF_ENCODE_FLAG_ROTATION_MASK) {
                if ((flags & V4L2_BUF_ENCODE_FLAG_ROTATION_MASK) == V4L2_BUF_ENCODE_FLAG_ROTATION_90) {
                    buf->flags |= MVX_BUFFER_FRAME_FLAG_ROTATION_90;
                } else if ((flags & V4L2_BUF_ENCODE_FLAG_ROTATION_MASK) == V4L2_BUF_ENCODE_FLAG_ROTATION_180) {
                    buf->flags |= MVX_BUFFER_FRAME_FLAG_ROTATION_180;
                } else if ((flags & V4L2_BUF_ENCODE_FLAG_ROTATION_MASK) == V4L2_BUF_ENCODE_FLAG_ROTATION_270) {
                    buf->flags |= MVX_BUFFER_FRAME_FLAG_ROTATION_270;
                }
            }
            if ((flags & V4L2_BUF_FLAG_MVX_BUFFER_EPR) == V4L2_BUF_FLAG_MVX_BUFFER_EPR) {
                buf->flags |= MVX_BUFFER_FRAME_FLAG_GENERAL;
            }
            if ((flags & V4L2_BUF_FLAG_MVX_BUFFER_ROI) == V4L2_BUF_FLAG_MVX_BUFFER_ROI) {
                buf->flags |= MVX_BUFFER_FRAME_FLAG_ROI;
            }
            if ((flags & V4L2_BUF_FLAG_MVX_BUFFER_CHR) == V4L2_BUF_FLAG_MVX_BUFFER_CHR) {
                buf->flags |= MVX_BUFFER_FRAME_FLAG_CHR;
            }
            if ((flags & V4L2_BUF_FLAG_MVX_BUFFER_RESET_RC) == V4L2_BUF_FLAG_MVX_BUFFER_RESET_RC)
            {
                buf->flags |= MVX_BUFFER_FRAME_FLAG_RESET_RC;
            }
        }

    } else {
        MVX_LOG_PRINT(&mvx_log_if, MVX_LOG_WARNING,
                  "unrecognized buffer format!.");

    }

    return 0;
}

static bool is_contiguous_planes(unsigned int pixelformat, int memory)
{
    return (memory != V4L2_MEMORY_MMAP) &&
           (pixelformat == V4L2_PIX_FMT_YUV420 ||
            pixelformat == V4L2_PIX_FMT_NV12 ||
            pixelformat == V4L2_PIX_FMT_NV21 ||
            pixelformat == V4L2_PIX_FMT_P010);
}

/* Update mvx_buffer from mvx_v4l2_buffer */
static int update_mvx_buffer(struct mvx_v4l2_buffer *vbuf)
{
    struct vb2_buffer *vb2 = to_vb2_buf(vbuf);
    struct mvx_buffer *mvx_buf = &vbuf->buf;
    struct mvx_v4l2_port *vport = vb2_get_drv_priv(vb2->vb2_queue);
    struct mvx_session *session = &vport->vsession->session;
    int i;
    int ret;

    if ((!session->dual_afbc_downscaled && vb2->num_planes != mvx_buf->nplanes) ||
        (session->dual_afbc_downscaled && vb2->num_planes > mvx_buf->nplanes)) {
        MVX_LOG_PRINT(&mvx_log_if, MVX_LOG_WARNING,
                  "VB2 and MVX buffers have different number of planes. vb2_planes=%u, mvx_planes=%u.",
                  vb2->num_planes, mvx_buf->nplanes);
        return -EINVAL;
    }

#if KERNEL_VERSION(4, 5, 0) <= LINUX_VERSION_CODE
    mvx_buf->user_data = vb2->timestamp;
#else
    {
        struct timeval *ts = &vbuf->vb2_v4l2_buffer.timestamp;

        mvx_buf->user_data = ((uint64_t)ts->tv_sec << 32) |
                     (ts->tv_usec & 0xffffffff);
    }
#endif

    for (i = 0; i < vb2->num_planes; i++) {
        unsigned int offset = vb2->planes[i].data_offset;

        /*
         * For single planar mmap buffers the offset is carried by
         * the lower part of the offset.
         */
        if (vb2->memory == V4L2_MEMORY_MMAP)
            offset += vb2->planes[i].m.offset & ~PAGE_MASK;

        /* MVX filled is the number of bytes excluding the offset. */
        ret = mvx_buffer_filled_set(mvx_buf, i,
                        vb2->planes[i].bytesused - offset, offset);
        if (ret != 0)
            return ret;
        mvx_buf->planes[i].length = vb2->planes[i].length;
    }

    mvx_buf->is_contiguous =
        is_contiguous_planes(vport->port->pixelformat, vb2->memory);

    ret = update_mvx_flags(mvx_buf, to_vb2_buf(vbuf));
    if (ret != 0)
        return 0;

    return 0;
}

static int update_v4l2_bytesused(struct mvx_v4l2_buffer *vbuf)
{
    struct vb2_buffer *b = to_vb2_buf(vbuf);
    struct mvx_buffer *buf = &vbuf->buf;
    struct mvx_v4l2_port *vport = vb2_get_drv_priv(b->vb2_queue);
    struct mvx_session *session = &vport->vsession->session;
    int i;

    if ((!session->dual_afbc_downscaled && b->num_planes != buf->nplanes) ||
        (session->dual_afbc_downscaled && b->num_planes > buf->nplanes)) {
        MVX_LOG_PRINT(&mvx_log_if, MVX_LOG_WARNING,
                  "VB2 and MVX buffers have different number of planes. vb2_planes=%u, mvx_planes=%u.",
                  b->num_planes, buf->nplanes);
        return -EINVAL;
    }

    /*
     * MVX filled is the number of bytes excluding the offset. The total
     * length is calculated as 'filled + offset' and should be <= length.
     *
     * V4L2 bytesused is the total length including the offset.
     * bytesused should be <= length and bytesused >= offset.
     */

    for (i = 0; i < b->num_planes; i++) {
        b->planes[i].bytesused =
            buf->planes[i].filled + buf->planes[i].offset;
        b->planes[i].data_offset = buf->planes[i].offset;
    }

    return 0;
}

static int update_vb2_flags(struct mvx_v4l2_buffer *vbuf)
{
    struct vb2_v4l2_buffer *b = &vbuf->vb2_v4l2_buffer;
    struct mvx_buffer *buf = &vbuf->buf;

    b->flags &= ~(V4L2_BUF_FLAG_ERROR |
              V4L2_BUF_FLAG_KEYFRAME |
              V4L2_BUF_FLAG_LAST);

    if (buf->flags & MVX_BUFFER_EOS)
        b->flags |= V4L2_BUF_FLAG_LAST;

    if (buf->flags & MVX_BUFFER_SYNCFRAME)
        b->flags |= V4L2_BUF_FLAG_KEYFRAME;

    if (buf->flags & MVX_BUFFER_CORRUPT)
        b->flags |= V4L2_BUF_FLAG_ERROR;

    if (buf->flags & MVX_BUFFER_REJECTED)
        clear_bytesused(&b->vb2_buf);

    if (buf->flags & MVX_BUFFER_DECODE_ONLY)
        b->flags |= V4L2_BUF_FLAG_MVX_DECODE_ONLY;

    if (buf->flags & MVX_BUFFER_CODEC_CONFIG)
        b->flags |= V4L2_BUF_FLAG_MVX_CODEC_CONFIG;

    if (buf->flags & MVX_BUFFER_AFBC_TILED_HEADERS)
        b->flags |= V4L2_BUF_FLAG_MVX_AFBC_TILED_HEADERS;

    if (buf->flags & MVX_BUFFER_AFBC_TILED_BODY)
        b->flags |= V4L2_BUF_FLAG_MVX_AFBC_TILED_BODY;

    if (buf->flags & MVX_BUFFER_AFBC_32X8_SUPERBLOCK)
        b->flags |= V4L2_BUF_FLAG_MVX_AFBC_32X8_SUPERBLOCK;

    if (buf->flags & MVX_BUFFER_FRAME_PRESENT && buf->dir == MVX_DIR_OUTPUT)
        b->flags |= V4L2_BUF_FLAG_MVX_BUFFER_FRAME_PRESENT;

    if (buf->flags & MVX_BUFFER_FRAME_NEED_REALLOC)
        b->flags |= V4L2_BUF_FLAG_MVX_BUFFER_NEED_REALLOC;

    if (buf->flags & MVX_BUFFER_ENC_STATS)
        b->flags |= V4L2_BUF_FLAG_MVX_BUFFER_ENC_STATS;

    /*
     * For encoder(especially VP8/VP9/AV1), one encoded frame may split into
     * several buffers, and set MVX_BUFFER_EOF flag with the last subframe.
     * Hence, add V4L2_BUF_FLAG_END_OF_SUB_FRAME flag to other subframes to
     * notify user.
     */
    if (!(buf->flags & MVX_BUFFER_EOF) && mvx_is_bitstream(buf->format))
        b->flags |= V4L2_BUF_FLAG_END_OF_SUB_FRAME;

    return 0;
}

/****************************************************************************
 * Exported functions and variables
 ****************************************************************************/

int mvx_v4l2_buffer_construct(struct mvx_v4l2_buffer *vbuf,
                  struct mvx_v4l2_session *vsession,
                  enum mvx_direction dir,
                  unsigned int nplanes,
                  struct sg_table **sgt)
{
    int ret;

    ret = mvx_buffer_construct(&vbuf->buf, vsession->ext->dev,
                   &vsession->session.mmu, dir,
                   nplanes, sgt);
    if (ret != 0)
        return ret;

    if (IS_ENABLED(CONFIG_DEBUG_FS)) {
        struct mvx_v4l2_port *vport = &vsession->port[dir];

        ret = buffer_debugfs_init(vport->dentry, vbuf);
        if (ret != 0) {
            MVX_SESSION_WARN(&vsession->session,
                     "Failed to create buffer debugfs entry.");
            goto destruct_buffer;
        }
    }

    return 0;

destruct_buffer:
    mvx_buffer_destruct(&vbuf->buf);

    return ret;
}

void mvx_v4l2_buffer_destruct(struct mvx_v4l2_buffer *vbuf)
{
    mvx_buffer_destruct(&vbuf->buf);

    if (IS_ENABLED(CONFIG_DEBUG_FS))
        debugfs_remove_recursive(vbuf->dentry);
}

struct mvx_v4l2_buffer *mvx_buffer_to_v4l2_buffer(struct mvx_buffer *buffer)
{
    return container_of(buffer, struct mvx_v4l2_buffer, buf);
}

/* Update mvx_v4l2_buffer from vb2_buffer */
int mvx_v4l2_buffer_set(struct mvx_v4l2_buffer *vbuf,
            struct vb2_buffer *b)
{
    int ret;

    ret = update_mvx_buffer(vbuf);
    if (ret != 0)
        return ret;

    return 0;
}

enum vb2_buffer_state mvx_v4l2_buffer_update(struct mvx_v4l2_buffer *vbuf)
{
    struct vb2_buffer *vb2 = to_vb2_buf(vbuf);
    struct mvx_buffer *mvx_buf = &vbuf->buf;
    int ret = 0;

    if (!V4L2_TYPE_IS_OUTPUT(vb2->type))
        ret = update_v4l2_bytesused(vbuf);

    if (ret != 0)
        goto error;

    ret = update_vb2_flags(vbuf);
    if (ret != 0 ||
        (vbuf->vb2_v4l2_buffer.flags & V4L2_BUF_FLAG_ERROR) != 0)
        goto error;

#if KERNEL_VERSION(4, 5, 0) <= LINUX_VERSION_CODE
    vb2->timestamp = mvx_buf->user_data;
#else
    {
        struct timeval *ts = &vbuf->vb2_v4l2_buffer.timestamp;

        ts->tv_sec = mvx_buf->user_data >> 32;
        ts->tv_usec = mvx_buf->user_data & 0xffffffff;
    }
#endif

    return VB2_BUF_STATE_DONE;

error:
    return VB2_BUF_STATE_ERROR;
}
