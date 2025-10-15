// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023-2024 Arm Technology (China) Co. Ltd. */

#include <linux/types.h>
#include <asm/cacheflush.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/of_iommu.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/iommu.h>
#include <linux/bitmap.h>
#include <linux/version.h>
#include <linux/dma-mapping.h>
#include <linux/acpi.h>
#include "config.h"
#include "aipu_priv.h"
#include "aipu_mm.h"
#include "aipu_common.h"
#include "aipu_dma_buf.h"
#include "v2.h"

#define EXIT_ENCODE_BUF_SIZE (1024)
static struct aipu_mem_region *aipu_mm_find_region_no_lock(struct aipu_memory_manager *mm,
							   u64 iova, char *log_str);
static struct aipu_tcb_buf *aipu_mm_find_tbuf_in_region_no_lock(struct aipu_memory_manager *mm,
								struct aipu_mem_region *reg,
								u64 iova);
static struct aipu_mem_region *aipu_mm_find_region(struct aipu_memory_manager *mm,
						   u64 iova, char *log_str);
static struct device *aipu_mm_create_child_dev(struct device *dev, u32 idx)
{
	struct device *child = NULL;

#if (KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE)
	if (idx) {
		dev_err(dev, "multiple mem regions are not supported (idx %d)\n", idx);
		return NULL;
	}
	return dev;
#endif

	child = devm_kzalloc(dev, sizeof(*child), GFP_KERNEL);
	if (!child)
		return NULL;

	device_initialize(child);
	dev_set_name(child, "%s:%s%u", dev_name(dev), "reserved_", idx);
	child->parent = dev;
	child->coherent_dma_mask = dev->coherent_dma_mask;
	child->dma_mask = dev->dma_mask;
	child->dma_parms = devm_kzalloc(dev, sizeof(*child->dma_parms),
					GFP_KERNEL);
	child->bus = dev->bus;
	if (!child->dma_parms)
		goto err;

#if KERNEL_VERSION(5, 10, 0) > LINUX_VERSION_CODE
	child->dma_pfn_offset = dev->dma_pfn_offset;
#else
	child->dma_range_map = dev->dma_range_map;
#endif

	return child;

err:
	put_device(child);
	return NULL;
}

int aipu_mm_hold_tcb_buf_alloc(struct aipu_memory_manager *mm, struct aipu_job *kern_job)
{
	struct aipu_hold_tcb_buf *htbuf = NULL;
	struct aipu_hold_tcb_buf *htbuf_head = mm->hold_tcb_head;
	struct aipu_tcb_buf *prev_tbuf = NULL;
	struct aipu_tcb_buf *curr_tbuf = NULL;
	struct aipu_mem_region *reg = NULL;
	struct aipu_buf_request buf;
	unsigned long flags;
	int ret = 0;
	struct aipu_job_desc *desc = &kern_job->desc;
	struct aipu_tcb *tcb = NULL;
	struct aipu_tcb *prev = NULL;
	char *va = NULL;
	u64 encode_offset = sizeof(struct aipu_tcb) * 2;
	int malloc_flag = 0;
	bool find_htbuf = false;
	unsigned char exit_encode[] = {
		0x00, 0x80, 0x4f, 0x01,
		0xff, 0x7f, 0x0c, 0x40,
		0xff, 0x7f, 0x0c, 0x40,
		0xff, 0x7f, 0x0c, 0x40
	};

	// find idle hold tcb buffer
	spin_lock_irqsave(&mm->shlock, flags);
	list_for_each_entry(htbuf, &htbuf_head->node, node) {
		if (htbuf->status == AIPU_MEM_HOLD_TYPE_IDLE &&
		    htbuf->index != htbuf_head->hold_index) {
			htbuf->status = AIPU_MEM_HOLD_TYPE_LINKING;
			find_htbuf = true;
			dev_dbg(mm->dev, "found hold index %d tcb  0x%llx from list.\n",
				htbuf->index, htbuf->head);
			break;
		}
	}
	spin_unlock_irqrestore(&mm->shlock, flags);

	// malloc new hold tcb buffer
	if (!find_htbuf) {
		htbuf = kmem_cache_zalloc(mm->hold_tbuf_cache, GFP_KERNEL);
		if (!htbuf) {
			ret = -ENOMEM;
			dev_err(mm->dev, "alloc cache hold tcb buffer failed");
			goto CACHE_FAIL;
		}

		INIT_LIST_HEAD(&htbuf->node);
		memset(&buf, 0, sizeof(buf));
		buf.bytes = sizeof(struct aipu_tcb) + EXIT_ENCODE_BUF_SIZE;
		buf.align_in_page = 1;
		buf.region = AIPU_BUF_REGION_DEFAULT;
		buf.data_type = AIPU_MM_DATA_TYPE_TCB;
		buf.asid = AIPU_BUF_ASID_0;
		ret = aipu_mm_alloc(mm, &buf, NULL);
		if (ret) {
			dev_err(mm->dev, "alloc hold tcb buffer failed");
			ret = -ENOMEM;
			goto MALLOC_TCB_FAIL;
		}

		malloc_flag = 1;
		spin_lock_irqsave(&mm->shlock, flags);
		htbuf->desc = buf.desc;
		htbuf->head = buf.desc.pa;
		htbuf->next_head = 0;
		htbuf->prev_tail = 0;
		htbuf->prev_head = 0;
		htbuf->prev_hold_tcb = 0;
		htbuf->status = AIPU_MEM_HOLD_TYPE_LINKING;
		htbuf->index = htbuf_head->nums;
		htbuf_head->nums++;
		list_add(&htbuf->node, &htbuf_head->node);
		spin_unlock_irqrestore(&mm->shlock, flags);

		va = aipu_mm_get_va(mm, htbuf->head + encode_offset);
		if (!va) {
			dev_err(mm->dev, "get encode va failed.");
			ret = -EFAULT;
			goto VA_FAIL;
		}

		memcpy(va, exit_encode, 16);
		dev_dbg(mm->dev, "malloc hold index %d\n", htbuf->index);
	}

	tcb = aipu_mm_get_tcb(mm, htbuf->head);
	if (!tcb) {
		dev_err(mm->dev, "get hold tcb buffer failed");
		ret = -EFAULT;
		goto VA_FAIL;
	}

	prev = aipu_mm_get_tcb(mm, desc->last_task_tcb_pa);
	if (!prev) {
		dev_err(mm->dev, "get prev tcb buffer failed");
		ret = -EFAULT;
		goto VA_FAIL;
	}

	// Modify hold tcb data
	spin_lock_irqsave(&mm->shlock, flags);
	memcpy(tcb, prev, sizeof(struct aipu_tcb));
	tcb->flag = TCB_FLAG_TASK_TYPE_TASK;
	tcb->flag |= TCB_FLAG_END_TYPE_GROUP_END;
	tcb->next = 0;
	tcb->spc = lower_32_bits(htbuf->head + encode_offset -
				 aipu_mm_get_asid_base(mm, AIPU_BUF_ASID_0));
	tcb->tcbp = lower_32_bits(htbuf->head - aipu_mm_get_asid_base(mm, AIPU_BUF_ASID_0));
	spin_unlock_irqrestore(&mm->shlock, flags);

	reg = aipu_mm_find_region(mm, htbuf->head, "find_tcb hold");
	if (!reg) {
		dev_err(mm->dev, "Can't found region at iova 0x%llx", htbuf->head);
		ret = -EFAULT;
		goto VA_FAIL;
	}

	spin_lock_irqsave(&mm->slock, flags);
	curr_tbuf = aipu_mm_find_tbuf_in_region_no_lock(mm, reg, htbuf->head);
	if (!curr_tbuf) {
		dev_err(mm->dev, "no TCB buffer is found at iova 0x%llx", htbuf->head);
		ret = -EFAULT;
		spin_unlock_irqrestore(&mm->slock, flags);
		goto VA_FAIL;
	}

	spin_unlock_irqrestore(&mm->slock, flags);

	reg = aipu_mm_find_region(mm, desc->last_task_tcb_pa, "find_tcb task");
	if (!reg) {
		dev_err(mm->dev, "no TCB buffer is found at iova 0x%llx", desc->last_task_tcb_pa);
		ret = -EFAULT;
		goto VA_FAIL;
	}

	spin_lock_irqsave(&mm->slock, flags);
	prev_tbuf = aipu_mm_find_tbuf_in_region_no_lock(mm, reg, desc->last_task_tcb_pa);
	if (!prev_tbuf) {
		dev_err(mm->dev, "no TCB buffer is found at iova 0x%llx", desc->last_task_tcb_pa);
		ret = -EFAULT;
		spin_unlock_irqrestore(&mm->slock, flags);
		goto VA_FAIL;
	}
	spin_unlock_irqrestore(&mm->slock, flags);

	spin_lock_irqsave(&mm->shlock, flags);
	curr_tbuf->tail = htbuf->head;
	curr_tbuf->tail_tcb = tcb;
	curr_tbuf->dep_job_id = 0;
	curr_tbuf->pinned = false;
	prev_tbuf->tail = desc->last_task_tcb_pa;
	prev_tbuf->tail_tcb = prev;
	prev->next = htbuf->head;
	htbuf->status = AIPU_MEM_HOLD_TYPE_LINK_PREV;
	htbuf->hold_tcb = tcb;
	htbuf->prev_tail = desc->last_task_tcb_pa;
	htbuf->prev_head = desc->head_tcb_pa;
	kern_job->curr_hold_tcb = htbuf->head;
	htbuf->prev_tbuf = prev_tbuf;
	spin_unlock_irqrestore(&mm->shlock, flags);

	return ret;

VA_FAIL:
	if (malloc_flag == 1) {
		aipu_mm_free(mm, &buf.desc, NULL, true);
		htbuf_head->nums--;
		kmem_cache_free(mm->hold_tbuf_cache, htbuf);
	}
	return ret;
MALLOC_TCB_FAIL:
	kmem_cache_free(mm->hold_tbuf_cache, htbuf);
CACHE_FAIL:
	return ret;
}

void aipu_mm_hold_tcb_buf_free(struct aipu_memory_manager *mm)
{
	struct aipu_hold_tcb_buf *htbuf_head = mm->hold_tcb_head;
	struct aipu_hold_tcb_buf *htbuf = NULL;
	struct aipu_hold_tcb_buf *next = NULL;
	struct aipu_buf_desc buf;

	if (!htbuf_head)
		return;

	list_for_each_entry_safe(htbuf, next, &htbuf_head->node, node) {
		dev_info(mm->dev, "t %d i %d s %d f %d ch 0x%llx pht 0x%llx pt 0x%llx ph 0x%llx nh 0x%llx\n",
			 htbuf_head->nums, htbuf->index,  htbuf->status,
			 htbuf_head->hold_index, htbuf->head, htbuf->prev_hold_tcb,
			 htbuf->prev_tail, htbuf->prev_head, htbuf->next_head);
		memset(&buf, 0, sizeof(buf));
		buf.pa = htbuf->head;
		buf.region = AIPU_BUF_REGION_DEFAULT;
		buf.asid = AIPU_BUF_ASID_0;
		aipu_mm_free(mm, &buf, NULL, true);
		list_del(&htbuf->node);
		kmem_cache_free(mm->hold_tbuf_cache, htbuf);
		htbuf_head->nums--;
	}
	htbuf_head->hold_index = -1;
	htbuf_head->hold_tcb = NULL;
}

struct aipu_hold_tcb_buf *aipu_mm_get_hold_htbuf(struct aipu_memory_manager *mm, u64 hold_tcb_pa)
{
	struct aipu_hold_tcb_buf *htbuf_head = mm->hold_tcb_head;
	struct aipu_hold_tcb_buf *htbuf = NULL;
	struct aipu_hold_tcb_buf *ret = NULL;
	unsigned long flags = 0;

	spin_lock_irqsave(&mm->shlock, flags);
	list_for_each_entry(htbuf, &htbuf_head->node, node) {
		if (htbuf->head == hold_tcb_pa) {
			ret = htbuf;
			break;
		}
	}
	spin_unlock_irqrestore(&mm->shlock, flags);

	return ret;
}

void aipu_mm_set_final_htbuf_index(struct aipu_memory_manager *mm, int index)
{
	struct aipu_hold_tcb_buf *htbuf_head = mm->hold_tcb_head;
	unsigned long flags = 0;

	spin_lock_irqsave(&mm->shlock, flags);
	htbuf_head->hold_index = index;
	spin_unlock_irqrestore(&mm->shlock, flags);
}

static struct aipu_tcb_buf *create_tcb_buf(struct aipu_memory_manager *mm,
					   struct aipu_mem_region *reg)
{
	struct aipu_tcb_buf *tbuf = NULL;

	tbuf = kmem_cache_zalloc(mm->tbuf_cache, GFP_KERNEL);
	if (tbuf) {
		INIT_LIST_HEAD(&tbuf->node);
		tbuf->reg = reg;
	}
	return tbuf;
}

static void destroy_tcb_buf(struct aipu_memory_manager *mm, struct aipu_tcb_buf *tbuf)
{
	unsigned long flags = 0;

	if (tbuf) {
		spin_lock_irqsave(&mm->slock, flags);
		list_del(&tbuf->node);
		spin_unlock_irqrestore(&mm->slock, flags);
		kmem_cache_free(mm->tbuf_cache, tbuf);
	}
}

static struct aipu_mem_region *aipu_mm_find_region_no_lock(struct aipu_memory_manager *mm,
							   u64 iova, char *log_str)
{
	struct aipu_mem_region *reg = NULL;
	struct aipu_mem_region_obj *obj = NULL;

	list_for_each_entry(obj, &mm->mem.head->list, list) {
		reg = obj->reg;
		if (reg && iova >= reg->base_iova && (iova - reg->base_iova < reg->bytes))
			return reg;
	}

	dev_err(mm->dev, "[%s] region not found: invalid address 0x%llx",
		log_str ? log_str : "find_reg", iova);
	return NULL;
}

static struct aipu_mem_region *aipu_mm_find_region(struct aipu_memory_manager *mm,
						   u64 iova, char *log_str)
{
	struct aipu_mem_region *reg = NULL;

	mutex_lock(&mm->lock);
	reg = aipu_mm_find_region_no_lock(mm, iova, log_str);
	mutex_unlock(&mm->lock);
	return reg;
}

static struct aipu_tcb_buf *aipu_mm_find_tbuf_in_region_no_lock(struct aipu_memory_manager *mm,
								struct aipu_mem_region *reg,
								u64 iova)
{
	struct aipu_tcb_buf *curr = NULL;

	list_for_each_entry(curr, &reg->tcb_buf_head->node, node) {
		if (iova >= curr->head && iova <= curr->tail)
			return curr;
	}

	return NULL;
}

static struct aipu_tcb_buf *aipu_mm_find_tcb_buf_no_lock(struct aipu_memory_manager *mm, u64 iova,
							 char *log_str)
{
	struct aipu_mem_region *reg = NULL;
	struct aipu_tcb_buf *tbuf = NULL;

	if (!mm)
		return NULL;

	reg = aipu_mm_find_region_no_lock(mm, iova, "find_tcb");
	if (!reg)
		return NULL;

	tbuf = aipu_mm_find_tbuf_in_region_no_lock(mm, reg, iova);
	if (log_str && !tbuf)
		dev_dbg(mm->dev, "[%s] no TCB buffer is found at iova 0x%llx", log_str, iova);

	return tbuf;
}

static unsigned long pa_to_bitmap_no(struct aipu_memory_manager *mm, struct aipu_mem_region *reg,
				     u64 pa)
{
	return (pa - reg->base_iova) >> PAGE_SHIFT;
}

static int aipu_mm_init_pages(struct aipu_memory_manager *mm, struct aipu_mem_region *reg)
{
	if (!mm || !reg)
		return -EINVAL;

	reg->count = reg->bytes >> PAGE_SHIFT;
	reg->bitmap = devm_kzalloc(reg->dev,
				   BITS_TO_LONGS(reg->count) * sizeof(long), GFP_KERNEL);
	if (!reg->bitmap)
		return -ENOMEM;

	reg->pages = vzalloc(reg->count * sizeof(struct aipu_virt_page *));
	if (!reg->pages)
		return -ENOMEM;

	reg->base_pfn = PFN_DOWN(reg->base_iova);

	return 0;
}

static void aipu_mm_destroy_region(struct aipu_memory_manager *mm, struct aipu_mem_region *reg)
{
	struct aipu_tcb_buf *tbuf = NULL;
	struct aipu_tcb_buf *next = NULL;

	if (!mm || !reg)
		return;

	/* release allocation to system */
	if (reg->base_va) {
		dma_free_attrs(reg->dev, reg->bytes, reg->base_va, reg->base_pa, reg->attrs);
		reg->base_va = NULL;
		reg->base_iova = 0;
	}

	if (reg->reserved) {
		of_reserved_mem_device_release(reg->dev);
		reg->base_pa = 0;
	}

	if (reg->pages) {
		vfree(reg->pages);
		reg->pages = NULL;
		reg->count = 0;
	}

	if (reg->bitmap) {
		devm_kfree(reg->dev, reg->bitmap);
		reg->bitmap = NULL;
	}

	if (reg->tcb_buf_head) {
		list_for_each_entry_safe(tbuf, next, &reg->tcb_buf_head->node, node)
			destroy_tcb_buf(mm, tbuf);

		destroy_tcb_buf(mm, reg->tcb_buf_head);
		reg->tcb_buf_head = NULL;
	}

	reg->bytes = 0;
	kmem_cache_free(mm->reg_cache, reg);
}

static struct aipu_mem_region *aipu_mm_create_region(struct aipu_memory_manager *mm,
						     struct aipu_mem_region_obj *obj,
						     enum aipu_mem_region_type type,
						     u64 size, u64 offset, u32 idx,
						     bool reserved)
{
	int ret = 0;
	void *va = NULL;
	struct aipu_mem_region *reg = NULL;

	if (!mm || !obj)
		return NULL;

	reg = kmem_cache_zalloc(mm->reg_cache, GFP_KERNEL);
	if (!reg)
		return NULL;

	reg->type = type;
	reg->bytes = size;
	reg->host_aipu_offset = offset;
	reg->reserved = reserved;
	reg->obj = obj;

	/* we use child dev to support multiple reserved regions */
	if (reserved)
		reg->dev = aipu_mm_create_child_dev(mm->dev, idx);
	else
		reg->dev = mm->dev;

	if (!reg->dev)
		goto err;

	/* only head of the list is created; for v3; */
	reg->tcb_buf_head = create_tcb_buf(mm, reg);
	if (!reg->tcb_buf_head)
		goto err;

	/* create a list head, not a real region */
	if (!size)
		return reg;

	if (reserved) {
		ret = of_reserved_mem_device_init_by_idx(reg->dev, mm->dev->of_node, idx);
		if (ret) {
			dev_err(reg->dev, "init reserved mem failed: idx %d (%d)\n",
				idx, ret);
			goto err;
		}
	}

	if(mm->has_iommu) {
		ret = dma_set_mask_and_coherent(reg->dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_err(reg->dev, "DMA set coherent mask failed: idx %d (%d)!\n", idx, ret);
			goto err;
		}
		// limit npu dma address within 3G
		reg->dev->bus_dma_limit = 0xc0000000;
	}

	va = dma_alloc_attrs(reg->dev, reg->bytes, &reg->base_pa, GFP_KERNEL, reg->attrs);
	if (!va) {
		dev_err(reg->dev, "dma_alloc_attrs failed: idx %d (bytes: 0x%llx, attrs %ld)\n",
			idx, reg->bytes, reg->attrs);
		goto err;
	}
	reg->base_iova = reg->base_pa - reg->host_aipu_offset;
	reg->base_va = va;

	/* reserve memory region use page structs to maintain the allocations in this reg */
	if (reserved) {
		ret = aipu_mm_init_pages(mm, reg);
		if (ret)
			goto err;

		dev_info(reg->dev, "init %s region done: %s [0x%llx, 0x%llx]\n",
			 type == AIPU_MEM_REGION_TYPE_MEMORY ? "MEMORY" :
			 (type == AIPU_MEM_REGION_TYPE_SRAM ? "SRAM" :
			  (type == AIPU_MEM_REGION_TYPE_DTCM ? "DTCM" : "GM")),
			 mm->has_iommu ? "iova" : "pa",
			 reg->base_iova, reg->base_iova + reg->bytes - 1);
	}

	/* works for non-reserved regions only */
	reg->locked = true;
	return reg;

err:
	aipu_mm_destroy_region(mm, reg);
	return NULL;
}

static struct aipu_mem_region_obj *aipu_mm_create_region_object(struct aipu_memory_manager *mm,
								enum aipu_mem_region_type type,
								u64 size, u64 offset, u32 idx,
								struct file *filp, bool reserved)
{
	struct aipu_mem_region_obj *obj = NULL;

	obj = kmem_cache_zalloc(mm->obj_cache, GFP_KERNEL);
	if (!obj)
		return NULL;

	obj->reg = aipu_mm_create_region(mm, obj, type, size, offset, idx, reserved);
	if (!obj->reg) {
		kmem_cache_free(mm->obj_cache, obj);
		return NULL;
	}

	obj->reg->filp = filp;
	INIT_LIST_HEAD(&obj->list);

	return obj;
}

static void aipu_mm_destroy_region_object(struct aipu_memory_manager *mm,
					  struct aipu_mem_region_obj *obj)
{
	if (obj) {
		aipu_mm_destroy_region(mm, obj->reg);
		obj->reg = NULL;
		list_del(&obj->list);
		kmem_cache_free(mm->obj_cache, obj);
	}
}

static int init_region_list(struct aipu_memory_manager *mm, struct aipu_mem_region_list *list)
{
	struct aipu_mem_region_obj *obj = NULL;

	obj = aipu_mm_create_region_object(mm, AIPU_MEM_REGION_TYPE_MEMORY, 0, 0, 0, NULL, false);
	if (!obj)
		return -ENOMEM;

	list->head = obj;
	list->cnt = 0;
	list->valid_cnt = 0;
	list->base = U64_MAX;
	list->range = 0;
	return 0;
}

static int get_free_bitmap_no(struct aipu_memory_manager *mm, struct aipu_mem_region *reg,
			      struct aipu_buf_request *buf_req)
{
	unsigned long alloc_nr = ALIGN(buf_req->bytes, PAGE_SIZE) >> PAGE_SHIFT;
	unsigned long align_order = order_base_2(buf_req->align_in_page);
	unsigned long mask = (1UL << align_order) - 1;
	unsigned long offset = reg->base_pfn & ((1UL << align_order) - 1);

	return bitmap_find_next_zero_area_off(reg->bitmap, reg->count, 0, alloc_nr, mask, offset);
}

static int aipu_mm_alloc_in_region_no_lock(struct aipu_memory_manager *mm,
					   struct aipu_buf_request *buf_req,
					   struct aipu_mem_region *reg, struct file *filp,
					   struct aipu_tcb_buf **tbuf_alloc)
{
	unsigned long bitmap_no = 0;
	unsigned long alloc_nr = 0;
	struct aipu_tcb_buf *tbuf = NULL;
	u64 dev_pa = 0;
	u32 dev_size = 0;

	/* this allocator is called in both reserved & smmu cases */

	/* filp == NULL means GM initialization or exit TCB allocation */
	if (!mm || !buf_req || !reg || reg->invalid || !tbuf_alloc)
		return -EINVAL;

	if (buf_req->data_type == AIPU_MM_DATA_TYPE_TCB) {
		tbuf = create_tcb_buf(mm, reg);
		if (!tbuf)
			return -ENOMEM;
	}

	alloc_nr = ALIGN(buf_req->bytes, PAGE_SIZE) >> PAGE_SHIFT;
	dev_size = alloc_nr * PAGE_SIZE;
	if (reg->reserved) {
		bitmap_no = get_free_bitmap_no(mm, reg, buf_req);
		if (bitmap_no >= reg->count) {
			dev_dbg(reg->dev, "alloc in region failed: no free buffer");
			goto fail;
		}

		if (!reg->pages[bitmap_no]) {
			reg->pages[bitmap_no] =
				devm_kzalloc(reg->dev, sizeof(struct aipu_virt_page), GFP_KERNEL);
			if (!reg->pages[bitmap_no])
				goto fail;
		}

		/* success */
		reg->pages[bitmap_no]->contiguous_alloc_len = alloc_nr;
		reg->pages[bitmap_no]->filp = filp;
		reg->pages[bitmap_no]->tid = task_pid_nr(current);
		reg->pages[bitmap_no]->locked = true;
		reg->pages[bitmap_no]->tcb = NULL;
		bitmap_set(reg->bitmap, bitmap_no, alloc_nr);

		dev_pa = reg->base_iova + (bitmap_no << PAGE_SHIFT);
	} else {
		dev_pa = reg->base_iova;
	}

	if (tbuf) {
		if (reg->reserved) {
			reg->pages[bitmap_no]->tcb = tbuf;
			tbuf->pfn = bitmap_no;
		}

		/**
		 * when head/tail is updated in job manager, all the following fields
		 * should be updated together.
		 *
		 * NOTE: all the nodes of the TCB chain scheduled in a job should be within
		 * this buf!
		 */
		tbuf->head = dev_pa;
		tbuf->tail = tbuf->head + dev_size - sizeof(struct aipu_tcb);
		tbuf->tail_tcb = NULL;
		tbuf->dep_job_id = 0;
		tbuf->pinned = false;
		*tbuf_alloc = tbuf;
	}

	buf_req->desc.dev_offset = dev_pa;
	buf_req->desc.pa = dev_pa;
	buf_req->desc.bytes = dev_size;
	buf_req->desc.asid = buf_req->asid;
	buf_req->desc.region = reg->type;

	dev_dbg(reg->dev,
		"allocation in region done: asid %d iova 0x%llx, bytes 0x%llx, type %d\n", buf_req->asid,
		buf_req->desc.pa, buf_req->desc.bytes, buf_req->data_type);
	return 0;

fail:
	if (tbuf)
		destroy_tcb_buf(mm, tbuf);
	return -ENOMEM;
}

static int aipu_mm_free_in_region(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf,
				  struct aipu_mem_region *reg, bool unlock)
{
	unsigned long bitmap_no = 0;
	unsigned long alloc_nr = 0;
	struct aipu_virt_page *page = NULL;
	struct aipu_tcb_buf *tbuf = NULL;

	if (!mm || !buf || !reg || reg->invalid)
		return -EINVAL;

	/* this __free__ function is only called for reserved cases */
	bitmap_no = pa_to_bitmap_no(mm, reg, buf->pa);
	if (bitmap_no >= reg->count) {
		dev_err(reg->dev,
			"free in region failed: no such an allocated buffer: pa 0x%llx",
			buf->pa);
		return -EINVAL;
	}

	page = reg->pages[bitmap_no];
	if (!page) {
		dev_err(reg->dev, "free in region failed: null page at bitmap_no %lu",
			bitmap_no);
		return -EINVAL;
	}

	alloc_nr = page->contiguous_alloc_len;
	if (!alloc_nr) {
		dev_err(reg->dev, "free in region failed: zero alloc_nr is invalid, 0x%llx\n",
			buf->pa);
		return -EINVAL;
	}

	/* do not update the __locked__ flag if unlock == false */
	if (unlock)
		page->locked = false;
	else if (page->locked)
		return DEFERRED_FREE;

	tbuf = page->tcb;

	/* do not free if this tcb is linked by a job or pinned;
	 * marked as unlocked and will be released at an appropriate point;
	 */
	if (tbuf && (tbuf->dep_job_id || tbuf->pinned)) {
		dev_info(reg->dev, "deferred free TCB: iova 0x%llx dep_job_id 0x%x ping %d\n",
			 buf->pa, tbuf->dep_job_id, tbuf->pinned);
		return DEFERRED_FREE;
	}

	/* do free */
	destroy_tcb_buf(mm, tbuf);
	bitmap_clear(reg->bitmap, bitmap_no, alloc_nr);
	memset(page, 0, sizeof(struct aipu_virt_page));

	dev_dbg(reg->dev, "free in region done: iova 0x%llx, bytes 0x%llx\n", buf->pa, buf->bytes);

	return 0;
}

static void aipu_mm_free_filp_in_region(struct aipu_memory_manager *mm,
					struct aipu_mem_region *reg, struct file *filp)
{
	unsigned long i = 0;
	unsigned long offset = 0;
	struct aipu_tcb_buf *tbuf = NULL;

	if (!mm || !reg || !reg->bitmap || !filp)
		return;

	mutex_lock(&mm->lock);
	while ((i = find_next_bit(reg->bitmap, reg->count, offset)) != reg->count) {
		offset = i + reg->pages[i]->contiguous_alloc_len;
		if (reg->pages[i] && reg->pages[i]->filp == filp) {
			reg->pages[i]->locked = false;
			tbuf = reg->pages[i]->tcb;
			if (tbuf && (tbuf->dep_job_id || tbuf->pinned))
				continue;

			/* do free */
			bitmap_clear(reg->bitmap, i, reg->pages[i]->contiguous_alloc_len);
			memset(reg->pages[i], 0, sizeof(struct aipu_virt_page));
			destroy_tcb_buf(mm, tbuf);
		}
	}
	mutex_unlock(&mm->lock);
}

static ssize_t aipu_gm_policy_sysfs_show(struct device *dev, struct device_attribute *attr,
					 char *buf)
{
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_priv *aipu = platform_get_drvdata(p_dev);
	struct aipu_partition *partition = aipu->partitions;
	struct aipu_memory_manager *mm = &partition->priv->mm;

	if (mm->gm_policy == AIPU_GM_POLICY_SHARED) {
		return snprintf(buf, 128, "[%d] AIPU GM is shared by tasks of all QoS level.\n",
				AIPU_GM_POLICY_SHARED);
	} else if (mm->gm_policy == AIPU_GM_POLICY_HALF_DIVIDED) {
		return snprintf(buf, 128,
				"[%d] GM is divided half-by-half for QoS slow & fast tasks.\n",
				AIPU_GM_POLICY_HALF_DIVIDED);
	}

	return snprintf(buf, 128, "[%d] AIPU has no GM (or GM is disabled).\n",
			AIPU_GM_POLICY_NONE);
}

static ssize_t aipu_gm_policy_sysfs_store(struct device *dev, struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct platform_device *p_dev = container_of(dev, struct platform_device, dev);
	struct aipu_priv *aipu = platform_get_drvdata(p_dev);
	struct aipu_partition *partition = aipu->partitions;
	struct aipu_memory_manager *mm = &partition->priv->mm;

	mutex_lock(&mm->lock);
	if ((strncmp(buf, "0", 1) == 0))
		mm->gm_policy = AIPU_GM_POLICY_NONE;
	else if ((strncmp(buf, "1", 1) == 0))
		mm->gm_policy = AIPU_GM_POLICY_SHARED;
	else if ((strncmp(buf, "2", 1) == 0))
		mm->gm_policy = AIPU_GM_POLICY_HALF_DIVIDED;
	else
		dev_err(mm->dev, "[sysfs] invalid GM policy: gm_policy should be 0/1/2");
	mutex_unlock(&mm->lock);

	return count;
}

static void add_region_list(struct aipu_memory_manager *mm, int asid,
			    struct aipu_mem_region *reg)
{
	struct aipu_mem_region_obj *obj = kmem_cache_zalloc(mm->obj_cache, GFP_KERNEL);

	obj->reg = reg;
	INIT_LIST_HEAD(&obj->list);

	list_add(&obj->list, &mm->ase[asid].head->list);
	mm->ase[asid].cnt++;
}

static int aipu_mm_add_reserved_regions(struct aipu_memory_manager *mm)
{
	int ret = 0;
	int idx = 0;
	int asid_idx = 0;
	struct device_node *np = NULL;
	enum aipu_mem_region_type type;
	struct resource res;
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region *reg = NULL;
	bool asid_set = false;
	u32 dtcm_cnt = 0;
	u64 offset = 0;
	u32 res_cnt = 0;

	do {
		u64 size = 0;

		np = of_parse_phandle(mm->dev->of_node, "memory-region", idx);
		if (!np)
			break;

		/* get type of this region */
		if (!strcmp(np->name, "memory")) {
			type = AIPU_MEM_REGION_TYPE_MEMORY;
		} else if (!strcmp(np->name, "sram")) {
			type = AIPU_MEM_REGION_TYPE_SRAM;
		} else if (!strcmp(np->name, "dtcm")) {
			type = AIPU_MEM_REGION_TYPE_DTCM;
		} else {
			dev_err(mm->dev, "invalid memory region name: %s\n", np->name);
			continue;
		}

		/* bypass mapping SoC SRAM if CPU cannot access it */
		if (type == AIPU_MEM_REGION_TYPE_SRAM && !AIPU_CONFIG_HOST_MAP_SRAM) {
			dev_info(mm->dev, "dts: bypass this sram region (cpu cannot access it)\n");
			continue;
		}

		if (type == AIPU_MEM_REGION_TYPE_DTCM && dtcm_cnt == mm->dtcm_max_cnt) {
			dev_err(mm->dev, "dts: bypass this dtcm region for the max_cnt limit\n");
			continue;
		}

		if (type == AIPU_MEM_REGION_TYPE_DTCM &&
		    mm->version == AIPU_ISA_VERSION_ZHOUYI_V1) {
			dev_err(mm->dev, "dts: bypass because v1 does not support dtcm\n");
			continue;
		}

		ret = of_address_to_resource(np, 0, &res);
		if (ret) {
			of_node_put(np);
			dev_err(mm->dev, "of_address_to_resource failed (ret = %d)", ret);
			continue;
		}

		size = res.end - res.start + 1;
		if (type == AIPU_MEM_REGION_TYPE_DTCM && size > ZHOUYI_V2_DTCM_MAX_BYTES) {
			dev_info(mm->dev,
				 "the DTCM will be clipped to the maximum configurable value\n");
			size = ZHOUYI_V2_DTCM_MAX_BYTES;
		}

		if (of_property_read_u64(np, "host-aipu-offset", &offset))
			offset = 0;

		obj = aipu_mm_create_region_object(mm, type, size, offset, idx, NULL, true);
		if (!obj) {
			dev_err(mm->dev, "create new region failed (ret = %ld)", PTR_ERR(reg));
			continue;
		}

		reg = obj->reg;
		list_add(&obj->list, &mm->mem.head->list);

		/* get asid of this region */
		if (mm->version > AIPU_ISA_VERSION_ZHOUYI_V1) {
			for (asid_idx = 0; asid_idx < ZHOUYI_ASID_COUNT; asid_idx++) {
				u32 asid = 0;

				if (!of_property_read_u32_index(np, "asid", asid_idx, &asid)) {
					if (asid >= ZHOUYI_ASID_COUNT) {
						dev_err(mm->dev, "dts: invalid asid: %u\n", asid);
						continue;
					}

					add_region_list(mm, asid, reg);
					if (asid >= mm->valid_asid_cnt)
						mm->valid_asid_cnt = asid + 1;

					asid_set = true;
				} else {
					break;
				}
			}

			if (!asid_set) {
				dev_err(mm->dev, "dts: reg asid is not set, use default asid\n");
				mm->valid_asid_cnt = 2;
				add_region_list(mm, AIPU_BUF_ASID_0, reg);
				add_region_list(mm, AIPU_BUF_ASID_1, reg);
			}
		} else {
			/* use ASID 0 for V1 */
			add_region_list(mm, AIPU_BUF_ASID_0, reg);
		}

		of_node_put(np);
		if (type == AIPU_MEM_REGION_TYPE_DTCM)
			dtcm_cnt++;
		res_cnt++;
	} while (++idx < AIPU_CONFIG_MAX_RESERVED_REGIONS);

	return res_cnt;
}

static void aipu_mm_set_asid_base(struct aipu_memory_manager *mm)
{
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region *reg = NULL;
	int asid = AIPU_BUF_ASID_0;

	for (asid = AIPU_BUF_ASID_0; asid < ZHOUYI_ASID_COUNT; asid++) {
		u64 first_base = 0;
		u64 asid_base = 0;
		u64 range = 0;
		u64 high = 0;
		u64 end  = 0;

		list_for_each_entry(obj, &mm->ase[asid].head->list, list) {
			reg = obj->reg;
			end = reg->base_iova + reg->bytes;

			if (!range) {
				high = end;
				first_base = reg->base_iova;
				asid_base = first_base;
				range = reg->bytes;
			}

			if (reg->base_iova >> 32 != first_base >> 32) {
				dev_err(mm->dev, "invalid region: [0x%llx, 0x%llx]",
					reg->base_iova, reg->base_iova + reg->bytes - 1);
				dev_err(mm->dev, "one asid should use the same hi32 base");
				reg->invalid = true;
				continue;
			}

			high = end > high ? end : high;

			if (reg->base_iova != first_base) {
				asid_base = ((first_base >> 32) << 32);
				range = high;
			}

			mm->ase[asid].valid_cnt++;
		}

		if (range) {
			mm->ase[asid].base = asid_base;
			mm->ase[asid].range = range;
			dev_info(mm->dev, "set ASID %d done: base: 0x%llx, range 0x%llx\n",
				 asid, asid_base, range);
		}
	}
}

/**
 * @aipu_init_mm() - initialize mm module during driver probe phase
 * @mm:      pointer to memory manager struct to be initialized
 * @p_dev:   pointer to the platform device struct
 * @version: AIPU ISA version
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_init_mm(struct aipu_memory_manager *mm, struct platform_device *p_dev, int version)
{
	int ret = 0;
	struct iommu_group *group = NULL;
	int asid = 0;

	if (!mm || !p_dev)
		return -EINVAL;

	memset(mm, 0, sizeof(*mm));
	mm->version = version;
	mm->dev = &p_dev->dev;
	mutex_init(&mm->lock);
	mm->sram_disable_head = devm_kzalloc(mm->dev, sizeof(*mm->sram_disable_head), GFP_KERNEL);
	if (!mm->sram_disable_head)
		return -ENOMEM;
	INIT_LIST_HEAD(&mm->sram_disable_head->list);
	mm->importer_bufs = devm_kzalloc(mm->dev, sizeof(*mm->importer_bufs), GFP_KERNEL);
	if (!mm->importer_bufs)
		return -ENOMEM;
	INIT_LIST_HEAD(&mm->importer_bufs->node);
	spin_lock_init(&mm->slock);
	spin_lock_init(&mm->shlock);
	mm->default_asid_base = 0;
	mm->default_asid_size = 0xC0000000;
	mm->valid_asid_cnt = 0;

	mm->obj_cache = kmem_cache_create("aipu_obj_cache", sizeof(struct aipu_mem_region_obj),
					  0, SLAB_PANIC, NULL);
	mm->reg_cache = kmem_cache_create("aipu_reg_cache", sizeof(struct aipu_mem_region),
					  0, SLAB_PANIC, NULL);
	mm->tbuf_cache = kmem_cache_create("aipu_tbuf_cache", sizeof(struct aipu_tcb_buf),
					   0, SLAB_PANIC, NULL);
	mm->hold_tbuf_cache = kmem_cache_create("aipu_hold_tbuf_cache",
						sizeof(struct aipu_hold_tcb_buf),
						0, SLAB_PANIC, NULL);
	if (!mm->obj_cache || !mm->reg_cache || !mm->tbuf_cache || !mm->hold_tbuf_cache)
		return -ENOMEM;

	for (asid = AIPU_BUF_ASID_0; asid < ZHOUYI_ASID_COUNT; asid++) {
		ret = init_region_list(mm, &mm->ase[asid]);
		if (ret)
			goto finish;
	}

	ret = init_region_list(mm, &mm->mem);
	if (ret)
		goto finish;

	/* we accept at maximum 2 GM regions: for QoS fast & slow */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3)
		mm->gm_max_cnt = 2;

	/* currently, we only support one DTCM region in v2_2 */
	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V2_2)
		mm->dtcm_max_cnt = 1;

	if (version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		mm->hold_tcb_head = kmem_cache_zalloc(mm->hold_tbuf_cache, GFP_KERNEL);
		if (!mm->hold_tcb_head)
			goto finish;
		INIT_LIST_HEAD(&mm->hold_tcb_head->node);
		mm->hold_tcb_head->nums = 0;
	}
	/**
	 * Device tree binding for Zhouyi V3:
	 *
	 *    gm-policy = <POLICY_NO>;
	 *
	 * where POLICY_NO should be one of the following values:
	 * 0: no GM;
	 * 1: GM is shared by all tasks in 1 cluster (by default if this attribute is not provided)
	 * 2: GM is divided half-by-half: for QoS slow & fast tasks, respectively
	 */
	if (version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		ret = device_property_read_u32(mm->dev, "gm-policy", &mm->gm_policy);
		if (ret || mm->gm_policy > AIPU_GM_POLICY_HALF_DIVIDED)
			mm->gm_policy = AIPU_GM_POLICY_SHARED;

		if (IS_ERR(aipu_common_create_attr(mm->dev, &mm->gm_policy_attr, "gm_policy", 0644,
						   aipu_gm_policy_sysfs_show,
						   aipu_gm_policy_sysfs_store))) {
			mm->gm_policy_attr = NULL;
			dev_err(mm->dev, "create gm_policy attr failed");
		}

		dev_info(mm->dev, "GM policy is %s",
			 mm->gm_policy == AIPU_GM_POLICY_SHARED ? "shared" : "half-by-half");
	} else {
		mm->gm_policy = AIPU_GM_POLICY_NONE;
		mm->gm_policy_attr = NULL;
	}

	group = iommu_group_get(mm->dev);
	if (group) {
		mm->valid_asid_cnt = ZHOUYI_ASID_COUNT;
		mm->has_iommu = true;
	}
	iommu_group_put(group);
	dev_info(mm->dev, "AIPU is%s behind an IOMMU\n", mm->has_iommu ? "" : " not");

	/*
	 * If AIPU is behind an IOMMU, in devicetree, memory-region attribute is optional;
	 * otherwise the device specific or system global DMA/CMA region is used;
	 *
	 * KMD can accept multiple DRAM regions and/or multiple SRAM regions;
	 */
	if (!mm->has_iommu)
		mm->res_cnt = aipu_mm_add_reserved_regions(mm);

	if (version > AIPU_ISA_VERSION_ZHOUYI_V1 && mm->res_cnt)
		aipu_mm_set_asid_base(mm);

	dev_info(mm->dev, "driver mem management is %s\n", mm->res_cnt ? "enabled" : "disabled");

finish:
	if (ret)
		aipu_deinit_mm(mm);
	return ret;
}

/**
 * @aipu_deinit_mm() - de-initialize mm module while kernel module unloading
 * @mm: pointer to memory manager struct initialized in aipu_init_mm()
 */
int aipu_deinit_mm(struct aipu_memory_manager *mm)
{
	int id = AIPU_BUF_ASID_0;
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region_obj *next = NULL;

	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V3) {
		if (mm->gm_policy_attr) {
			aipu_common_destroy_attr(mm->dev, &mm->gm_policy_attr);
			mm->gm_policy_attr = NULL;
		}
		if (mm->hold_tbuf_cache) {
			aipu_mm_hold_tcb_buf_free(mm);
			kmem_cache_free(mm->hold_tbuf_cache, mm->hold_tcb_head);
			mm->hold_tcb_head = NULL;
			kmem_cache_destroy(mm->hold_tbuf_cache);
			mm->hold_tbuf_cache = NULL;
		}
	}
	if (mm->mem.head) {
		list_for_each_entry_safe(obj, next, &mm->mem.head->list, list)
			aipu_mm_destroy_region_object(mm, obj);

		aipu_mm_destroy_region_object(mm, mm->mem.head);
		mm->mem.head = NULL;
	}

	for (id = AIPU_BUF_ASID_0; id < ZHOUYI_ASID_COUNT; id++) {
		if (mm->ase[id].head) {
			list_for_each_entry_safe(obj, next, &mm->ase[id].head->list, list) {
				list_del(&obj->list);
				kmem_cache_free(mm->obj_cache, obj);
				/* reg has been freed when destroying mm->mem */
			}

			kmem_cache_free(mm->reg_cache, mm->ase[id].head->reg);
			kmem_cache_free(mm->obj_cache, mm->ase[id].head);
			mm->ase[id].head = NULL;
		}
	}

	mm->res_cnt = 0;
	kmem_cache_destroy(mm->obj_cache);
	mm->obj_cache = NULL;
	kmem_cache_destroy(mm->reg_cache);
	mm->reg_cache = NULL;
	kmem_cache_destroy(mm->tbuf_cache);
	mm->tbuf_cache = NULL;
	return 0;
}

static int aipu_mm_direct_alloc(struct aipu_memory_manager *mm, struct aipu_buf_request *buf_req,
				struct file *filp, struct aipu_tcb_buf **tcb_alloc)
{
	int ret = 0;
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_tcb_buf *tcb = NULL;

	obj = aipu_mm_create_region_object(mm, AIPU_MEM_REGION_TYPE_MEMORY, buf_req->bytes, 0, 0,
					   filp, false);
	if (!obj || !obj->reg)
		return -ENOMEM;

	ret = aipu_mm_alloc_in_region_no_lock(mm, buf_req, obj->reg, filp, &tcb);
	if (ret) {
		ret = -ENOMEM;
		goto err;
	}

	mutex_lock(&mm->lock);
	list_add(&obj->list, &mm->mem.head->list);
	mutex_unlock(&mm->lock);

	if (tcb_alloc)
		*tcb_alloc = tcb;
	return 0;

err:
	aipu_mm_destroy_region_object(mm, obj);
	return ret;
}

/**
 * @aipu_mm_alloc() - alloc memory buffer for user request
 * @mm:      pointer to memory manager struct initialized in aipu_init_mm()
 * @buf_req: pointer to buffer request struct from userland
 * @filp:    pointer to the file struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_alloc(struct aipu_memory_manager *mm, struct aipu_buf_request *buf_req,
		  struct file *filp)
{
	int ret = 0;
	struct aipu_mem_region_obj *obj = NULL;
	int type;
	unsigned long flags;
	struct aipu_tcb_buf *tbuf = NULL;
	bool allocated = false;
	bool fall_back = false;

	if (!mm || !buf_req)
		return -EINVAL;

	if (!buf_req->bytes || !is_power_of_2(buf_req->align_in_page) ||
	    buf_req->asid >= ZHOUYI_ASID_COUNT) {
		dev_err(mm->dev, "[malloc] invalid alloc request: bytes 0x%llx, align %u, asid %u",
			buf_req->bytes, buf_req->align_in_page, buf_req->asid);
		return -EINVAL;
	}

	if (mm->version == AIPU_ISA_VERSION_ZHOUYI_V1)
		buf_req->asid = AIPU_BUF_ASID_0;

	if (buf_req->region < AIPU_MEM_REGION_TYPE_MAX) {
		type = buf_req->region;
	} else {
		dev_err(mm->dev, "[malloc] invalid alloc request: region type %d",
			buf_req->region);
		return -EINVAL;
	}

	/* fall back to SRAM if DTCM/GM is not applicable for certain archs */
	if (mm->version < AIPU_ISA_VERSION_ZHOUYI_V2_2 && type == AIPU_MEM_REGION_TYPE_DTCM)
		type = AIPU_MEM_REGION_TYPE_SRAM;

	if ((mm->version < AIPU_ISA_VERSION_ZHOUYI_V3 || mm->gm_policy == AIPU_GM_POLICY_NONE) &&
	    type == AIPU_MEM_REGION_TYPE_GM)
		type = AIPU_MEM_REGION_TYPE_SRAM;

	if (!mm->res_cnt) {
		ret = aipu_mm_direct_alloc(mm, buf_req, filp, &tbuf);
		allocated = !ret;
		goto tcb_handle;
	}

	mutex_lock(&mm->lock);

	/* should check after lock */
	if (type == AIPU_MEM_REGION_TYPE_SRAM && mm->sram_disable)
		type = AIPU_MEM_REGION_TYPE_MEMORY;

alloc:
	list_for_each_entry(obj, &mm->ase[buf_req->asid].head->list, list) {
		if (obj->reg->type == type) {
			ret = aipu_mm_alloc_in_region_no_lock(mm, buf_req, obj->reg, filp, &tbuf);
			if (!ret) {
				allocated = true;
				break;
			}
		}
	}

	/* fall back to main memory */
	if (!allocated && !fall_back) {
		if (type != AIPU_MEM_REGION_TYPE_MEMORY) {
		type = AIPU_MEM_REGION_TYPE_MEMORY;
		fall_back = true;
		goto alloc;
		} else if (buf_req->asid > AIPU_BUF_ASID_0){
			dev_dbg(mm->dev, "fail to malloc memory size 0x%llx from asid %d ",
				buf_req->bytes, buf_req->asid);
			buf_req->asid++;
			dev_dbg(mm->dev, "change new asid %d\n", buf_req->asid);
			if (buf_req->asid < mm->valid_asid_cnt)
				goto alloc;
			else
				dev_err(mm->dev, "Inalid asid parameter.\n");
		}
	}

	WARN_ON(buf_req->desc.pa % (buf_req->align_in_page << PAGE_SHIFT));
	mutex_unlock(&mm->lock);

tcb_handle:
	if (tbuf) {
		spin_lock_irqsave(&mm->slock, flags);
		list_add_tail(&tbuf->node, &tbuf->reg->tcb_buf_head->node);
		spin_unlock_irqrestore(&mm->slock, flags);
	}

	if (!allocated) {
		dev_err(mm->dev,
			"failed in allocating for: bytes 0x%llx, page align %d, type %d\n",
			buf_req->bytes, buf_req->align_in_page, buf_req->data_type);
	} else {
		dev_dbg(mm->dev,
			"allocate done (%s): asid %d iova 0x%llx, type %d, bytes 0x%llx (al %d, rg %d)\n",
			fall_back ? "fall back to memory" : "as requested", buf_req->asid,
			buf_req->desc.pa, buf_req->data_type, buf_req->bytes,
			buf_req->align_in_page, buf_req->desc.region);
	}
	return ret;
}

static int aipu_mm_direct_free(struct aipu_memory_manager *mm, struct aipu_mem_region *reg,
			       bool unlock)
{
	struct aipu_tcb_buf *tbuf = NULL;
	u64 iova = 0;

	if (!mm || !reg)
		return -EINVAL;

	if (unlock)
		reg->locked = false;
	else if (reg->locked)
		return DEFERRED_FREE;

	iova = reg->base_iova;

	/* do not return tbuf for deferred free cases */
	tbuf = aipu_mm_find_tbuf_in_region_no_lock(mm, reg, iova);
	if (tbuf && (tbuf->dep_job_id || tbuf->pinned)) {
		dev_info(reg->dev, "deferred free (direct) TCB: iova 0x%llx\n", iova);
		return DEFERRED_FREE;
	}

	/* do free */
	aipu_mm_destroy_region_object(mm, reg->obj);

	dev_dbg(mm->dev, "direct free done: iova 0x%llx\n", iova);
	return 0;
}

/**
 * @aipu_mm_free() - free buffer allocated by aipu_mm_alloc()
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @buf:  pointer to the buffer descriptor to be released
 * @filp: pointer to the file struct
 * @unlock: unlock the page or not
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_free(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf, struct file *filp,
		 bool unlock)
{
	int ret = 0;
	struct aipu_mem_region *reg = NULL;

	if (!mm || !buf)
		return -EINVAL;

	reg = aipu_mm_find_region(mm, buf->pa, "free");
	if (!reg)
		return -EINVAL;

	mutex_lock(&mm->lock);
	if (reg->reserved)
		ret = aipu_mm_free_in_region(mm, buf, reg, unlock);
	else
		ret = aipu_mm_direct_free(mm, reg, unlock);
	mutex_unlock(&mm->lock);

	if (ret == DEFERRED_FREE)
		ret = 0;

	return ret;
}

void tensor_dcache_inval_poc(void *start, void *end) {
    uintptr_t line_size, addr_start, addr_end, tmp;

    line_size = cache_line_size();
    tmp = line_size - 1;

    addr_start = (uintptr_t)start;
    addr_end = (uintptr_t)end;

    if (addr_end & tmp) {
        addr_end &= ~tmp;
        __asm__ __volatile__("dc civac, %0" : : "r" (addr_end) : "memory");
    }

    if (addr_start & tmp) {
        addr_start &= ~tmp;
        __asm__ __volatile__("dc civac, %0" : : "r" (addr_start) : "memory");
        addr_start += line_size;
    }

    while (addr_start < addr_end) {
        __asm__ __volatile__("dc ivac, %0" : : "r" (addr_start) : "memory");
        addr_start += line_size;
    }

    __asm__ __volatile__("dsb sy" : : : "memory");
}

static void tensor_flush_dcache_range(void *start, void *end) {
	uintptr_t addr_start, addr_end;
	const uint64_t line_size = cache_line_size();

	addr_start = (uintptr_t)start;
	addr_end = (uintptr_t)end;

	// make sure addr_start and addr_end are cache line aligned
	addr_start = addr_start & ~(line_size - 1);
	addr_end = (addr_end + line_size - 1) & ~(line_size - 1);

	while (addr_start < addr_end) {
		__asm__ volatile ("dc cvac, %0" : : "r" (addr_start) : "memory");
        addr_start += line_size;
	}

	// Ensure all cache operations complete
	__asm__ volatile ("dsb sy" : : : "memory");

}

/**
 * @aipu_mm_cache_flush() - invald buffer allocated by aipu_mm_alloc()
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @buf:  pointer to the buffer descriptor to be invalid
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_cache_flush(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf)
{
	int ret = 0;
	struct aipu_mem_region *reg = NULL;

	if (!mm || !buf)
		return -EINVAL;

	reg = aipu_mm_find_region(mm, buf->pa, "flush");
	if (!reg)
	{
		dev_err(mm->dev, "The region of buf->pa 0x%llx not found\n", buf->pa);
		return 0;
	}

	if(mm->has_iommu) {
		mutex_lock(&mm->lock);
		tensor_flush_dcache_range(reg->base_va, (void *)reg->base_va + reg->bytes);
		mutex_unlock(&mm->lock);
	}

	return ret;
}

/**
 * @aipu_mm_cache_invalid() - invald buffer allocated by aipu_mm_alloc()
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @buf:  pointer to the buffer descriptor to be invalid
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_cache_invalid(struct aipu_memory_manager *mm, struct aipu_buf_desc *buf)
{
	int ret = 0;
	struct aipu_mem_region *reg = NULL;

	if (!mm || !buf)
		return -EINVAL;

	reg = aipu_mm_find_region(mm, buf->pa, "invalid");
	if (!reg)
	{
		dev_err(mm->dev, "The region of buf->pa 0x%llx not found\n", buf->pa);
		return 0;
	}

	if(mm->has_iommu) {
		mutex_lock(&mm->lock);
		tensor_dcache_inval_poc(reg->base_va, (void *)reg->base_va + reg->bytes);
		mutex_unlock(&mm->lock);
	}

	return ret;
}

/**
 * @aipu_mm_free_buffers() - free all the buffers allocated from one fd
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @filp: pointer to the file struct
 */
void aipu_mm_free_buffers(struct aipu_memory_manager *mm, struct file *filp)
{
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region_obj *next = NULL;

	if (mm->res_cnt) {
		list_for_each_entry_safe(obj, next, &mm->mem.head->list, list)
			aipu_mm_free_filp_in_region(mm, obj->reg, filp);

		return;
	}

	mutex_lock(&mm->lock);
	list_for_each_entry_safe(obj, next, &mm->mem.head->list, list) {
		if (obj->reg->filp == filp) {
			aipu_mm_direct_free(mm, obj->reg, true);
			/* --- reg should not be used below --- */
		}
	}
	mutex_unlock(&mm->lock);
}

/**
 * @aipu_mm_mmap_buf() - mmap an allocated buffer for user thread
 * @mm: pointer to memory manager struct initialized in aipu_init_mm()
 * @vma: pointer to the vm_area_struct
 * @filp: pointer to the file struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_mmap_buf(struct aipu_memory_manager *mm, struct vm_area_struct *vma, struct file *filp)
{
	int ret = 0;
	dma_addr_t iova = 0;
	struct aipu_mem_region *reg = NULL;

	if (!mm || !vma)
		return -EINVAL;

	iova = vma->vm_pgoff << PAGE_SHIFT;

	reg = aipu_mm_find_region(mm, iova, "mmap");
	if (!reg)
		return -EINVAL;

	vma->vm_pgoff = (iova - reg->base_iova) >> PAGE_SHIFT;

#if KERNEL_VERSION(6, 3, 0) <= LINUX_VERSION_CODE || \
	(defined(__ANDROID_COMMON_KERNEL__) && KERNEL_VERSION(6, 1, 43) <= LINUX_VERSION_CODE)
	vm_flags_set(vma, VM_IO);
#else
	vma->vm_flags |= VM_IO;
#endif
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	ret = dma_mmap_attrs(reg->dev, vma, reg->base_va, reg->base_pa,
			     reg->bytes, reg->attrs);
	if (ret)
		dev_err(mm->dev, "dma_mmap_attrs failed at iova 0x%llx (ret = %d)", iova, ret);

	vma->vm_pgoff = iova >> PAGE_SHIFT;

	return ret;
}

/**
 * @aipu_mm_disable_sram_allocation() - disable buffer allocations from soc sram
 * @mm: pointer to memory manager struct initialized in aipu_init_mm()
 * @filp: pointer to the file struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_disable_sram_allocation(struct aipu_memory_manager *mm, struct file *filp)
{
	int ret = 0;
	struct aipu_mem_region_obj *obj = NULL;
	struct aipu_mem_region *reg = NULL;
	struct aipu_sram_disable_per_fd *sram_disable_per_fd = NULL;

	if (!mm || !mm->res_cnt)
		return -EINVAL;

	mutex_lock(&mm->lock);
	/* If SRAM is under using by driver & AIPU, it cannot be disabled. */
	list_for_each_entry(obj, &mm->mem.head->list, list) {
		reg = obj->reg;

		if (reg->type == AIPU_MEM_REGION_TYPE_SRAM &&
		    !bitmap_empty(reg->bitmap, reg->count)) {
			dev_err(mm->dev, "the SRAM region to be disabled is under using");
			ret = -EPERM;
			break;
		}
	}

	if (!ret) {
		int found = 0;

		list_for_each_entry(sram_disable_per_fd, &mm->sram_disable_head->list, list) {
			if (sram_disable_per_fd->filp == filp) {
				sram_disable_per_fd->cnt++;
				found = 1;
				break;
			}
		}
		if (!found) {
			sram_disable_per_fd = kzalloc(sizeof(*sram_disable_per_fd), GFP_KERNEL);
			if (!sram_disable_per_fd) {
				ret = -ENOMEM;
				goto unlock;
			}
			sram_disable_per_fd->cnt++;
			sram_disable_per_fd->filp = filp;
			list_add(&sram_disable_per_fd->list, &mm->sram_disable_head->list);
		}
		mm->sram_disable++;
	}
unlock:
	mutex_unlock(&mm->lock);
	return ret;
}

/**
 * @aipu_mm_enable_sram_allocation() - enable buffer allocations from soc sram (disabled before)
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @filp: pointer to the file struct
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_enable_sram_allocation(struct aipu_memory_manager *mm, struct file *filp)
{
	int ret = 0;
	struct aipu_sram_disable_per_fd *sram_disable_per_fd = NULL;

	if (!mm || !mm->res_cnt)
		return -EINVAL;

	mutex_lock(&mm->lock);
	if (mm->sram_disable == 0) {
		ret = -EPERM;
		goto unlock;
	}

	list_for_each_entry(sram_disable_per_fd, &mm->sram_disable_head->list, list) {
		if (sram_disable_per_fd->filp == filp) {
			if (sram_disable_per_fd->cnt)
				sram_disable_per_fd->cnt--;
			break;
		}
	}
	mm->sram_disable--;
unlock:
	mutex_unlock(&mm->lock);
	return ret;
}

/**
 * @aipu_mm_get_va() - get the kernel virtual address of an allocated buffer.
 *                     currently do not support non-tcb va from importers.
 * @mm:     pointer to memory manager struct initialized in aipu_init_mm()
 * @dev_pa: device physical address returned by aipu_mm_alloc()
 *
 * Return: 0 on success and error code otherwise.
 */
char *aipu_mm_get_va(struct aipu_memory_manager *mm, u64 dev_pa)
{
	struct aipu_mem_region *reg = NULL;

	if (!mm)
		return NULL;

	reg = aipu_mm_find_region(mm, dev_pa, "get_va");
	if (!reg)
		return NULL;

	return (char *)((unsigned long)reg->base_va + dev_pa - reg->base_iova);
}

/**
 * @aipu_mm_get_tcb() - get the pointer to the TCB of a TCB list.
 * @mm: pointer to memory manager struct initialized in aipu_init_mm()
 * @pa: address of the TCB
 *
 * Return: tcb va on success or NULL otherwise.
 */
struct aipu_tcb *aipu_mm_get_tcb(struct aipu_memory_manager *mm, u64 pa)
{
	struct aipu_mem_region *reg = NULL;
	struct aipu_tcb_buf *tbuf = NULL;
	unsigned long flags;
	struct aipu_tcb *tcb = NULL;

	reg = aipu_mm_find_region(mm, pa, "find_tcb");
	if (!reg)
		return NULL;
	spin_lock_irqsave(&mm->slock, flags);
	tbuf = aipu_mm_find_tbuf_in_region_no_lock(mm, reg, pa);
	if (!tbuf) {
		dev_err(mm->dev, "[get_tcb] no TCB buffer is found at iova 0x%llx", pa);
		goto unlock;
	}

	reg = tbuf->reg;
	tcb = (struct aipu_tcb *)((char *)(reg->base_va) + pa - reg->base_iova);

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
	return tcb;
}

/**
 * @aipu_mm_set_tcb_tail() - set the pointer to the tail TCB of a TCB list.
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @tail: address of the tail TCB of a TCB list
 *
 * Return: 0 on success and error code otherwise.
 */
struct aipu_tcb *aipu_mm_set_tcb_tail(struct aipu_memory_manager *mm, u64 tail)
{
	struct aipu_mem_region *reg = NULL;
	struct aipu_tcb_buf *tbuf = NULL;
	unsigned long flags;
	struct aipu_tcb *ret = NULL;

	spin_lock_irqsave(&mm->slock, flags);
	tbuf = aipu_mm_find_tcb_buf_no_lock(mm, tail, "set_tail");
	if (!tbuf)
		goto unlock;

	reg = tbuf->reg;
	tbuf->tail = tail;
	tbuf->tail_tcb = (struct aipu_tcb *)((char *)(reg->base_va) + tail - reg->base_iova);
	ret = tbuf->tail_tcb;

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
	return ret;
}

/**
 * @aipu_mm_link_tcb() - link a TCB list to the tail of an existing TCB list.
 * @mm:           pointer to memory manager struct initialized in aipu_init_mm()
 * @prev_tail:    address of the tail TCB of a previous TCB list to be linked
 * @next_head_32: head address of the next TCB list
 * @next_job_id:  Job ID of the next TCB list
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_link_tcb(struct aipu_memory_manager *mm, u64 prev_tail, u32 next_head_32,
		     int next_job_id)
{
	struct aipu_hold_tcb_buf *htbuf = NULL;
	unsigned long flags;
	int ret = 0;
	u32 temp = 0;

	htbuf = aipu_mm_get_hold_htbuf(mm, prev_tail);
	if (!htbuf)
		return -EFAULT;

	spin_lock_irqsave(&mm->shlock, flags);
	htbuf->hold_tcb->next = next_head_32;
	temp = htbuf->hold_tcb->next;
	htbuf->next_head = next_head_32;
	if (htbuf->status == AIPU_MEM_HOLD_TYPE_LINK_PREV)
		htbuf->status = AIPU_MEM_HOLD_TYPE_LINKED;
	else if (htbuf->status == AIPU_MEM_HOLD_TYPE_IDLE)
		htbuf->status = AIPU_MEM_HOLD_TYPE_LINK_NEXT;
	else
		dev_info(mm->dev, "ABNORMAL STATUS link hold tcb index %d status %d\n",
			 htbuf->index, htbuf->status);

	dev_dbg(mm->dev, "new tcb task link prev htcb index %d status %d\n",
		htbuf->index, htbuf->status);

	spin_unlock_irqrestore(&mm->shlock, flags);
	return ret;
}

/**
 * @aipu_mm_unlink_tcb() - unlink a TCB list from an existing TCB list.
 * @mm:           pointer to memory manager struct initialized in aipu_init_mm()
 * @prev_tail:    address of the tail TCB of a previous TCB list to be unlinked
 * @free_tcb:     free TCB or not
 *
 * Return: 0 on success and error code otherwise.
 */
int aipu_mm_unlink_tcb(struct aipu_memory_manager *mm, u64 curr_hold, bool free_tcb)
{
	struct aipu_tcb_buf *tbuf = NULL;
	struct aipu_hold_tcb_buf *htbuf = NULL;
	struct aipu_hold_tcb_buf *prev_htbuf = NULL;
	struct aipu_buf_desc buf;
	unsigned long flags;
	int ret = 0;
	u32 temp = 0;
	int prev_status = -1, prev_index = -1;

	htbuf = aipu_mm_get_hold_htbuf(mm, curr_hold);
	if (!htbuf)
		return -EFAULT;
	if (htbuf->prev_hold_tcb != 0) {
		prev_htbuf = aipu_mm_get_hold_htbuf(mm, htbuf->prev_hold_tcb);
		if (!prev_htbuf)
			dev_info(mm->dev, "PREV HOLD TCB 0x%llx IS NOT SET.", htbuf->prev_hold_tcb);
	}
	spin_lock_irqsave(&mm->shlock, flags);

	if (!htbuf->hold_tcb) {
		dev_err(mm->dev, "hold TCB was not set before unlinking it");
		ret = -EINVAL;
		goto unlock;
	}
	if (prev_htbuf) {
		prev_htbuf->hold_tcb->next = 0;
		temp = prev_htbuf->hold_tcb->next;
		if (prev_htbuf->status == AIPU_MEM_HOLD_TYPE_LINK_NEXT) {
			prev_htbuf->status = AIPU_MEM_HOLD_TYPE_IDLE;
		} else if (prev_htbuf->status == AIPU_MEM_HOLD_TYPE_LINKED) {
			prev_htbuf->status = AIPU_MEM_HOLD_TYPE_LINK_PREV;
		} else {
			dev_info(mm->dev, "ABNORMAL UNLINK PREV HOLD TCB STATUS %d\n",
				 prev_htbuf->status);
		}
		htbuf->prev_hold_tcb = 0;
		prev_status = prev_htbuf->status;
		prev_index = prev_htbuf->index;
	}

	tbuf = htbuf->prev_tbuf;
	tbuf->tail_tcb->next = 0;
	tbuf->dep_job_id = 0;
	tbuf->pinned = false;
	temp = tbuf->tail_tcb->next;
	htbuf->prev_head = 0;
	htbuf->prev_tail = 0;
	if (htbuf->status == AIPU_MEM_HOLD_TYPE_LINKED) {
		htbuf->status = AIPU_MEM_HOLD_TYPE_LINK_NEXT;
	} else if (htbuf->status == AIPU_MEM_HOLD_TYPE_LINK_PREV) {
		htbuf->status = AIPU_MEM_HOLD_TYPE_IDLE;
	} else {
		dev_info(mm->dev, "ABNORMAL UNLINK CUR HOLD TCB STATUS %d\n", htbuf->status);
	}
	dev_dbg(mm->dev, "unlink prev hold index %d status %d, "
		"unlink curr hold tcb index %d status %d final holder index %d",
		prev_index, prev_status, htbuf->index, htbuf->status,
		mm->hold_tcb_head->hold_index);

unlock:
	spin_unlock_irqrestore(&mm->shlock, flags);
	if (!ret && free_tcb) {
		memset(&buf, 0, sizeof(buf));
		buf.pa = tbuf->head;
		aipu_mm_free(mm, &buf, NULL, false);
	}
	htbuf->prev_tbuf = NULL;
	return ret;
}

/**
 * @aipu_mm_pin_tcb() - pin a TCB list until it is ready to be freed in a future moment
 * @mm:   pointer to memory manager struct initialized in aipu_init_mm()
 * @tail: address of the tail TCB of a TCB list
 */
void aipu_mm_pin_tcb(struct aipu_memory_manager *mm, u64 tail)
{
	struct aipu_tcb_buf *tbuf = NULL;
	unsigned long flags;

	spin_lock_irqsave(&mm->slock, flags);
	tbuf = aipu_mm_find_tcb_buf_no_lock(mm, tail, NULL);
	if (!tbuf)
		goto unlock;

	tbuf->pinned = true;

unlock:
	spin_unlock_irqrestore(&mm->slock, flags);
}

u64 aipu_mm_get_asid_base(struct aipu_memory_manager *mm, u32 asid)
{
	if (!mm || asid >= ZHOUYI_ASID_COUNT)
		return 0;

	if (mm->res_cnt)
		return mm->ase[asid].base;

	return mm->default_asid_base;
}

u64 aipu_mm_get_asid_size(struct aipu_memory_manager *mm, u32 asid)
{
	if (!mm || asid >= ZHOUYI_ASID_COUNT)
		return 0;

	if (mm->res_cnt)
		return mm->ase[asid].range;

	return mm->default_asid_size;
}

void aipu_mm_get_asid(struct aipu_memory_manager *mm, struct aipu_cap *cap)
{
	int i;
	if (!mm || !cap)
		return;

	for(i = 0; i < mm->valid_asid_cnt; i++){
		cap->asid_base[i] = aipu_mm_get_asid_base(mm, i);
	}
}

u32 aipu_mm_get_asid_cnt(struct aipu_memory_manager *mm)
{
	if (!mm)
		return 0;
	return mm->valid_asid_cnt;
}

int aipu_mm_init_gm(struct aipu_memory_manager *mm, int bytes)
{
	if (!mm || !bytes || mm->gm_policy == AIPU_GM_POLICY_NONE)
		return -EINVAL;

	mm->gm_bytes = bytes;
	return 0;
}

int aipu_mm_gm_policy_switch(struct aipu_memory_manager *mm, enum aipu_gm_policy next)
{
	int ret = 0;

	if (!mm || mm->version != AIPU_ISA_VERSION_ZHOUYI_V3)
		return -EINVAL;

	if (next == mm->gm_policy)
		return ret;

	mutex_lock(&mm->lock);
	mm->gm_policy = next;
	mutex_unlock(&mm->lock);
	return ret;
}

void aipu_mm_get_gm(struct aipu_memory_manager *mm, struct aipu_cap *cap)
{
	if (!mm || !cap)
		return;

	cap->gm0_size = 0;
	cap->gm1_size = 0;

	mutex_lock(&mm->lock);
	if (mm->gm_policy == AIPU_GM_POLICY_SHARED) {
		cap->gm0_size = mm->gm_bytes;
	} else {
		cap->gm0_size = mm->gm_bytes >> 1;
		cap->gm1_size = cap->gm0_size;
	}
	mutex_unlock(&mm->lock);
}

void get_dtcm(struct aipu_memory_manager *mm, u64 *base, u32 *size)
{
	struct aipu_mem_region *reg = NULL;
	struct aipu_mem_region_obj *obj = NULL;

	if (!mm || !base || !size || !mm->res_cnt)
		return;

	/* no lock because we only support reserved regions for DTCM */
	list_for_each_entry(obj, &mm->mem.head->list, list) {
		reg = obj->reg;
		if (reg->type == AIPU_MEM_REGION_TYPE_DTCM) {
			*base = reg->base_iova;
			*size = reg->bytes;
			return;
		}
	}

	*base = 0;
	*size = 0;
}
