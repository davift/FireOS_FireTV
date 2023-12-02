#ifndef _LINUX_KASAN_H
#define _LINUX_KASAN_H

#include <linux/sched.h>
#include <linux/types.h>

struct kmem_cache;
struct page;
struct vm_struct;

#ifdef CONFIG_KASAN

#define KASAN_SHADOW_SCALE_SHIFT 3

#include <asm/kasan.h>
#include <asm/pgtable.h>

extern unsigned char kasan_zero_page[PAGE_SIZE];
extern pte_t kasan_zero_pte[PTRS_PER_PTE];
extern pmd_t kasan_zero_pmd[PTRS_PER_PMD];
extern pud_t kasan_zero_pud[PTRS_PER_PUD];
extern bool kasan_initialized;

void kasan_populate_zero_shadow(const void *shadow_start,
				const void *shadow_end);

static inline void *kasan_mem_to_shadow(const void *addr)
{
	return (void *)((unsigned long)addr >> KASAN_SHADOW_SCALE_SHIFT)
		+ KASAN_SHADOW_OFFSET;
}

/* Enable reporting bugs after kasan_disable_current() */
extern void kasan_enable_current(void);

/* Disable reporting bugs for current task */
extern void kasan_disable_current(void);

void kasan_unpoison_shadow(const void *address, size_t size);

void kasan_unpoison_task_stack(struct task_struct *task);
void kasan_unpoison_stack_above_sp_to(const void *watermark);

void kasan_alloc_pages(struct page *page, unsigned int order);
void kasan_free_pages(struct page *page, unsigned int order);

void kasan_cache_create(struct kmem_cache *cache, size_t *size,
			unsigned long *flags);
void kasan_cache_shrink(struct kmem_cache *cache);
void kasan_cache_shutdown(struct kmem_cache *cache);

void kasan_poison_slab(struct page *page);
void kasan_unpoison_object_data(struct kmem_cache *cache, void *object);
void kasan_poison_object_data(struct kmem_cache *cache, void *object);
void kasan_init_slab_obj(struct kmem_cache *cache, const void *object);

void kasan_kmalloc_large(const void *ptr, size_t size, gfp_t flags);
void kasan_kfree_large(const void *ptr);
void kasan_poison_kfree(void *ptr);
void kasan_kmalloc(struct kmem_cache *s, const void *object, size_t size,
		  gfp_t flags);
void kasan_krealloc(const void *object, size_t new_size, gfp_t flags);

void kasan_slab_alloc(struct kmem_cache *s, void *object, gfp_t flags);
bool kasan_slab_free(struct kmem_cache *s, void *object);

struct kasan_cache {
	int alloc_meta_offset;
	int free_meta_offset;
};

int kasan_module_alloc(void *addr, size_t size);
void kasan_free_shadow(const struct vm_struct *vm);

size_t ksize(const void *);
static inline void kasan_unpoison_slab(const void *ptr) { ksize(ptr); }
size_t kasan_metadata_size(struct kmem_cache *cache);

bool kasan_save_enable_multi_shot(void);
void kasan_restore_multi_shot(bool enabled);

#else /* CONFIG_KASAN */

static inline void kasan_unpoison_shadow(const void *address, size_t size) {}

static inline void kasan_unpoison_task_stack(struct task_struct *task) {}
static inline void kasan_unpoison_stack_above_sp_to(const void *watermark) {}

static inline void kasan_enable_current(void) {}
static inline void kasan_disable_current(void) {}

static inline void kasan_alloc_pages(struct page *page, unsigned int order) {}
static inline void kasan_free_pages(struct page *page, unsigned int order) {}

static inline void kasan_cache_create(struct kmem_cache *cache,
				      size_t *size,
				      unsigned long *flags) {}
static inline void kasan_cache_shrink(struct kmem_cache *cache) {}
static inline void kasan_cache_shutdown(struct kmem_cache *cache) {}

static inline void kasan_poison_slab(struct page *page) {}
static inline void kasan_unpoison_object_data(struct kmem_cache *cache,
					void *object) {}
static inline void kasan_poison_object_data(struct kmem_cache *cache,
					void *object) {}
static inline void kasan_init_slab_obj(struct kmem_cache *cache,
				const void *object) {}

static inline void kasan_kmalloc_large(void *ptr, size_t size, gfp_t flags) {}
static inline void kasan_kfree_large(const void *ptr) {}
static inline void kasan_poison_kfree(void *ptr) {}
static inline void kasan_kmalloc(struct kmem_cache *s, const void *object,
				size_t size, gfp_t flags) {}
static inline void kasan_krealloc(const void *object, size_t new_size,
				 gfp_t flags) {}

static inline void kasan_slab_alloc(struct kmem_cache *s, void *object,
				   gfp_t flags) {}
static inline bool kasan_slab_free(struct kmem_cache *s, void *object)
{
	return false;
}

static inline int kasan_module_alloc(void *addr, size_t size) { return 0; }
static inline void kasan_free_shadow(const struct vm_struct *vm) {}

static inline void kasan_unpoison_slab(const void *ptr) { }
static inline size_t kasan_metadata_size(struct kmem_cache *cache) { return 0; }

#endif /* CONFIG_KASAN */

#endif /* LINUX_KASAN_H */