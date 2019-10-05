/* This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "SYMLNK: %s: " fmt, __func__
#define pr_symlink(reason, fmt, ...)			\
do {							\
	if (pr_debugmask & (reason))			\
		pr_info(fmt, ##__VA_ARGS__);		\
	else						\
		pr_debug(fmt, ##__VA_ARGS__);		\
} while (0)

static int pr_debugmask;

#include <linux/of.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>

#include "veneer-primitives.h"

#define UNIFIED_SYSFS_PROPERTY	"lge,symlink-map"
#define UNIFIED_SYSFS_ROOT	"lge_power"

struct sysfs_group {
	struct list_head node;
	const char* name;
	struct proc_dir_entry* pde;
};

struct sysfs_map {
	const char*		group;
	const char*		symlink;
	const char*		source;

	struct proc_dir_entry*	handle;
};

static struct proc_dir_entry* sysfs_root;

static struct proc_dir_entry* group_search(struct proc_dir_entry* root,
	struct list_head* groups, const char* name) {

	struct sysfs_group* group;
	list_for_each_entry(group, groups, node) {
		if (!strcmp(group->name, name))
			goto success;
	}

	group = kzalloc(sizeof(struct sysfs_group), GFP_KERNEL);
	if (!group) {
		pr_symlink(ERROR, "ERROR to alloc memory for sysfs_group\n");
		goto error;
	}
	group->pde = proc_mkdir(name, root);
	if (!group->pde) {
		pr_symlink(ERROR, "ERROR to mkdir a group, %s\n", name);
		goto error;
	}

	group->name = name;
	list_add(&group->node, groups);
success:
	return group->pde;

error:
	kfree(group);
	return NULL;
}

static void group_free(struct list_head* groups) {
	struct sysfs_group* iter;
	struct sysfs_group* safe;

	list_for_each_entry_safe(iter, safe, groups, node) {
		list_del(&iter->node);
		kfree(iter);
	}
}

static bool sysfs_array(struct device_node* dnode, struct sysfs_map* array, int count) {
	int index, i=0; // 'i' is for the foreach iteration

	for (index=0; index<count; ++index) {
		if (of_property_read_string_index(dnode, UNIFIED_SYSFS_PROPERTY,
				i++, &array[index].group) ||
			of_property_read_string_index(dnode, UNIFIED_SYSFS_PROPERTY,
				i++, &array[index].symlink) ||
			of_property_read_string_index(dnode, UNIFIED_SYSFS_PROPERTY,
				i++, &array[index].source)) {

			pr_symlink(ERROR, "ERROR get %ith string\n", i);
			return false;
		}
	}

	for (index=0; index<count; ++index)
		pr_symlink(VERBOSE, "get %dth node is %s, %s, %s\n", index,
				array[index].group, array[index].symlink,
				array[index].source);

	return true;
}

static bool sysfs_mount(struct sysfs_map* array, int count) {
	bool	ret = false;
	int	index;

	struct list_head	groups = LIST_HEAD_INIT(groups);
	struct proc_dir_entry*	root = proc_mkdir(UNIFIED_SYSFS_ROOT, NULL);
	struct proc_dir_entry* sysfs_group;

	if (root != NULL) {
		for (index=0; index<count; index++) {
			if (!strcmp(array[index].source, "NULL")) {
				pr_symlink(VERBOSE, "%s user node didn't have kernel node\n",
					array[index].symlink);
				continue;
			}

			sysfs_group = group_search(root, &groups, array[index].group);
			if (sysfs_group == NULL) {
				pr_symlink(ERROR, "ERROR making group '%s'\n",
					array[index].group);
				goto out;
			}

			array[index].handle = proc_symlink(array[index].symlink, sysfs_group, array[index].source);
			if (!array[index].handle) {
				pr_symlink(ERROR, "ERROR making symlink '%s'\n",
					array[index].symlink);
				goto out;
			}
		}

		ret = true;
	}
	else {
		pr_symlink(ERROR, "ERROR making root sysfs\n");
	}

	sysfs_root = root; // Update global variable here.
out :	group_free(&groups);
	return ret;
}

bool unified_sysfs_create(struct device_node* dnode) {
	bool			ret = false;
	int			count = of_property_count_strings(dnode, UNIFIED_SYSFS_PROPERTY) / 3;
	struct sysfs_map*	array = kzalloc(count * sizeof(struct sysfs_map), GFP_KERNEL);
	pr_debugmask = ERROR | UPDATE;

	if (count <= 0 || array == NULL) {
		pr_symlink(ERROR, "Failed to parsing device tree for unified-sysfs\n");
		goto out;
	}

	if (!sysfs_array(dnode, array, count)) {
		pr_symlink(ERROR, "Failed to make array for unified-sysfs\n");
		goto out;
	}

	if (!sysfs_mount(array, count)) {
		pr_symlink(ERROR, "Failed to make array for unified-sysfs\n");
		goto out;
	}

	pr_symlink(UPDATE, "Created.\n");
	ret = true;

out:
	kfree(array);
	return ret;
}

void unified_sysfs_destroy(void) {
	proc_remove(sysfs_root);
	pr_symlink(UPDATE, "Destroyed.\n");
}
