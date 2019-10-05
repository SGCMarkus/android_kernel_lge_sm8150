#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <net/patchcodeid.h>

void patch_code_id(const char* log){
// Do nothing. This API is only for checking patch code ID.
if (log == NULL) {} // dummy code.
}

