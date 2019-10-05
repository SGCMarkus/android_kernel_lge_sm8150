#define pr_fmt(fmt) "VOTER: %s: " fmt, __func__
#define pr_voter(fmt, ...) pr_info(fmt, ##__VA_ARGS__)
#define pr_dbg_voter(fmt, ...) pr_debug(fmt, ##__VA_ARGS__)

#include "veneer-primitives.h"

#define PASSOVER_DISABLED -1

/* DO NOT export voter_list */
struct voter_list {
	struct list_head head;
	struct mutex	 lock;
	enum voter_type	 type;
	int 		 passover;
};

// at this time, we have 5 voter classes for IUSB, IBAT, IDC, VFLOAT and HVDCP.
// If you need to add other voter class, add here and update the total list
// for convenient iteration.
static struct voter_list voter_list_iusb = {
	.head = LIST_HEAD_INIT(voter_list_iusb.head),
	.lock = __MUTEX_INITIALIZER(voter_list_iusb.lock),
	.type = VOTER_TYPE_IUSB,
	.passover = PASSOVER_DISABLED,
};
static struct voter_list voter_list_ibat = {
	.head = LIST_HEAD_INIT(voter_list_ibat.head),
	.lock = __MUTEX_INITIALIZER(voter_list_ibat.lock),
	.type = VOTER_TYPE_IBAT,
	.passover = PASSOVER_DISABLED,
};
static struct voter_list voter_list_idc = {
	.head = LIST_HEAD_INIT(voter_list_idc.head),
	.lock = __MUTEX_INITIALIZER(voter_list_idc.lock),
	.type = VOTER_TYPE_IDC,
	.passover = PASSOVER_DISABLED,
};
static struct voter_list voter_list_vfloat = {
	.head = LIST_HEAD_INIT(voter_list_vfloat.head),
	.lock = __MUTEX_INITIALIZER(voter_list_vfloat.lock),
	.type = VOTER_TYPE_VFLOAT,
	.passover = PASSOVER_DISABLED,
};
static struct voter_list voter_list_hvdcp = {
	.head = LIST_HEAD_INIT(voter_list_hvdcp.head),
	.lock = __MUTEX_INITIALIZER(voter_list_hvdcp.lock),
	.type = VOTER_TYPE_HVDCP,
	.passover = PASSOVER_DISABLED,
};

static struct voter_list* voter_lists [] = {
	&voter_list_iusb, &voter_list_ibat, &voter_list_idc, &voter_list_vfloat, &voter_list_hvdcp, NULL,
};
static void (*voter_callback)(enum voter_type type, int limit);

///////////////////////////////////////////////////////////////////////////////
// Helper functions
///////////////////////////////////////////////////////////////////////////////
static struct voter_list* 	voter_list_get_by_type(enum voter_type type);
static struct voter_list* 	voter_list_get_by_entry(struct voter_entry* entry);
static int 			atomic_voter_id(void);
static void 			atomic_voter_callback(struct voter_list* list_of);
static int 			atomic_voter_effected(struct voter_list* list_of);
///////////////////////////////////////////////////////////////////////////////

static struct voter_list* voter_list_get_by_type(enum voter_type type) {
	int index;

	switch (type) {
	case VOTER_TYPE_IUSB:
		index = 0;
		break;
	case VOTER_TYPE_IBAT:
		index = 1;
		break;
	case VOTER_TYPE_IDC:
		index = 2;
		break;
	case VOTER_TYPE_VFLOAT:
		index = 3;
		break;
	case VOTER_TYPE_HVDCP:
		index = 4;
		break;
	default: pr_voter("Invalid limit voter type %d\n", type);
		index = ARRAY_SIZE(voter_lists) - 1;
		break;
	}

	return voter_lists[index];
}

static struct voter_list* voter_list_get_by_entry(struct voter_entry* entry) {
	return entry ? voter_list_get_by_type(entry->type) : NULL;
}

static int atomic_voter_effected(struct voter_list* list_of) {
	int effecting_limit = VOTE_TOTALLY_RELEASED;
	struct voter_entry* iter;

	list_for_each_entry(iter, &list_of->head, node) {
		if (iter->limit < effecting_limit) {
			effecting_limit = iter->limit;
		}
	}

	return effecting_limit;
}

static int atomic_voter_id(void) {
	static int voter_id = 0;
	return voter_id++;
}

static void atomic_voter_callback(/* @Nonnull */ struct voter_list* list) {
	struct voter_entry*	iter;
	enum voter_type		type = list->type;
	int			limit;
	char name[1024] = {0, };

	if (!voter_callback) {
		pr_voter("unified voter is not initiated\n");
		return;
	}

	if (list->passover == PASSOVER_DISABLED) {
		limit = atomic_voter_effected(list);
		list_for_each_entry(iter, &list->head, node) {
			if (iter->limit == limit && limit != VOTE_TOTALLY_RELEASED) {
				if (strlen(name) > 0)
					strcat(name, ", ");
				strcat(name, iter->name);

				iter->effecting = true;
			}
			else
				iter->effecting = false;
		}

		if (limit != VOTE_TOTALLY_RELEASED)
			pr_voter("%s : Voter %s effected (%d)\n",
				vote_title(list->type), name, limit);
	}
	else {
		limit = list->passover;
		list_for_each_entry(iter, &list->head, node) {
			iter->effecting = false;
		}

		pr_voter("%s : Voting %d is set to passover\n",
			vote_title(list->type), limit);
	}

	voter_callback(type, limit);
}

struct voter_entry* veneer_voter_search(enum voter_type type, const char* name) {
	struct voter_list* list_of = voter_list_get_by_type(type);
	struct voter_entry* iter;

	if (list_of) {
		list_for_each_entry(iter, &list_of->head, node) {
			if (!strcmp(iter->name, name)) {
				return iter;
			}
		}
	}

	return NULL;
}

enum charging_suspended veneer_voter_suspended(enum voter_type type) {
	struct voter_list*	list = voter_list_get_by_type(type);
	enum charging_suspended	ret = CHARGING_NOT_SUSPENDED;
	struct voter_entry* iter;

	if (list) {
		list_for_each_entry(iter, &list->head, node) {
			if (iter->limit == VOTE_TOTALLY_BLOCKED) {
				if (iter->fakeui) {
					ret = CHARGING_SUSPENDED_WITH_FAKE_OFFLINE;
					break;
				}
				else
					ret = CHARGING_SUSPENDED_WITH_ONLINE;
			}
		}
	}

	return ret;
}

void veneer_voter_passover(enum voter_type type, int limit, bool enable) {
	struct voter_list* list = voter_list_get_by_type(type);
	int passover;

	if (list) {
		/* '!enable' causes rerunning election finally */
		passover = enable ? limit : PASSOVER_DISABLED;

		mutex_lock(&list->lock);
		if (list->passover != passover) {
			list->passover = passover;
			atomic_voter_callback(list);
		}
		mutex_unlock(&list->lock);
	}
}

bool veneer_voter_register(struct voter_entry* entry, const char* name, enum voter_type type, bool fakeui) {
	struct voter_list* list_to = voter_list_get_by_type(type);

	if (!veneer_voter_search(type, name)) {
		if (type == VOTER_TYPE_VFLOAT && fakeui) {
			pr_voter("suspending on vfloat is not permitted...\n");
		}
		if (type == VOTER_TYPE_HVDCP && fakeui) {
			pr_voter("voting hvdcp has no UI...\n");
		}

		// Building constants
		strlcpy(entry->name, name, VOTER_NAME_LENGTH);
		entry->type = type;
		entry->fakeui = fakeui;

		// Building runtime default variables
		entry->limit = VOTE_TOTALLY_RELEASED;
		entry->effecting = false;

		// Building private members
		mutex_lock(&list_to->lock);
		entry->id = atomic_voter_id();
		list_add(&entry->node, &list_to->head);
		mutex_unlock(&list_to->lock);

		return true;
	}
	else {
		pr_voter("duplicated registering %s\n", name);
		return false;
	}
}

void veneer_voter_unregister(struct voter_entry* entry) {
	struct voter_list* list = voter_list_get_by_entry(entry);

	if (entry && list) {
		mutex_lock(&list->lock);
		list_del(&entry->node);
		mutex_unlock(&list->lock);
	}
	else
		pr_voter("invalid voter to unregister\n");
}

void veneer_voter_set(struct voter_entry* voter, int limit) {
	struct voter_list* list_of;

	if (!voter) {
		pr_voter("voter is NULL\n");
		return;
	}

	if (voter->limit == limit) {
		if (limit != VOTE_TOTALLY_RELEASED)
			pr_dbg_voter("voting values(%d) are same for %s, %s\n",
				limit, voter->name, vote_title(voter->type));
		return;
	}

	list_of = voter_list_get_by_entry(voter);
	if (list_of) {
		mutex_lock(&list_of->lock);
		voter->limit = limit;
		atomic_voter_callback(list_of);
		mutex_unlock(&list_of->lock);
	}
	else
		pr_voter("Couldn't find the list of %s\n", voter->name);
}

void veneer_voter_rerun(struct voter_entry* voter) {
	struct voter_list* list_of;

	if (!voter) {
		pr_voter("voter is NULL\n");
		return;
	}

	list_of = voter_list_get_by_entry(voter);
	if (list_of) {
		mutex_lock(&list_of->lock);
		atomic_voter_callback(list_of);
		mutex_unlock(&list_of->lock);
	}
	else
		pr_voter("Couldn't find the list of %s\n", voter->name);
}

void veneer_voter_release(struct voter_entry* voter) {
	veneer_voter_set(voter, VOTE_TOTALLY_RELEASED);
}

bool veneer_voter_effecting(struct voter_entry* voter) {
	return voter->effecting == true;
}

bool veneer_voter_enabled(struct voter_entry* voter) {
	return voter->limit != VOTE_TOTALLY_RELEASED;
}

bool veneer_voter_create(void (*back_veneer_voter)(enum voter_type type, int limit)) {
	if (back_veneer_voter == NULL) {
		pr_voter("callback should not be null\n");
		return false;
	}
	else {
		voter_callback = back_veneer_voter;
		return true;
	}
}

void veneer_voter_destroy(void) {
	voter_callback = NULL;
}
